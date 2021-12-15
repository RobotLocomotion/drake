#include <memory>
#include <utility>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/pixel_types.h"
#include "drake/systems/sensors/rgbd_sensor.h"

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");
DEFINE_bool(render_on, true, "Sets rendering generally enabled (or not)");
DEFINE_bool(color, true, "Sets the enabled camera to render color");
DEFINE_bool(depth, true, "Sets the enabled camera to render depth");
DEFINE_bool(label, true, "Sets the enabled camera to render label");
DEFINE_double(render_fps, 10, "Frames per simulation second to render");
DEFINE_string(camera_xyz_rpy, "0.8, 0.0, 0.5, -2.2, 0.0, 1.57",
    "Sets the camera pose by xyz (meters) and rpy (radians) values.");

namespace drake {
namespace geometry {
namespace render {
namespace minimal_example {
namespace {

/** This example serves as the baseline demonstration of all the features
 RenderEngineVtk currently can exercise. The goal is to exhaust different
 functionalities of rendering color, depth, and label images via a purposely
 designed scene with various rendering settings. Below is a checklist
 enumerating each feature we plan to exercise in this example(s).

   X Render each supported shape type (see shape_specification.h)
     X Almost resolved by AddShapes() (see below and resolve TODO).
     X Once with arbitrary RGBA color (unique for each object)
     X Once with diffuse texture
       - Textures should have asymmetric features so that the application of
         the texture is clearly correct -- not flipped, rotated, etc.
   - Mesh texture specified in two different ways
     - obj/png file name replacement
     - Specified in PerceptionProperties (this is the *only* mechanism for
       applying a diffuse texture to other shapes)
   - Camera functionality
     - Exercise full intrinsic and extrinsic properties. Two separate cameras
       would be sufficient.
   X Moving objects (showing object pose updated -- this should include
      translation and rotation) (see mustard bottle)
   - Clipping range across all output image types
     - Make sure there are objects at the limits of the clipping range that
       will get clipped (such that the image would be different if they
       weren't clipped).
   - Depth range
     - Similar to clipping range -- make sure we have objects near the limits
       of the depth range such that if the depth range were incorrect, we'd
       produce a different image.
   - Render labels
     - Exercise all five types of labels (empty, don't care, don't render,
       unspecified, user-defined).
   - (optional) Confirm that occluding *transparent* objects are rendered the
     same. */

using Eigen::Vector3d;
using lcm::DrakeLcm;
using math::RigidTransformd;
using math::RollPitchYawd;
using multibody::MultibodyPlant;
using multibody::Parser;
using systems::Context;
using systems::InputPort;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::PixelType;
using systems::sensors::RgbdSensor;

struct Material {
  // For convenience, create a handful of *implicit* conversion constructors.
  // NOLINTNEXTLINE(runtime/explicit)
  Material(const Rgba rgba_in) : rgba(rgba_in) {}
  // NOLINTNEXTLINE(runtime/explicit)
  Material(const std::string& diffuse_map_in) : diffuse_map(diffuse_map_in) {}

  RenderLabel label{RenderLabel::kDontCare};
  Rgba rgba{1, 1, 1, 1};
  std::string diffuse_map;
};

RigidTransformd ParseCameraPose(const std::string& raw_input_str) {
  std::string input_str = raw_input_str;
  std::string delimiter = ", ";
  std::vector<double> xyzrpy_numeric;

  size_t pos = 0;
  std::string token;
  while ((pos = input_str.find(delimiter)) != std::string::npos) {
    token = input_str.substr(0, pos);
    input_str.erase(0, pos + delimiter.length());
    xyzrpy_numeric.push_back(std::stod(token));
  }
  xyzrpy_numeric.push_back(std::stod(input_str));
  DRAKE_DEMAND(xyzrpy_numeric.size() == 6);

  const RigidTransformd X_WB(
      RollPitchYawd{xyzrpy_numeric[3], xyzrpy_numeric[4], xyzrpy_numeric[5]},
      Vector3d(xyzrpy_numeric[0], xyzrpy_numeric[1], xyzrpy_numeric[2]));

  return X_WB;
}

// Make an instance of the given shape, at the given position, with the given
// material (named as indicated).
std::unique_ptr<GeometryInstance> MakeInstance(const Shape& shape,
                                               const Vector3d& p_WS,
                                               const Material& mat,
                                               const std::string& name) {
  auto instance = std::make_unique<GeometryInstance>(RigidTransformd(p_WS),
                                                     shape.Clone(), name);

  PerceptionProperties percep_props;
  percep_props.AddProperty("label", "id", mat.label);
  percep_props.AddProperty("phong", "diffuse", mat.rgba);
  if (!mat.diffuse_map.empty()) {
    percep_props.AddProperty("phong", "diffuse_map", mat.diffuse_map);
  }
  instance->set_perception_properties(percep_props);

  IllustrationProperties illus_props;
  // NOTE: DrakeVisualizer doesn't get texture information for arbitrary
  // primitives; they will only appear textured in the renderer.
  illus_props.AddProperty("phong", "diffuse", mat.rgba);
  instance->set_illustration_properties(illus_props);
  return instance;
}

// This adds shapes to the world according the required list above. It handles
// adding a colored and textured version of each shape (if it is supported).
void AddShapes(SceneGraph<double>* scene_graph) {
  DRAKE_DEMAND(scene_graph != nullptr);

  const SourceId source_id = scene_graph->RegisterSource("main");
  const std::string texture_path =
      FindResourceOrThrow("drake/geometry/render/dev/4_color_texture.png");
  double x = 0.4;
  double dx = -0.15;

  const Box box(0.1, 0.075, 0.05);
  scene_graph->RegisterAnchoredGeometry(
      source_id, MakeInstance(box, Vector3d(x, -0.25, 0), Rgba(1.0, 0.25, 0.25),
                              "rgba_box"));
  scene_graph->RegisterAnchoredGeometry(
      source_id,
      MakeInstance(box, Vector3d(x, 0.25, 0), texture_path, "texture_box"));
  x += dx;

  const Capsule capsule(0.05, 0.1);
  scene_graph->RegisterAnchoredGeometry(
      source_id, MakeInstance(capsule, Vector3d(x, -0.25, 0),
                              Rgba(1.0, 1.0, 0.25), "rgba_capsule"));
  // NOTE: Apparently we don't have texture coordinates for capsules.
  scene_graph->RegisterAnchoredGeometry(
      source_id, MakeInstance(capsule, Vector3d(x, 0.25, 0), texture_path,
                              "texture_capsule"));
  x += dx;

  const Cylinder cylinder(0.05, 0.1);
  scene_graph->RegisterAnchoredGeometry(
      source_id, MakeInstance(cylinder, Vector3d(x, -0.25, 0),
                              Rgba(0.25, 1.0, 0.25), "rgba_cylinder"));
  scene_graph->RegisterAnchoredGeometry(
      source_id, MakeInstance(cylinder, Vector3d(x, 0.25, 0), texture_path,
                              "texture_cylinder"));
  x += dx;

  const Ellipsoid ellipsoid(0.05, 0.025, 0.0375);
  scene_graph->RegisterAnchoredGeometry(
      source_id, MakeInstance(ellipsoid, Vector3d(x, -0.25, 0),
                              Rgba(0.25, 1.0, 1.0), "rgba_ellipsoid"));
  scene_graph->RegisterAnchoredGeometry(
      source_id, MakeInstance(ellipsoid, Vector3d(x, 0.25, 0), texture_path,
                              "texture_ellipsoid"));
  x += dx;

  const Sphere sphere(0.05);
  scene_graph->RegisterAnchoredGeometry(
      source_id, MakeInstance(sphere, Vector3d(x, -0.25, 0),
                              Rgba(0.25, 0.25, 1.0), "rgba_sphere"));
  scene_graph->RegisterAnchoredGeometry(
      source_id, MakeInstance(sphere, Vector3d(x, 0.25, 0), texture_path,
                              "texture_sphere"));
  x += dx;

  // We also need to add Mesh, Convex, and HalfSpace.
}

int do_main() {
  // Declare builder, plant, and scene_graph.
  systems::DiagramBuilder<double> builder;
  MultibodyPlant<double>* plant = nullptr;
  SceneGraph<double>* scene_graph = nullptr;

  std::tie(plant, scene_graph) =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);

  // Add RenderEngineVtk with default params.
  scene_graph->set_name("scene_graph");
  const std::string render_name("renderer");
  scene_graph->AddRenderer(render_name,
                           MakeRenderEngineVtk(RenderEngineVtkParams()));

  AddShapes(scene_graph);

  // Add a single body (using a textured mesh) into a MultibodyPlant. We'll
  // use MBP to *move* the object (by giving it an initial velocity; we don't
  // want to have to wait for gravity to take effect to observe a difference
  // in position.
  Parser parser{plant};
  const auto mustard_bottle = parser.AddModelFromFile(
      FindResourceOrThrow(
          "drake/manipulation/models/ycb/sdf/006_mustard_bottle.sdf"),
      "mustard_bottle");

  DrakeLcm lcm;
  DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, &lcm);
  if (FLAGS_render_on) {
    // Create the camera and pose it to face the objects.
    const ColorRenderCamera color_camera{
        {render_name, {640, 480, M_PI_4}, {0.01, 10.0}, {}}, false};
    const DepthRenderCamera depth_camera{color_camera.core(), {0.01, 10.0}};
    const RigidTransformd X_WB = ParseCameraPose(FLAGS_camera_xyz_rpy);

    auto world_id = plant->GetBodyFrameIdOrThrow(plant->world_body().index());
    auto camera = builder.AddSystem<RgbdSensor>(
        world_id, X_WB, color_camera, depth_camera);
    builder.Connect(scene_graph->get_query_output_port(),
                    camera->query_object_input_port());

    // Broadcast the images.
    // Publishing images to drake visualizer
    auto image_to_lcm_image_array =
        builder.template AddSystem<systems::sensors::ImageToLcmImageArrayT>();
    image_to_lcm_image_array->set_name("converter");

    systems::lcm::LcmPublisherSystem* image_array_lcm_publisher{nullptr};
    if ((FLAGS_color || FLAGS_depth || FLAGS_label)) {
      image_array_lcm_publisher =
          builder.template AddSystem(systems::lcm::LcmPublisherSystem::Make<
              lcmt_image_array>(
              "DRAKE_RGBD_CAMERA_IMAGES", &lcm,
              1. / FLAGS_render_fps /* publish period */));
      image_array_lcm_publisher->set_name("publisher");

      builder.Connect(
          image_to_lcm_image_array->image_array_t_msg_output_port(),
          image_array_lcm_publisher->get_input_port());
    }

    if (FLAGS_color) {
      const auto& port =
          image_to_lcm_image_array->DeclareImageInputPort<PixelType::kRgba8U>(
              "color");
      builder.Connect(camera->color_image_output_port(), port);
    }

    if (FLAGS_depth) {
      const auto& port =
          image_to_lcm_image_array
              ->DeclareImageInputPort<PixelType::kDepth32F>("depth");
      builder.Connect(camera->depth_image_32F_output_port(), port);
    }

    if (FLAGS_label) {
      const auto& port =
          image_to_lcm_image_array
              ->DeclareImageInputPort<PixelType::kLabel16I>("label");
      builder.Connect(camera->label_image_output_port(), port);
    }
  }

  plant->Finalize();
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  auto& context = static_cast<systems::DiagramContext<double>&>(
      simulator.get_mutable_context());
  auto& plant_context = diagram->GetMutableSubsystemContext(*plant, &context);

  // Initialize the moving bottle's position and speed so we can observe motion.
  // The mustard bottle spins while climbing slightly.
  plant->mutable_gravity_field().set_gravity_vector({0, 0, 0});
  const auto& mustard_body =
      plant->GetBodyByName("base_link_mustard", mustard_bottle);
  const RigidTransformd X_WMustardBottle(RollPitchYawd{-M_PI / 2, 0, -M_PI / 2},
                                         Vector3d(0, 0, 0));
  plant->SetFreeBodyPose(&plant_context, mustard_body, X_WMustardBottle);
  const multibody::SpatialVelocity<double> V_WMustardBottle(
      Vector3d{0.6, 0, 0}, Vector3d{0, 0, 0.1});
  plant->SetFreeBodySpatialVelocity(&plant_context, mustard_body,
                                    V_WMustardBottle);

  simulator.get_mutable_integrator().set_maximum_step_size(0.002);
  simulator.set_target_realtime_rate(1.f);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace minimal_example
}  // namespace render
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::render::minimal_example::do_main();
}
