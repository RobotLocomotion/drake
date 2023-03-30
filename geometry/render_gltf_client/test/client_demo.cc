/* This example serves as the baseline demonstration of all the features a
 Drake RenderEngine currently can exercise.  More specifically, users of this
 program can use either RenderEngineVtk or RenderEngineGltfClient to render
 color, depth, or label images from an example scene.

 If RenderEngineGltfClient is in use, a rendering server is also needed to
 render images.  Please refer to the README.md in this folder for instructions
 to launch a rendering server.

 The goal of the the example scene is to exhaust different functionalities of
 rendering color, depth, and label images via a purposely designed scene with
 various rendering settings. Below is a checklist enumerating each feature we
 plan to exercise in this example.

 TODO(zachfang): Exercise the remaining rendering features on the list.
 Note: "-" means "not yet complete" and "X" means "complete".
   X Render each supported shape type (see shape_specification.h)
     X Almost resolved by `example_scene.sdf` (see TODO in the .sdf file).
     X Once with arbitrary RGBA color (unique for each object)
     X Once with diffuse texture
       - Textures should have asymmetric features so that the application of
         the texture is clearly correct -- not flipped, rotated, etc.
   - Mesh texture specified in two different ways
     X obj/png file name replacement
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

#include <filesystem>
#include <memory>
#include <utility>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/render_gltf_client/factory.h"
#include "drake/geometry/render_vtk/factory.h"
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
#include "drake/systems/sensors/image_writer.h"
#include "drake/systems/sensors/pixel_types.h"
#include "drake/systems/sensors/rgbd_sensor.h"

using drake::geometry::RenderEngineGltfClientParams;

// TODO(jwnimmer-tri) This is way too much configuration data to parse using
// gflags. Rewrite to use YAML input, instead.
DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");
DEFINE_bool(color, true, "Sets the enabled camera to render color");
DEFINE_bool(depth, true, "Sets the enabled camera to render depth");
DEFINE_bool(label, true, "Sets the enabled camera to render label");
DEFINE_double(render_fps, 10, "Frames per simulation second to render");

/* Helpful viewpoints:
 Front (default): "0.8, 0.0, 0.5, -2.2, 0.0, 1.57"
 Top: "0.0, 0.0, 1.5, -3.14, 0.0, 1.57"
 Right: "0.0, 1.0, 0.0, 1.57, 3.14, 0.0"
 Left: "0.0, -1.0, 0.0, -1.57, 0.0, 0.0"
 Behind: "-0.8, 0.0, 0.5, -2.2, 0.0, -1.57"
 Cylinder, Sphere Close-Ups:
   Diffuse: "0.0, 0.0, 0.0, 1.57, 3.14, 0.0"
   Textured: "0.0, 0.0, 0.0, -1.57, 0.0, 0.0"
 */
DEFINE_string(camera_xyz_rpy, "0.8, 0.0, 0.5, -2.2, 0.0, 1.57",
              "Sets the camera pose by xyz (meters) and rpy (radians) values.");
DEFINE_string(
    save_dir, "",
    "If not empty, the rendered images will be saved to this directory. "
    "Otherwise, no files will be saved.");
DEFINE_string(
    server_base_url, RenderEngineGltfClientParams{}.base_url,
    "The base_url for the render server.");
DEFINE_string(
    server_render_endpoint, RenderEngineGltfClientParams{}.render_endpoint,
    "The render_endpoint for the render server.");
DEFINE_bool(
    cleanup, RenderEngineGltfClientParams{}.cleanup,
    "Whether the client should cleanup files generated or retrieved from the "
    "server.");

static bool valid_render_engine(const char* flagname, const std::string& val) {
  if (val == "vtk")
    return true;
  else if (val == "client")
    return true;
  drake::log()->error("Invalid value for {}: '{}'; options: 'client' or 'vtk'.",
                      flagname, val);
  return false;
}
DEFINE_string(render_engine, "client",
              "Renderer choice, options: 'client', 'vtk'.");
DEFINE_validator(render_engine, &valid_render_engine);

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;
using lcm::DrakeLcm;
using math::RigidTransformd;
using math::RollPitchYawd;
using multibody::Body;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::SpatialVelocity;
using render::ColorRenderCamera;
using render::DepthRenderCamera;
using systems::Context;
using systems::InputPort;
using systems::lcm::LcmPublisherSystem;
using systems::sensors::ImageToLcmImageArrayT;
using systems::sensors::ImageWriter;
using systems::sensors::PixelType;
using systems::sensors::RgbdSensor;

RigidTransformd ParseCameraPose(const std::string& input_str) {
  const char delimiter = ',';
  std::vector<double> xyzrpy_numeric;

  size_t pos = 0;
  size_t next = 0;
  while ((next = input_str.find(delimiter, pos)) != std::string::npos) {
    xyzrpy_numeric.push_back(std::stod(input_str.substr(pos, next)));
    pos = next + 1;
  }
  xyzrpy_numeric.push_back(std::stod(input_str.substr(pos, next)));
  DRAKE_DEMAND(xyzrpy_numeric.size() == 6);

  const RigidTransformd X_WB(
      RollPitchYawd{xyzrpy_numeric[3], xyzrpy_numeric[4], xyzrpy_numeric[5]},
      Vector3d(xyzrpy_numeric[0], xyzrpy_numeric[1], xyzrpy_numeric[2]));

  return X_WB;
}

// Validates `save_dir` and determines whether image saving will be enabled.
// Throws if an non-empty but invalid directory is supplied.
bool IsValidSaveDirectory(const std::string& save_dir) {
  if (save_dir.empty())
    return false;

  if (!std::filesystem::exists(save_dir) ||
      !std::filesystem::is_directory(save_dir)) {
    throw std::logic_error(fmt::format(
        "Provided save_dir {} is invalid", save_dir));
  }
  return true;
}

int DoMain() {
  DRAKE_THROW_UNLESS(FLAGS_color || FLAGS_depth || FLAGS_label);

  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);

  // Log debug messages and set `params.verbose` to true if `cleanup=false`.
  if (!FLAGS_cleanup) logging::set_log_level("debug");

  const std::string renderer_name("renderer");
  if (FLAGS_render_engine == "vtk") {
    scene_graph.AddRenderer(renderer_name, geometry::MakeRenderEngineVtk({}));
  } else {  // FLAGS_render_engine == "client"
    RenderEngineGltfClientParams params;
    params.base_url = FLAGS_server_base_url;
    params.render_endpoint = FLAGS_server_render_endpoint;
    params.cleanup = FLAGS_cleanup;
    params.verbose = !FLAGS_cleanup;
    scene_graph.AddRenderer(renderer_name,
                            geometry::MakeRenderEngineGltfClient(params));
  }

  // We assume the example sdf contains a body called "base_link_mustard".  We
  // will use MbP to *move* the mustard bottle by giving it an initial velocity;
  // we don't want to have to wait for gravity to take effect to observe a
  // difference in position.
  Parser parser{&plant};
  parser.AddModels(FindResourceOrThrow(
      "drake/geometry/render_gltf_client/test/example_scene.sdf"));

  DrakeLcm lcm;
  DrakeVisualizerd::AddToBuilder(&builder, scene_graph, &lcm);
  const ColorRenderCamera color_camera{
      {renderer_name, {640, 480, M_PI_4}, {0.01, 10.0}, {}}, false};
  const DepthRenderCamera depth_camera{color_camera.core(), {0.01, 10.0}};
  const RigidTransformd X_WB = ParseCameraPose(FLAGS_camera_xyz_rpy);

  const FrameId world_id = scene_graph.world_frame_id();
  RgbdSensor* camera =
      builder.AddSystem<RgbdSensor>(world_id, X_WB, color_camera, depth_camera);
  builder.Connect(scene_graph.get_query_output_port(),
                  camera->query_object_input_port());

  // Broadcast images via LCM for visualization (via drake visualizer).
  const double image_publish_period = 1. / FLAGS_render_fps;
  ImageToLcmImageArrayT* image_to_lcm_image_array =
      builder.template AddSystem<ImageToLcmImageArrayT>();
  image_to_lcm_image_array->set_name("converter");

  LcmPublisherSystem* image_array_lcm_publisher{nullptr};
  image_array_lcm_publisher =
      builder.template AddSystem(LcmPublisherSystem::Make<lcmt_image_array>(
          "DRAKE_RGBD_CAMERA_IMAGES", &lcm, image_publish_period));
  image_array_lcm_publisher->set_name("publisher");

  builder.Connect(image_to_lcm_image_array->image_array_t_msg_output_port(),
                  image_array_lcm_publisher->get_input_port());

  ImageWriter* image_writer{nullptr};
  const bool save_images = IsValidSaveDirectory(FLAGS_save_dir);
  if (save_images) {
    image_writer = builder.template AddSystem<ImageWriter>();
  }

  const std::string filename =
      (std::filesystem::path(FLAGS_save_dir) / "{image_type}_{count:03}").
          string();

  if (FLAGS_color) {
    const auto& port =
        image_to_lcm_image_array->DeclareImageInputPort<PixelType::kRgba8U>(
            "color");
    builder.Connect(camera->color_image_output_port(), port);

    if (save_images) {
      const auto& writer_port =
          image_writer->DeclareImageInputPort<PixelType::kRgba8U>(
              "color", filename, image_publish_period, 0.);
      builder.Connect(camera->color_image_output_port(), writer_port);
    }
  }

  if (FLAGS_depth) {
    const auto& port =
        image_to_lcm_image_array->DeclareImageInputPort<PixelType::kDepth32F>(
            "depth");
    builder.Connect(camera->depth_image_32F_output_port(), port);

    if (save_images) {
      const auto& writer_port =
          image_writer->DeclareImageInputPort<PixelType::kDepth32F>(
              "depth", filename, image_publish_period, 0.);
      builder.Connect(camera->depth_image_32F_output_port(), writer_port);
    }
  }

  if (FLAGS_label) {
    const auto& port =
        image_to_lcm_image_array->DeclareImageInputPort<PixelType::kLabel16I>(
            "label");
    builder.Connect(camera->label_image_output_port(), port);

    if (save_images) {
      const auto& writer_port =
          image_writer->DeclareImageInputPort<PixelType::kLabel16I>(
              "label", filename, image_publish_period, 0.);
      builder.Connect(camera->label_image_output_port(), writer_port);
    }
  }

  plant.Finalize();
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  auto& context = static_cast<systems::DiagramContext<double>&>(
      simulator.get_mutable_context());
  auto& plant_context = plant.GetMyMutableContextFromRoot(&context);

  // Initialize the moving bottle's position and speed so we can observe motion.
  // The mustard bottle spins while climbing slightly.
  plant.mutable_gravity_field().set_gravity_vector({0, 0, 0});
  const Body<double>& mustard_body = plant.GetBodyByName(
      "base_link_mustard",
      plant.GetModelInstanceByName("example_scene::mustard_bottle"));
  const RigidTransformd X_WMustardBottle(RollPitchYawd{-M_PI / 2, 0, -M_PI / 2},
                                         Vector3d(0, 0, 0));
  plant.SetFreeBodyPose(&plant_context, mustard_body, X_WMustardBottle);
  const SpatialVelocity<double> V_WMustardBottle(Vector3d{0.6, 0, 0},
                                                 Vector3d{0, 0, 0.1});
  plant.SetFreeBodySpatialVelocity(&plant_context, mustard_body,
                                   V_WMustardBottle);
  simulator.set_target_realtime_rate(1.f);
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::DoMain();
}
