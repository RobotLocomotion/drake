#include <memory>
#include <utility>

#include <gflags/gflags.h>

#include "drake/examples/scene_graph/bouncing_ball_plant.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
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

namespace drake {
namespace examples {
namespace scene_graph {
namespace bouncing_ball {
namespace {

using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::DrakeVisualizerd;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using geometry::GeometryId;
using geometry::HalfSpace;
using geometry::IllustrationProperties;
using geometry::PerceptionProperties;
using geometry::ProximityProperties;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderEngineVtkParams;
using geometry::render::RenderLabel;
using geometry::SceneGraph;
using geometry::SourceId;
using lcm::DrakeLcm;
using math::RigidTransformd;
using math::RotationMatrixd;
using systems::InputPort;
using systems::sensors::PixelType;
using systems::sensors::RgbdSensor;
using std::make_unique;

int do_main() {
  systems::DiagramBuilder<double> builder;
  auto scene_graph = builder.AddSystem<SceneGraph<double>>();
  scene_graph->set_name("scene_graph");
  const std::string render_name("renderer");
  scene_graph->AddRenderer(render_name,
                           MakeRenderEngineVtk(RenderEngineVtkParams()));

  // Create two bouncing balls --> two plants. Put the balls at positions
  // mirrored over the origin (<0.25, 0.25> and <-0.25, -0.25>, respectively).
  // See below for setting the initial *height*.
  const SourceId ball_source_id1 = scene_graph->RegisterSource("ball1");
  auto bouncing_ball1 = builder.AddSystem<BouncingBallPlant>(
      ball_source_id1, scene_graph, Vector2<double>(0.25, 0.25));
  bouncing_ball1->set_name("BouncingBall1");

  const SourceId ball_source_id2 = scene_graph->RegisterSource("ball2");
  auto bouncing_ball2 = builder.AddSystem<BouncingBallPlant>(
      ball_source_id2, scene_graph, Vector2<double>(-0.25, -0.25));
  bouncing_ball2->set_name("BouncingBall2");

  const SourceId global_source = scene_graph->RegisterSource("anchored");
  // Add a "ground" halfspace. Define the pose of the half space (H) in the
  // world from its normal (Hz_W) and a point on the plane (p_WH). In this case,
  // X_WH will be the identity.
  Vector3<double> Hz_W(0, 0, 1);
  Vector3<double> p_WHo_W(0, 0, 0);
  const GeometryId ground_id = scene_graph->RegisterAnchoredGeometry(
      global_source,
      make_unique<GeometryInstance>(HalfSpace::MakePose(Hz_W, p_WHo_W),
                                    make_unique<HalfSpace>(), "ground"));
  scene_graph->AssignRole(global_source, ground_id, ProximityProperties());
  scene_graph->AssignRole(global_source, ground_id, IllustrationProperties());

  builder.Connect(bouncing_ball1->get_geometry_pose_output_port(),
                  scene_graph->get_source_pose_port(ball_source_id1));
  builder.Connect(scene_graph->get_query_output_port(),
                  bouncing_ball1->get_geometry_query_input_port());

  builder.Connect(bouncing_ball2->get_geometry_pose_output_port(),
                  scene_graph->get_source_pose_port(ball_source_id2));
  builder.Connect(scene_graph->get_query_output_port(),
                  bouncing_ball2->get_geometry_query_input_port());

  DrakeLcm lcm;
  DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, &lcm);

  if (FLAGS_render_on) {
    PerceptionProperties properties;
    properties.AddProperty("phong", "diffuse", Vector4d{0.8, 0.8, 0.8, 1.0});
    properties.AddProperty("label", "id", RenderLabel(ground_id.get_value()));
    scene_graph->AssignRole(global_source, ground_id, properties);

    // Create the camera.
    const ColorRenderCamera color_camera{
        {render_name, {640, 480, M_PI_4}, {0.1, 2.0}, {}}, false};
    const DepthRenderCamera depth_camera{color_camera.core(), {0.1, 2.0}};
    // We need to position and orient the camera. We have the camera body frame
    // B (see rgbd_sensor.h) and the camera frame C (see camera_info.h).
    // By default X_BC = I in the RgbdSensor. So, to aim the camera, Cz = Bz
    // should point from the camera position to the origin. By points *down* the
    // image, so we need to align it in the -Wz direction. So,  we compute the
    // basis using camera Y-ish in the By ≈ -Wz direction to compute Bx, and
    // then use Bx an and Bz to compute By.
    const Vector3d p_WB(0.3, -1, 0.25);
    // Set rotation looking at the origin.
    const Vector3d Bz_W = -p_WB.normalized();
    const Vector3d Bx_W = -Vector3d::UnitZ().cross(Bz_W).normalized();
    const Vector3d By_W = Bz_W.cross(Bx_W).normalized();
    const RotationMatrixd R_WB =
        RotationMatrixd::MakeFromOrthonormalColumns(Bx_W, By_W, Bz_W);
    const RigidTransformd X_WB(R_WB, p_WB);

    auto camera = builder.AddSystem<RgbdSensor>(
        scene_graph->world_frame_id(), X_WB, color_camera, depth_camera);
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

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto init_ball = [&](BouncingBallPlant<double>* system, double z,
                       double zdot) {
    systems::Context<double>& ball_context =
        diagram->GetMutableSubsystemContext(*system,
                                            &simulator.get_mutable_context());
    system->set_z(&ball_context, z);
    system->set_zdot(&ball_context, zdot);
  };
  init_ball(bouncing_ball1, 0.3, 0.);
  init_ball(bouncing_ball2, 0.3, 0.3);

  simulator.get_mutable_integrator().set_maximum_step_size(0.002);
  simulator.set_target_realtime_rate(1.f);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace scene_graph
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::scene_graph::bouncing_ball::do_main();
}
