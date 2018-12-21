#include <memory>
#include <utility>

#include <gflags/gflags.h>

#include "drake/examples/scene_graph/dev/bouncing_ball_plant.h"
#include "drake/geometry/dev/geometry_visualization.h"
#include "drake/geometry/dev/scene_graph.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/sensors/dev/rgbd_camera.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

namespace drake {
namespace examples {
namespace scene_graph {
namespace bouncing_ball {
namespace {

using geometry::GeometryInstance;
template <typename T>
using ProximitySceneGraph = geometry::SceneGraph<T>;
template <typename T>
using RenderSceneGraph = geometry::dev::SceneGraph<T>;
using geometry::dev::ConnectDrakeVisualizer;
using geometry::dev::render::DepthCameraProperties;
using geometry::dev::render::Fidelity;
using geometry::HalfSpace;
using geometry::SourceId;
using lcm::DrakeLcm;
using systems::InputPort;
using systems::sensors::dev::RgbdCamera;
using std::make_unique;

int do_main() {
  systems::DiagramBuilder<double> builder;
  auto proximity_scene_graph = builder.AddSystem<ProximitySceneGraph<double>>();
  proximity_scene_graph->set_name("proximity_scene_graph");

  // Create two bouncing balls --> two plants. Put the balls at positions
  // mirrored over the origin (<0.25, 0.25> and <-0.25, -0.25>, respectively).
  // See below for setting the initial *height*.
  SourceId ball_source_id1 = proximity_scene_graph->RegisterSource("ball1");
  auto bouncing_ball1 = builder.AddSystem<BouncingBallPlant>(
      ball_source_id1, proximity_scene_graph, Vector2<double>(0.25, 0.25));
  bouncing_ball1->set_name("BouncingBall1");

  SourceId ball_source_id2 = proximity_scene_graph->RegisterSource("ball2");
  auto bouncing_ball2 = builder.AddSystem<BouncingBallPlant>(
      ball_source_id2, proximity_scene_graph, Vector2<double>(-0.25, -0.25));
  bouncing_ball2->set_name("BouncingBall2");

  SourceId global_source = proximity_scene_graph->RegisterSource("anchored");
  // Add a "ground" halfspace. Define the pose of the half space (H) in the
  // world from its normal (Hz_W) and a point on the plane (p_WH). In this case,
  // X_WH will be the identity.
  Vector3<double> Hz_W(0, 0, 1);
  Vector3<double> p_WH(0, 0, 0);
  proximity_scene_graph->RegisterAnchoredGeometry(
      global_source,
      make_unique<GeometryInstance>(HalfSpace::MakePose(Hz_W, p_WH),
                                    make_unique<HalfSpace>(), "ground"));

  builder.Connect(bouncing_ball1->get_geometry_pose_output_port(),
                  proximity_scene_graph->get_source_pose_port(ball_source_id1));
  builder.Connect(proximity_scene_graph->get_query_output_port(),
                  bouncing_ball1->get_geometry_query_input_port());

  builder.Connect(bouncing_ball2->get_geometry_pose_output_port(),
                  proximity_scene_graph->get_source_pose_port(ball_source_id2));
  builder.Connect(proximity_scene_graph->get_query_output_port(),
                  bouncing_ball2->get_geometry_query_input_port());

  // Add rendering scene graph -- use it for both illustration and perception.
  auto render_scene_graph =
      builder.AddSystem<RenderSceneGraph<double>>(*proximity_scene_graph);
  render_scene_graph->set_name("render_scene_graph");
  builder.Connect(bouncing_ball1->get_geometry_pose_output_port(),
                  render_scene_graph->get_source_pose_port(ball_source_id1));
  builder.Connect(bouncing_ball2->get_geometry_pose_output_port(),
                  render_scene_graph->get_source_pose_port(ball_source_id2));

  DrakeLcm lcm;
  ConnectDrakeVisualizer(&builder, *render_scene_graph, &lcm);

  // Create the camera.
  DepthCameraProperties camera_properties(640, 480, M_PI_4, Fidelity::kLow, 0.1,
                                          2.0);
  Vector3<double> p_WC(-0.25, -0.75, 0.15);
  Vector3<double> rpy_WC(0, 0, M_PI_2 * 0.9);
  auto camera = builder.AddSystem<RgbdCamera>("fixed", p_WC, rpy_WC,
                                              camera_properties, false);
  builder.Connect(render_scene_graph->get_query_output_port(),
                  camera->query_object_input_port());

  // Broadcast the images.
  // Publishing images to drake visualizer
  auto image_to_lcm_image_array =
      builder.template AddSystem<systems::sensors::ImageToLcmImageArrayT>(
          "color", "depth", "label");
  image_to_lcm_image_array->set_name("converter");

  auto image_array_lcm_publisher = builder.template AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robotlocomotion::image_array_t>(
          "DRAKE_RGBD_CAMERA_IMAGES", &lcm,
          1.0 / 10 /* publish period = 10 fps */));
  image_array_lcm_publisher->set_name("publisher");

  builder.Connect(
      camera->color_image_output_port(),
      image_to_lcm_image_array->color_image_input_port());

  builder.Connect(
      camera->depth_image_output_port(),
      image_to_lcm_image_array->depth_image_input_port());

  builder.Connect(
      camera->label_image_output_port(),
      image_to_lcm_image_array->label_image_input_port());

  builder.Connect(
      image_to_lcm_image_array->image_array_t_msg_output_port(),
      image_array_lcm_publisher->get_input_port());

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

  simulator.get_mutable_integrator()->set_maximum_step_size(0.002);
  simulator.set_target_realtime_rate(1.f);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

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
