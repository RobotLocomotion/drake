#include <memory>
#include <utility>

#include <gflags/gflags.h>

#include "drake/examples/scene_graph/dev/bouncing_ball_plant.h"
#include "drake/geometry/dev/geometry_visualization.h"
#include "drake/geometry/dev/render/render_engine_vtk.h"
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
#include "drake/systems/sensors/pixel_types.h"

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

using geometry::GeometryInstance;
template <typename T>
using ProximitySceneGraph = geometry::SceneGraph<T>;
template <typename T>
using RenderSceneGraph = geometry::dev::SceneGraph<T>;
using geometry::GeometryId;
using geometry::HalfSpace;
using geometry::IllustrationProperties;
using geometry::ProximityProperties;
using geometry::SourceId;
using geometry::dev::ConnectDrakeVisualizer;
using geometry::dev::render::DepthCameraProperties;
using geometry::dev::render::RenderEngineVtk;
using lcm::DrakeLcm;
using std::make_unique;
using systems::InputPort;
using systems::sensors::PixelType;
using systems::sensors::dev::RgbdCamera;

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
  GeometryId ground_id = proximity_scene_graph->RegisterAnchoredGeometry(
      global_source,
      make_unique<GeometryInstance>(HalfSpace::MakePose(Hz_W, p_WH),
                                    make_unique<HalfSpace>(), "ground"));
  proximity_scene_graph->AssignRole(global_source, ground_id,
                                    ProximityProperties());
  proximity_scene_graph->AssignRole(global_source, ground_id,
                                    IllustrationProperties());

  builder.Connect(bouncing_ball1->get_geometry_pose_output_port(),
                  proximity_scene_graph->get_source_pose_port(ball_source_id1));
  builder.Connect(proximity_scene_graph->get_query_output_port(),
                  bouncing_ball1->get_geometry_query_input_port());

  builder.Connect(bouncing_ball2->get_geometry_pose_output_port(),
                  proximity_scene_graph->get_source_pose_port(ball_source_id2));
  builder.Connect(proximity_scene_graph->get_query_output_port(),
                  bouncing_ball2->get_geometry_query_input_port());

  // Add rendering scene graph -- use it for both illustration and perception.
  auto render_scene_graph = builder.AddSystem<RenderSceneGraph<double>>();
  render_scene_graph->set_name("render_scene_graph");
  const std::string render_name("renderer");
  render_scene_graph->AddRenderer(render_name, make_unique<RenderEngineVtk>());
  render_scene_graph->CopyFrom(*proximity_scene_graph);
  builder.Connect(bouncing_ball1->get_geometry_pose_output_port(),
                  render_scene_graph->get_source_pose_port(ball_source_id1));
  builder.Connect(bouncing_ball2->get_geometry_pose_output_port(),
                  render_scene_graph->get_source_pose_port(ball_source_id2));

  DrakeLcm lcm;
  ConnectDrakeVisualizer(&builder, *render_scene_graph, &lcm);

  if (FLAGS_render_on) {
    // Create the camera.
    DepthCameraProperties camera_properties(640, 480, M_PI_4, render_name, 0.1,
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
        builder.template AddSystem<systems::sensors::ImageToLcmImageArrayT>();
    image_to_lcm_image_array->set_name("converter");

    systems::lcm::LcmPublisherSystem* image_array_lcm_publisher{nullptr};
    if ((FLAGS_color || FLAGS_depth || FLAGS_label)) {
      image_array_lcm_publisher =
          builder.template AddSystem(systems::lcm::LcmPublisherSystem::Make<
              robotlocomotion::image_array_t>(
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
      builder.Connect(camera->depth_image_output_port(), port);
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
  simulator.Initialize();
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
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
