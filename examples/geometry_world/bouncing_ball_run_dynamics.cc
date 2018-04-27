#include <memory>
#include <utility>

#include "drake/examples/geometry_world/bouncing_ball_plant.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {
namespace examples {
namespace geometry_world {
namespace bouncing_ball {
namespace {

using geometry::GeometryInstance;
using geometry::SceneGraph;
using geometry::HalfSpace;
using geometry::SourceId;
using lcm::DrakeLcm;
using systems::InputPortDescriptor;
using systems::rendering::PoseBundleToDrawMessage;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::Serializer;
using std::make_unique;

int do_main() {
  systems::DiagramBuilder<double> builder;
  auto scene_graph = builder.AddSystem<SceneGraph<double>>();
  scene_graph->set_name("scene_graph");

  // Create two bouncing balls --> two plants. Put the balls at positions
  // mirrored over the origin (<0.25, 0.25> and <-0.25, -0.25>, respectively).
  // See below for setting the initial *height*.
  SourceId ball_source_id1 = scene_graph->RegisterSource("ball1");
  auto bouncing_ball1 = builder.AddSystem<BouncingBallPlant>(
      ball_source_id1, scene_graph, Vector2<double>(0.25, 0.25));
  bouncing_ball1->set_name("BouncingBall1");

  SourceId ball_source_id2 = scene_graph->RegisterSource("ball2");
  auto bouncing_ball2 = builder.AddSystem<BouncingBallPlant>(
      ball_source_id2, scene_graph, Vector2<double>(-0.25, -0.25));
  bouncing_ball2->set_name("BouncingBall2");

  SourceId global_source = scene_graph->RegisterSource("anchored");
  // Add a "ground" halfspace. Define the pose of the half space (H) in the
  // world from its normal (Hz_W) and a point on the plane (p_WH). In this case,
  // X_WH will be the identity.
  Vector3<double> Hz_W(0, 0, 1);
  Vector3<double> p_WH(0, 0, 0);
  scene_graph->RegisterAnchoredGeometry(
      global_source,
      make_unique<GeometryInstance>(HalfSpace::MakePose(Hz_W, p_WH),
                                    make_unique<HalfSpace>()));
  DrakeLcm lcm;
  PoseBundleToDrawMessage* converter =
      builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem* publisher =
      builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher->set_publish_period(1 / 60.0);

  builder.Connect(bouncing_ball1->get_geometry_pose_output_port(),
                  scene_graph->get_source_pose_port(ball_source_id1));
  builder.Connect(scene_graph->get_query_output_port(),
                  bouncing_ball1->get_geometry_query_input_port());

  builder.Connect(bouncing_ball2->get_geometry_pose_output_port(),
                  scene_graph->get_source_pose_port(ball_source_id2));
  builder.Connect(scene_graph->get_query_output_port(),
                  bouncing_ball2->get_geometry_query_input_port());

  builder.Connect(scene_graph->get_pose_bundle_output_port(),
                  converter->get_input_port(0));
  builder.Connect(*converter, *publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(*scene_graph);
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
  simulator.StepTo(10);

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace geometry_world
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::geometry_world::bouncing_ball::do_main();
}
