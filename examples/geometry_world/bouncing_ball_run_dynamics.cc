#include <memory>
#include <utility>

#include "drake/examples/geometry_world/bouncing_ball_plant.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_system.h"
#include "drake/geometry/geometry_visualization.h"
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
namespace bouncing_ball {
namespace {

using geometry::GeometryInstance;
using geometry::GeometrySystem;
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
  auto geometry_system = builder.AddSystem<GeometrySystem<double>>();
  geometry_system->set_name("geometry_system");

  SourceId ball_source_id = geometry_system->RegisterSource("ball1");
  auto bouncing_ball = builder.AddSystem<BouncingBallPlant>(
      ball_source_id, geometry_system, Vector2<double>(0.25, 0.25));
  bouncing_ball->set_name("BouncingBall1");

  SourceId ball_source_id2 = geometry_system->RegisterSource("ball2");
  auto bouncing_ball2 = builder.AddSystem<BouncingBallPlant>(
      ball_source_id2, geometry_system, Vector2<double>(-0.25, -0.25));
  bouncing_ball->set_name("BouncingBall2");

  SourceId global_source = geometry_system->RegisterSource("anchored");
  Vector3<double> normal_G(0, 0, 1);
  Vector3<double> point_G(0, 0, 0);
  geometry_system->RegisterAnchoredGeometry(
      global_source,
      make_unique<GeometryInstance>(HalfSpace::MakePose(normal_G, point_G),
                                    make_unique<HalfSpace>()));
  DrakeLcm lcm;
  PoseBundleToDrawMessage* converter =
      builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem* publisher =
      builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher->set_publish_period(1 / 60.0);

  builder.Connect(bouncing_ball->get_geometry_id_output_port(),
                  geometry_system->get_source_frame_id_port(ball_source_id));
  builder.Connect(bouncing_ball->get_geometry_pose_output_port(),
                  geometry_system->get_source_pose_port(ball_source_id));
  builder.Connect(geometry_system->get_query_output_port(),
                  bouncing_ball->get_geometry_query_input_port());

  builder.Connect(bouncing_ball2->get_geometry_id_output_port(),
                  geometry_system->get_source_frame_id_port(ball_source_id2));
  builder.Connect(bouncing_ball2->get_geometry_pose_output_port(),
                  geometry_system->get_source_pose_port(ball_source_id2));
  builder.Connect(geometry_system->get_query_output_port(),
                  bouncing_ball2->get_geometry_query_input_port());

  builder.Connect(geometry_system->get_pose_bundle_output_port(),
                  converter->get_input_port(0));
  builder.Connect(*converter, *publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(*geometry_system);
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
  init_ball(bouncing_ball, 0.3, 0.);
  init_ball(bouncing_ball2, 0.3, 0.3);

  simulator.get_mutable_integrator()->set_maximum_step_size(0.002);
  simulator.set_target_realtime_rate(1.f);
  simulator.Initialize();
  simulator.StepTo(10);

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::bouncing_ball::do_main();
}
