#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/rimless_wheel/rimless_wheel.h"
#include "drake/examples/rimless_wheel/rimless_wheel_geometry.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace rimless_wheel {
namespace {

DEFINE_double(accuracy, 1e-4, "Accuracy of the rimless wheel system (unitless);"
    " must be positive.");
DEFINE_double(initial_angle, 0.0, "Initial angle of the wheel (rad).  Must be"
    " in the interval (slope - alpha, slope + alpha), as described in "
    "http://underactuated.mit.edu/underactuated.html?chapter=simple_legs .");
DEFINE_double(initial_angular_velocity, 5.0,
              "Initial angular velocity of the wheel (rad/sec).");
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

/// Simulates the rimless wheel from various initial velocities (accepted as
/// command-line arguments.  Run drake-visualizer to watch the results.
int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto rimless_wheel = builder.AddSystem<RimlessWheel>();
  rimless_wheel->set_name("rimless_wheel");

  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  RimlessWheelGeometry::AddToBuilder(
      &builder, rimless_wheel->get_floating_base_state_output_port(),
      scene_graph);
  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& rw_context = diagram->GetMutableSubsystemContext(
      *rimless_wheel, &simulator.get_mutable_context());
  RimlessWheelContinuousState<double>& state =
      rimless_wheel->get_mutable_continuous_state(&rw_context);

  // Check that command line argument puts the wheel above the ground.
  const RimlessWheelParams<double>& params =
      rimless_wheel->get_parameters(rw_context);
  const double alpha = rimless_wheel->calc_alpha(params);
  DRAKE_DEMAND(FLAGS_initial_angle > params.slope() - alpha);
  DRAKE_DEMAND(FLAGS_initial_angle < params.slope() + alpha);

  state.set_theta(FLAGS_initial_angle);
  state.set_thetadot(FLAGS_initial_angular_velocity);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.get_mutable_context().SetAccuracy(FLAGS_accuracy);
  simulator.AdvanceTo(10);

  // Check that the state is still inside the expected region (I did not miss
  // any collisions).
  DRAKE_DEMAND(state.theta() >= params.slope() - alpha);
  DRAKE_DEMAND(state.theta() <= params.slope() + alpha);

  return 0;
}

}  // namespace
}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::rimless_wheel::DoMain();
}
