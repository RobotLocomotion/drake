#include <gflags/gflags.h>

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/manipulation_station/station_simulation.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace manipulation_station {
namespace {

// Simple example which simulates the manipulation station (and visualizes it
// with drake visualizer).
// TODO(russt): Replace this with a slightly more interesting minimal example
// (e.g. picking up an object) and perhaps a slightly more descriptive name.

using Eigen::VectorXd;

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(duration, 4.0, "Simulation duration.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  systems::DiagramBuilder<double> builder;

  // Create the "manipulation station".
  auto station = builder.AddSystem<StationSimulation>();
  station->Finalize();

  geometry::ConnectDrakeVisualizer(&builder, station->get_mutable_scene_graph(),
                                   station->GetOutputPort("pose_bundle"));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto &context = diagram->GetMutableSubsystemContext(
      *station, &simulator.get_mutable_context());

  // Set initial conditions for the IIWA:
  VectorXd q0 = VectorXd::Zero(7);
  q0(1) = 0.3;
  station->SetIiwaPosition(q0, &context);
  const VectorXd qdot0 = VectorXd::Zero(7);
  station->SetIiwaVelocity(qdot0, &context);

  // Position command should hold the arm at the initial state.
  context.FixInputPort(station->GetInputPort("iiwa_position").get_index(), q0);

  // Zero feed-forward torque.
  context.FixInputPort(
      station->GetInputPort("iiwa_feedforward_torque").get_index(),
      VectorXd::Zero(7));

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_duration);

  // Check that the arm is (very roughly) in the commanded position.
  VectorXd q = station->GetIiwaPosition(context);
  if (!is_approx_equal_abstol(q, q0, 1.e-3)) {
    std::cout << "q - q0  = " << (q-q0).transpose() << std::endl;
    DRAKE_ABORT_MSG("q is not sufficiently close to q0.");
  }

  return 0;
}

}  // namespace
}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::manipulation_station::do_main(argc, argv);
}
