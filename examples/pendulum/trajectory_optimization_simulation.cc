#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

using drake::solvers::SolutionResult;

namespace drake {
namespace examples {
namespace pendulum {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int DoMain() {
  systems::DiagramBuilder<double> builder;

  auto pendulum = std::make_unique<PendulumPlant<double>>();
  pendulum->set_name("pendulum");

  auto context = pendulum->CreateDefaultContext();

  const int kNumTimeSamples = 21;
  const double kMinimumTimeStep = 0.2;
  const double kMaximumSampleTime = 0.5;
  systems::trajectory_optimization::DirectCollocation dircol(
      pendulum.get(), *context, kNumTimeSamples, kMinimumTimeStep,
      kMaximumSampleTime);

  dircol.AddEqualTimeIntervalsConstraints();

  // TODO(russt): Add this constraint to PendulumPlant and get it automatically
  // through DirectCollocation.
  const double kTorqueLimit = 3.0;  // N*m.
  const solvers::VectorXDecisionVariable& u = dircol.input();
  dircol.AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  dircol.AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

  PendulumState<double> initial_state, final_state;
  initial_state.set_theta(0.0);
  initial_state.set_thetadot(0.0);
  final_state.set_theta(M_PI);
  final_state.set_thetadot(0.0);

  dircol.AddLinearConstraint(dircol.initial_state() ==
                             initial_state.get_value());
  dircol.AddLinearConstraint(dircol.final_state() == final_state.get_value());

  const double R = 10;  // Cost on input "effort".
  dircol.AddRunningCost((R * u) * u);

  const double timespan_init = 4;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_state.get_value(), final_state.get_value()});
  dircol.SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x);
  SolutionResult result = dircol.Solve();
  if (result != SolutionResult::kSolutionFound) {
    std::cerr << "Failed to solve optimization for the swing-up trajectory"
              << std::endl;
    return 1;
  }

  const PiecewisePolynomialTrajectory pp_traj =
      dircol.ReconstructInputTrajectory();
  const PiecewisePolynomialTrajectory pp_xtraj =
      dircol.ReconstructStateTrajectory();
  auto input_trajectory = builder.AddSystem<systems::TrajectorySource>(pp_traj);
  input_trajectory->set_name("input trajectory");
  auto state_trajectory =
      builder.AddSystem<systems::TrajectorySource>(pp_xtraj);
  state_trajectory->set_name("state trajectory");

  // The choices of PidController constants here are fairly arbitrary,
  // but seem to effectively swing up the pendulum and hold it.
  const double Kp = 10;
  const double Ki = 0;
  const double Kd = 1;
  auto pid_controlled_pendulum =
      builder.AddSystem<systems::controllers::PidControlledSystem<double>>(
          std::move(pendulum), Kp, Ki, Kd);
  pid_controlled_pendulum->set_name("PID Controlled Pendulum");

  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"),
      multibody::joints::kFixed, tree.get());

  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");

  builder.Connect(input_trajectory->get_output_port(),
                  pid_controlled_pendulum->get_control_input_port());
  builder.Connect(state_trajectory->get_output_port(),
                  pid_controlled_pendulum->get_state_input_port());
  builder.Connect(pid_controlled_pendulum->get_state_output_port(),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(pp_xtraj.get_end_time());

  const auto& pendulum_state =
      PendulumPlant<double>::get_state(diagram->GetSubsystemContext(
          *(pid_controlled_pendulum->plant()), simulator.get_context()));

  if (!is_approx_equal_abstol(pendulum_state.get_value(),
                              final_state.get_value(), 1e-3)) {
    throw std::runtime_error("Did not reach trajectory target.");
  }
  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::pendulum::DoMain();
}
