// Generates a swing-up trajectory for acrobot and displays the trajectory
// in DrakeVisualizer. Trajectory generation code is based on
// pendulum_swing_up.cc.

#include <iostream>
#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/acrobot/acrobot_geometry.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

using drake::solvers::SolutionResult;


namespace drake {
namespace examples {
namespace acrobot {

typedef trajectories::PiecewisePolynomial<double> PiecewisePolynomialType;

namespace {
DEFINE_double(realtime_factor, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int do_main() {
  AcrobotPlant<double> acrobot;
  auto context = acrobot.CreateDefaultContext();

  const int kNumTimeSamples = 21;
  const double kMinimumTimeStep = 0.2;
  const double kMaximumTimeStep = 0.5;
  systems::trajectory_optimization::DirectCollocation dircol(
      &acrobot, *context, kNumTimeSamples, kMinimumTimeStep,
      kMaximumTimeStep);

  dircol.AddEqualTimeIntervalsConstraints();

  // Current limit for MIT's acrobot is 7-9 Amps, according to Michael Posa.
  const double kTorqueLimit = 8;
  auto u = dircol.input();
  dircol.AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  dircol.AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

  const Eigen::Vector4d x0(0, 0, 0, 0);
  const Eigen::Vector4d xG(M_PI, 0, 0, 0);
  dircol.AddLinearConstraint(dircol.initial_state() == x0);
  dircol.AddLinearConstraint(dircol.final_state() == xG);

  const double R = 10;  // Cost on input "effort".
  dircol.AddRunningCost((R * u) * u);

  const double timespan_init = 4;
  // Certain solvers (SNOPT) are very sensitive when the initial guess starts
  // exactly in the straight-down configuration. This term nudges the initial
  // guess away from that configuration.
  const Eigen::Vector4d x0_perturbation{std::numeric_limits<double>::epsilon(),
                                        0, 0, 0};
  auto traj_init_x = PiecewisePolynomialType::FirstOrderHold(
      {0, timespan_init}, {x0 + x0_perturbation, xG});
  dircol.SetInitialTrajectory(PiecewisePolynomialType(), traj_init_x);
  const auto result = solvers::Solve(dircol);
  if (!result.is_success()) {
    std::cerr << "No solution found.\n";
    return 1;
  }

  // Now construct a diagram to visualize the results.
  systems::DiagramBuilder<double> builder;
  const trajectories::PiecewisePolynomial<double> pp_xtraj =
      dircol.ReconstructStateTrajectory(result);
  auto state_source = builder.AddSystem<systems::TrajectorySource>(pp_xtraj);

  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  AcrobotGeometry::AddToBuilder(
      &builder, state_source->get_output_port(), scene_graph);
  ConnectDrakeVisualizer(&builder, *scene_graph);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.Initialize();
  simulator.AdvanceTo(pp_xtraj.end_time());
  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::acrobot::do_main();
}
