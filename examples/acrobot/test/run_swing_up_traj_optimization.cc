// Generates a swing-up trajectory for acrobot and displays the trajectory
// in DrakeVisualizer. Trajectory generation code is based on
// pendulum_swing_up.cc.

#include <iostream>
#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/acrobot/acrobot_geometry.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h"
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
  if (!solvers::SnoptSolver::is_available()) {
    std::cout << "This test was flaky with IPOPT, so currently requires SNOPT."
              << std::endl;
    return 0;
  }

  AcrobotPlant<double> acrobot;
  auto context = acrobot.CreateDefaultContext();

  const int kNumTimeSamples = 21;
  const double kMinimumTimeStep = 0.05;
  const double kMaximumTimeStep = 0.2;
  systems::trajectory_optimization::DirectCollocation dircol(
      &acrobot, *context, kNumTimeSamples, kMinimumTimeStep, kMaximumTimeStep);

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
  auto traj_init_x = PiecewisePolynomialType::FirstOrderHold(
      {0, timespan_init}, {x0, xG});
  dircol.SetInitialTrajectory(PiecewisePolynomialType(), traj_init_x);

  solvers::SnoptSolver solver;
  const auto result = solver.Solve(dircol);
  if (!result.is_success()) {
    std::cerr << "No solution found.\n";
    return 1;
  }

  // Stabilize the trajectory with LQR and simulate.
  systems::DiagramBuilder<double> builder;

  auto plant = builder.AddSystem<AcrobotPlant>();
  auto plant_context = plant->CreateDefaultContext();

  systems::controllers::FiniteHorizonLinearQuadraticRegulatorOptions options;
  const trajectories::PiecewisePolynomial<double> xtraj =
      dircol.ReconstructStateTrajectory(result);
  const trajectories::PiecewisePolynomial<double> utraj =
      dircol.ReconstructInputTrajectory(result);
  options.x0 = &xtraj;
  options.u0 = &utraj;

  const Eigen::Matrix4d Q = Eigen::Vector4d(10.0, 10.0, 1.0, 1.0).asDiagonal();
  options.Qf = Q;
  auto regulator = builder.AddSystem(
      systems::controllers::MakeFiniteHorizonLinearQuadraticRegulator(
          *plant, *plant_context, options.u0->start_time(),
          options.u0->end_time(), Q, Vector1d::Constant(1), options));

  builder.Connect(regulator->get_output_port(0), plant->get_input_port(0));
  builder.Connect(plant->get_output_port(0), regulator->get_input_port(0));

  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  AcrobotGeometry::AddToBuilder(&builder, plant->get_output_port(0),
                                scene_graph);
  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.Initialize();
  simulator.get_mutable_context().SetTime(options.u0->start_time());
  simulator.AdvanceTo(options.u0->end_time());

  // Confirm that the stabilized system is in the vicinity of the upright.
  DRAKE_DEMAND(is_approx_equal_abstol(
      simulator.get_context().get_continuous_state_vector().CopyToVector(), xG,
      0.1));
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
