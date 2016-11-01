
#include <iostream>
#include <memory>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/Pendulum/pendulum_swing_up.h"
#include "drake/examples/Pendulum/pendulum_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/solvers/trajectoryOptimization/dircol_trajectory_optimization.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/trajectory_source.h"
#include "drake/systems/plants/rigid_body_plant/drake_visualizer.h"
#include "drake/util/drakeAppUtil.h"

using drake::solvers::SolutionResult;
using drake::MatrixCompareType;

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

namespace drake {
namespace examples {
namespace pendulum {
namespace {

int do_main(int argc, char* argv[]) {
  systems::DiagramBuilder<double> builder;
  auto pendulum = std::make_unique<PendulumPlant<double>>();
  PendulumPlant<double>* pendulum_p = pendulum.get();

  // This is a fairly small number of time samples for this system,
  // and it winds up making the controller do a lot of the work when
  // getting to the target state.  I (sam.creasey) suspect that a
  // different interpolation strategy (not linear interpolation of a
  // non-linear system, basically) would reduce this effect.
  const int kNumTimeSamples = 21;
  const int kTrajectoryTimeLowerBound = 2;
  const int kTrajectoryTimeUpperBound = 6;

  const Eigen::Vector2d x0(0, 0);
  const Eigen::Vector2d xG(M_PI, 0);

  drake::solvers::DircolTrajectoryOptimization dircol_traj(
      pendulum->get_tau_port().get_size(),
      pendulum->get_output_port().get_size(),
      kNumTimeSamples,  kTrajectoryTimeLowerBound,
      kTrajectoryTimeUpperBound);
  drake::examples::pendulum::AddSwingUpTrajectoryParams(
      kNumTimeSamples, x0, xG, &dircol_traj);

  const double timespan_init = 4;
  auto traj_init_x = PiecewisePolynomialType::FirstOrderHold(
      {0, timespan_init}, {x0, xG});
  SolutionResult result =
      dircol_traj.SolveTraj(timespan_init, PiecewisePolynomialType(),
                            traj_init_x);
  if (result != SolutionResult::kSolutionFound) {
    std::cerr << "Result is an Error" << std::endl;
    return 1;
  }

  const PiecewisePolynomialTrajectory pp_traj =
      dircol_traj.ReconstructInputTrajectory();
  const PiecewisePolynomialTrajectory pp_xtraj =
      dircol_traj.ReconstructStateTrajectory();
  auto input_source = builder.AddSystem<
    systems::TrajectorySource>(pp_traj);
  auto state_source = builder.AddSystem<
    systems::TrajectorySource>(pp_xtraj);

  lcm::DrakeLcm lcm;
  RigidBodyTree tree(GetDrakePath() + "/examples/Pendulum/Pendulum.urdf",
                     systems::plants::joints::kFixed);
  auto publisher =
      builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);

  // The choices of PidController constants here are fairly arbitrary,
  // but seem to effectively swing up the pendulum and hold it.
  auto controller = builder.AddSystem<systems::PidControlledSystem>(
      std::move(pendulum), 10., 0., 1.);

  builder.Connect(input_source->get_output_port(0),
                  controller->get_input_port(0));
  builder.Connect(state_source->get_output_port(0),
                  controller->get_input_port(1));
  builder.Connect(controller->get_output_port(0),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>* controller_context =
      diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), controller);
  controller->SetDefaultState(controller_context);

  simulator.Initialize();
  simulator.StepTo(kTrajectoryTimeUpperBound);

  systems::Context<double>* pendulum_context =
      controller->GetMutableSubsystemContext(controller_context, pendulum_p);
  auto state_vec =
      pendulum_context->get_continuous_state()->CopyToVector();
  if (!CompareMatrices(state_vec, xG, 1e-3, MatrixCompareType::absolute)) {
    throw std::runtime_error("Did not reach trajectory target.");
  }
  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::pendulum::do_main(argc, argv);
}
