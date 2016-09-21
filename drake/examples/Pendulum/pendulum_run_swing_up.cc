
#include <cmath>
#include <memory>
#include <stdexcept>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/Pendulum/Pendulum.h"
#include "drake/examples/Pendulum/pendulum_swing_up.h"
#include "drake/solvers/trajectoryOptimization/dircol_trajectory_optimization.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/controllers/simple_feedback_trajectory_controller.h"
#include "drake/systems/feedback_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/robot_state_tap.h"
#include "drake/util/drakeAppUtil.h"

using drake::SimpleFeedbackTrajectoryController;
using drake::solvers::SolutionResult;
using drake::MatrixCompareType;

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

int main(int argc, char* argv[]) {
  auto p = make_shared<Pendulum>();

  const int kNumTimeSamples = 21;
  const int kTrajectoryTimeLowerBound = 2;
  const int kTrajectoryTimeUpperBound = 6;

  const Eigen::Vector2d x0(0, 0);
  const Eigen::Vector2d xG(M_PI, 0);

  drake::solvers::DircolTrajectoryOptimization dircol_traj(
      drake::getNumInputs(*p), drake::getNumStates(*p),
      kNumTimeSamples,  kTrajectoryTimeLowerBound,
      kTrajectoryTimeUpperBound);
  drake::examples::pendulum::AddSwingUpTrajectoryParams(
      p, kNumTimeSamples, x0, xG, &dircol_traj);

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

  const PiecewisePolynomialType pp_traj =
      dircol_traj.ReconstructInputTrajectory();
  const PiecewisePolynomialType pp_xtraj =
      dircol_traj.ReconstructStateTrajectory();

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if (!lcm->good()) return 1;

  auto visualizer = std::make_shared<drake::BotVisualizer<PendulumState>>(
      lcm, drake::GetDrakePath() + "/examples/Pendulum/Pendulum.urdf",
      drake::systems::plants::joints::kFixed);
  auto robot_state_tap =
      std::make_shared<drake::RobotStateTap<PendulumState>>();

  Eigen::MatrixXd Q(2, 2);
  Q << 10, 0, 0, 1;  // arbitrary, taken from PendulumPlant.m
  Eigen::MatrixXd R(1, 1);
  R << 1;  // arbitrary, taken from PendulumPlant.m

  auto control = std::make_shared<SimpleFeedbackTrajectoryController<Pendulum>>(
      p, pp_traj, pp_xtraj, Q, R);
  auto traj_sys = drake::feedback(p, control);
  auto sys = drake::cascade(drake::cascade(traj_sys, visualizer),
                            robot_state_tap);

  drake::SimulationOptions options;
  options.realtime_factor = 1.0;
  if (commandLineOptionExists(argv, argv + argc, "--non-realtime")) {
    options.warn_real_time_violation = true;
  }

  PendulumState<double> x0_state = x0;
  drake::runLCM(sys, lcm, 0, kTrajectoryTimeUpperBound, x0_state, options);
  if (!CompareMatrices(toEigen(robot_state_tap->get_input_vector()), xG,
                       1e-4, MatrixCompareType::absolute)) {
    throw std::runtime_error("Did not reach trajectory target.");
  }

  return 0;
}
