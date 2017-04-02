
#include <cmath>
#include <memory>

#include "gtest/gtest.h"

#include "drake/automotive/simple_car.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/lcm/lcm_call_matlab.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace automotive {
namespace {

// Sets up a simple trajectory optimization problem that finds a series
// of DrivingCommand's that takes the SimpleCar from an initial condition
// off the x-axis back to the x-axis.
GTEST_TEST(TrajectoryOptimizationTest, SimpleCarDircolTest) {
  SimpleCar<double> plant;
  auto context = plant.CreateDefaultContext();

  SimpleCarState<double> x0, xf;

  x0.set_x(0.0);
  x0.set_y(1.0);
  x0.set_heading(-.02);
  x0.set_velocity(15.0);  // m/s = ~ 33mph

  const double initial_duration = 30.0;  // seconds
  xf.set_x(x0.x() + initial_duration * x0.velocity());
  xf.set_y(0.0);
  xf.set_heading(0.0);
  xf.set_velocity(x0.velocity());

  const int kNumTimeSamples = 10;

  // The solved trajectory may deviate from the initial guess at a reasonable
  // duration.
  const double kTrajectoryTimeLowerBound = 0.8 * initial_duration,
               kTrajectoryTimeUpperBound = 1.2 * initial_duration;

  systems::DircolTrajectoryOptimization prog(&plant, *context, kNumTimeSamples,
                                             kTrajectoryTimeLowerBound,
                                             kTrajectoryTimeUpperBound);

  // Input limits (note that the steering limit imposed by SimpleCar is larger).
  DrivingCommand<double> lower_limit, upper_limit;
  lower_limit.set_steering_angle(-M_PI_2);
  lower_limit.set_acceleration(-std::numeric_limits<double>::infinity());
  upper_limit.set_steering_angle(M_PI_2);
  upper_limit.set_acceleration(std::numeric_limits<double>::infinity());
  prog.AddInputBounds(lower_limit.get_value(), upper_limit.get_value());

  // Ensure that time intervals are (relatively) evenly spaced.
  prog.AddTimeIntervalBounds(kTrajectoryTimeLowerBound / (kNumTimeSamples - 1),
                             kTrajectoryTimeUpperBound / (kNumTimeSamples - 1));

  // Fix initial conditions.
  prog.AddLinearConstraint(prog.initial_state() == x0.get_value());

  // Fix final conditions.
  prog.AddLinearConstraint(prog.final_state() == xf.get_value());

  // Cost function: int_0^T [ u'u ] dt.
  prog.AddRunningCost(prog.input().transpose() * prog.input());

  // Initial guess is a straight line from the initial state to the final state.
  auto initial_state_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      {0, initial_duration}, {x0.get_value(), xf.get_value()});

  solvers::SolutionResult result =
      prog.SolveTraj(initial_duration, PiecewisePolynomial<double>(),
                     initial_state_trajectory);

  solvers::SolverType solver;
  int solver_result;
  prog.GetSolverResult(&solver, &solver_result);

  if (solver == solvers::SolverType::kIpopt) {
    EXPECT_EQ(result,
              solvers::SolutionResult::kIterationLimit);  // TODO(russt): Tune
                                                          // Ipopt for this
                                                          // example.
  } else {
    EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound);
  }

  // Plot the solution.
  // Note: see lcm_call_matlab.h for instructions on viewing the plot.
  Eigen::MatrixXd inputs;
  Eigen::MatrixXd states;
  std::vector<double> times_out;
  prog.GetResultSamples(&inputs, &states, &times_out);
  lcm::LcmCallMatlab("plot", states.row(SimpleCarStateIndices::kX),
                     states.row(SimpleCarStateIndices::kY));
  lcm::LcmCallMatlab("xlabel", "x (m)");
  lcm::LcmCallMatlab("ylabel", "y (m)");

  // Checks that the input commands found are not too large.
  EXPECT_LE(inputs.row(0).lpNorm<1>(), 0.1);
  EXPECT_LE(inputs.row(1).lpNorm<1>(), 1);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
