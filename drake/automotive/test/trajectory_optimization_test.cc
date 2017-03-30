
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
  const double kTrajectoryTimeLowerBound = 0.8 * initial_duration,
               kTrajectoryTimeUpperBound = 1.2 * initial_duration;

  systems::DircolTrajectoryOptimization prog(&plant, *context, kNumTimeSamples,
                                             kTrajectoryTimeLowerBound,
                                             kTrajectoryTimeUpperBound);

  // Input limits (note that the steering limit imposed by SimpleCar is larger).
  DrivingCommand<double> lower_limit, upper_limit;
  lower_limit.set_steering_angle(-M_PI_2);
  lower_limit.set_throttle(0.0);
  lower_limit.set_brake(0.0);
  upper_limit.set_steering_angle(M_PI_2);
  upper_limit.set_throttle(std::numeric_limits<double>::infinity());
  upper_limit.set_brake(std::numeric_limits<double>::infinity());
  prog.AddInputBounds(lower_limit.get_value(), upper_limit.get_value());

  // Fix initial conditions.
  prog.AddLinearConstraint(prog.initial_state().array() ==
                           x0.get_value().array());

  // Fix final conditions.
  prog.AddLinearConstraint(prog.final_state().array() ==
                           xf.get_value().array());

  // Cost function: int_0^T [ u'u ] dt
  prog.AddRunningCost(prog.input().transpose() * prog.input());

  auto initial_state_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      {0, initial_duration}, {x0.get_value(), xf.get_value()});

  solvers::SolutionResult result =
      prog.SolveTraj(initial_duration, PiecewisePolynomial<double>(),
                     initial_state_trajectory);

  EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound);

  // Plot the solution.
  Eigen::MatrixXd inputs;
  Eigen::MatrixXd states;
  std::vector<double> times_out;
  prog.GetResultSamples(&inputs, &states, &times_out);
  lcm::LcmCallMatlab("plot",states.row(SimpleCarStateIndices::kX),states.row(SimpleCarStateIndices::kY));
  lcm::LcmCallMatlab("xlabel","x (m)");
  lcm::LcmCallMatlab("ylabel","y (m)");

}

}  // namespace
}  // namespace automotive
}  // namespace drake
