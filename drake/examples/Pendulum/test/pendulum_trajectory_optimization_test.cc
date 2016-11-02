#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/Pendulum/pendulum_swing_up.h"

using drake::solvers::SolutionResult;

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

namespace drake {
namespace examples {
namespace pendulum {
namespace {

GTEST_TEST(PendulumTrajectoryOptimization,
           PendulumTrajectoryOptimizationTest) {
  const int kNumTimeSamples = 21;
  const int kTrajectoryTimeLowerBound = 2;
  const int kTrajectoryTimeUpperBound = 6;

  const Eigen::Vector2d x0(0, 0);
  const Eigen::Vector2d xG(M_PI, 0);

  solvers::DircolTrajectoryOptimization dircol_traj(
      1 /* num inputs */, 2 /* num_states */,
      kNumTimeSamples, kTrajectoryTimeLowerBound,
      kTrajectoryTimeUpperBound);
  AddSwingUpTrajectoryParams(kNumTimeSamples, x0, xG, &dircol_traj);

  const double timespan_init = 4;
  auto traj_init_x = PiecewisePolynomialType::FirstOrderHold(
      {0, timespan_init}, {x0, xG});
  SolutionResult result =
      dircol_traj.SolveTraj(timespan_init, PiecewisePolynomialType(),
                            traj_init_x);
  ASSERT_EQ(result, SolutionResult::kSolutionFound) << "Result is an Error";

  Eigen::MatrixXd inputs;
  Eigen::MatrixXd states;
  std::vector<double> times_out;

  dircol_traj.GetResultSamples(&inputs, &states, &times_out);
  EXPECT_TRUE(CompareMatrices(x0, states.col(0),
                              1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(xG, states.col(kNumTimeSamples - 1),
                              1e-10, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake
