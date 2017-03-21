#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/Pendulum/pendulum_plant.h"
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

  const PendulumPlant<double> pendulum;
  auto context = pendulum.CreateDefaultContext();

  systems::DircolTrajectoryOptimization dircol_traj(
      &pendulum, *context,
      kNumTimeSamples, kTrajectoryTimeLowerBound,
      kTrajectoryTimeUpperBound);
  AddSwingUpTrajectoryParams(kNumTimeSamples, x0, xG, &dircol_traj);

  const double timespan_init = 4;
  auto traj_init_x = PiecewisePolynomialType::FirstOrderHold(
      {0, timespan_init}, {x0, xG});
  const SolutionResult result =
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
  const PiecewisePolynomialTrajectory state_traj =
      dircol_traj.ReconstructStateTrajectory();
  for (int i = 0; i < kNumTimeSamples; ++i) {
    EXPECT_TRUE(CompareMatrices(states.col(i), state_traj.value(times_out[i]),
                                1e-10, MatrixCompareType::absolute));
  }

  // Test interpolation.
  const double t1(times_out[2]);
  const double t2(times_out[3]);
  // The value at t2 is larger than the value at t1, for this example.

  const Eigen::MatrixXd linear_avr_of_2_points((state_traj.value(t2) +
      state_traj.value(t1)) * 0.5);
  const Eigen::MatrixXd spline_interpolation(state_traj.value((t2 + t1) *
      0.5));

  // In this particular point we expect the spline to be quite different from
  // linear_avr_of_2_points at the midpoint: not trying to be exact, but want a
  // regression test.
  double abs_diff =
      std::fabs(linear_avr_of_2_points(0, 0) - spline_interpolation(0, 0));
  EXPECT_LT(abs_diff, 0.1);
  EXPECT_GT(abs_diff, 0.09);
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake
