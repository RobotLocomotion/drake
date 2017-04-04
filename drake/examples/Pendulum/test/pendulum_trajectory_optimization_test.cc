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

  systems::DircolTrajectoryOptimization dircol(
      &pendulum, *context,
      kNumTimeSamples, kTrajectoryTimeLowerBound,
      kTrajectoryTimeUpperBound);
  AddSwingUpTrajectoryParams(x0, xG, &dircol);

  const double timespan_init = 4;
  auto traj_init_x = PiecewisePolynomialType::FirstOrderHold(
      {0, timespan_init}, {x0, xG});
  const SolutionResult result =
      dircol.SolveTraj(timespan_init, PiecewisePolynomialType(),
                            traj_init_x);
  ASSERT_EQ(result, SolutionResult::kSolutionFound) << "Result is an Error";

  Eigen::MatrixXd inputs;
  Eigen::MatrixXd states;
  std::vector<double> times_out;

  dircol.GetResultSamples(&inputs, &states, &times_out);
  EXPECT_TRUE(CompareMatrices(x0, states.col(0),
                              1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(xG, states.col(kNumTimeSamples - 1),
                              1e-10, MatrixCompareType::absolute));
  const PiecewisePolynomialTrajectory state_traj =
      dircol.ReconstructStateTrajectory();
  for (int i = 0; i < kNumTimeSamples; ++i) {
    EXPECT_TRUE(CompareMatrices(states.col(i), state_traj.value(times_out[i]),
                                1e-10, MatrixCompareType::absolute));
  }

  // Test interpolation
  for (int i = 0; i < kNumTimeSamples - 1; ++i) {
    double t_l = times_out[i];
    double t_r = times_out[i + 1];
    double h_i = t_r - t_l;
    const Eigen::Vector2d x_l = state_traj.value(t_l);
    const Eigen::Vector2d x_r = state_traj.value(t_r);
    const Eigen::Vector2d xdot_l = state_traj.derivative()->value(t_l);
    const Eigen::Vector2d xdot_r = state_traj.derivative()->value(t_r);
    // For a cubic polynomial, if x_l, x_r, xdot_l, xdot_r are known, then the
    // 4 coefficients of the cubic polynomial can be computed from x_l, x_r,
    // xdot_l, xdot_r, and thus the midpoint of the polynomial can be computed
    // from x_l, x_r, xdot_l, xdot_r. Check out equation 9 of
    // Direct Trajectory Optimization Using Nonlinear Programming and
    // Collocation. By C.R. Hargraves and S.W. Paris
    const Eigen::Vector2d xm = 0.5 * (x_l + x_r) + h_i / 8 * (xdot_l - xdot_r);
    const Eigen::Vector2d spline_midpoint(state_traj.value((t_l + t_r) / 2));
    EXPECT_TRUE(CompareMatrices(xm, spline_midpoint, 1E-10,
                                MatrixCompareType::absolute));
  }
  // TODO(hongkai.dai): Add a test on the optimal cost, when #5685 is merged, so
  // as to make sure the cost parsed in the optimization program, is the same
  // integrated cost as we want.
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake
