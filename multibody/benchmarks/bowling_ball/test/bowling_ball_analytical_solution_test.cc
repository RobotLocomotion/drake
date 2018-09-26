#include "drake/multibody/benchmarks/bowling_ball/bowling_ball_analytical_solution.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace benchmarks {
namespace bowling_ball {
namespace {

// Compare the analytical solution with the expected results.
constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Test trivial case of ball not moving.
GTEST_TEST(BowlingBallAnalyticalSolution, BowlingBallDoesNotMove) {
  BowlingBallAnalyticalSolution<double> ball;
  const double wx0 = 0,  wy0 = 0,  wz0 = 0;
  const double x0 = 0,  z0 = 0,  xDt0 = 0,  zDt0 = 0;
  ball.SetInitialValues(x0, z0, xDt0, zDt0, wx0, wy0, wz0);

  // Expected solution is no motion, with rolling starting at t = 0.
  const double t = 0;
  const Vector3<double> p_NoBcm_N(x0, ball.radius(), z0);
  const Vector3<double> v_NBcm_N(xDt0, 0, zDt0);
  const Vector3<double> w_NB_N(wx0, wy0, wz0);

  const double max_diff = ball.CalcMaxDifferenceWithAnalyticalSolution(
                               t, p_NoBcm_N, v_NBcm_N, w_NB_N);
  EXPECT_LE(max_diff, 2 * kEpsilon);
}

// Test case of ball that rolls in a straight line.
GTEST_TEST(BowlingBallAnalyticalSolution, BowlingBallStraightLineRolling) {
  BowlingBallAnalyticalSolution<double> ball;
  const double wx0 = 1,  wy0 = 0,  wz0 = 0;
  const double x0 = 0,  z0 = 0,  xDt0 = 0,  zDt0 = wx0 * ball.radius();
  ball.SetInitialValues(x0, z0, xDt0, zDt0, wx0, wy0, wz0);

  // Expected solution is rolling in a straight line, starting at t = 0.
  const double t = 0.12345;
  const Vector3<double> p_NoBcm_N(x0, ball.radius(), z0 + zDt0 * t);
  const Vector3<double> v_NBcm_N(xDt0, 0, zDt0);
  const Vector3<double> w_NB_N(wx0, wy0, wz0);

  const double max_diff = ball.CalcMaxDifferenceWithAnalyticalSolution(
                               t, p_NoBcm_N, v_NBcm_N, w_NB_N);
  EXPECT_LE(max_diff, 2 * kEpsilon);
}

// Test case of ball that slides in a straight line.
GTEST_TEST(BowlingBallAnalyticalSolution, BowlingBallStraightLineSliding) {
  BowlingBallAnalyticalSolution<double> ball;
  const double wx0 = 0,  wy0 = 0,  wz0 = 0;
  const double x0 = 0,  z0 = 0,  xDt0 = 0,  zDt0 = 5;
  ball.SetInitialValues(x0, z0, xDt0, zDt0, wx0, wy0, wz0);
  ball.SetCoefficientOfKineticFriction(0);

  // Expected solution is always sliding in a straight line.
  const double t = 0.12345;
  const Vector3<double> p_NoBcm_N(x0, ball.radius(), z0 + zDt0 * t);
  const Vector3<double> v_NBcm_N(xDt0, 0, zDt0);
  const Vector3<double> w_NB_N(wx0, wy0, wz0);

  const double max_diff = ball.CalcMaxDifferenceWithAnalyticalSolution(
      t, p_NoBcm_N, v_NBcm_N, w_NB_N);
  EXPECT_LE(max_diff, 2 * kEpsilon);
}

// Test case of ball that transitions from sliding to rolling.
// Note: This tests default initialization (default parameters).
GTEST_TEST(BowlingBallAnalyticalSolution, BowlingBallSlidingToRolling) {
  BowlingBallAnalyticalSolution<double> ball;

  // Verify the detail parameter start with sliding and transition to rolling.
  const double t_start_rolling = ball.CalcTimeAtWhichRollingStarts();
  const double time_diff = std::abs(2.054 - t_start_rolling);
  EXPECT_LE(time_diff, 0.002);

  // Compare with MotionGenesis test results near half-way to start of rolling.
  double t = 1.0;
  Vector3<double> p_NoBcm_N(0.1888469, ball.radius(), 7.443151);
  Vector3<double> v_NBcm_N(0.3776938, 0, 6.886302);
  Vector3<double> w_NB_N(25.79123, 0, -16.38603);
  double max_diff = ball.CalcMaxDifferenceWithAnalyticalSolution(
                         t, p_NoBcm_N, v_NBcm_N, w_NB_N);
  EXPECT_LE(max_diff, 1.0E-5);

  // Test results at start of rolling (compare with MotionGenesis results).
  t = 2.054;
  p_NoBcm_N = Vector3<double>(0.7967288, ball.radius(), 14.08270);
  v_NBcm_N = Vector3<double>(0.7751817, 0, 5.714238);
  w_NB_N = Vector3<double>(52.93412, 0, -7.180933);
  max_diff = ball.CalcMaxDifferenceWithAnalyticalSolution(
                  t, p_NoBcm_N, v_NBcm_N, w_NB_N);
  EXPECT_LE(max_diff, 1.0E-5);

  // Test results at start of rolling (compare with MotionGenesis results).
  t = 2.4;
  p_NoBcm_N = Vector3<double>(1.064940, ball.radius(), 16.05983);
  v_NBcm_N = Vector3<double>(0.7751817, 0, 5.714238);
  w_NB_N = Vector3<double>(52.93412, 0, -7.180933);
  max_diff = ball.CalcMaxDifferenceWithAnalyticalSolution(
      t, p_NoBcm_N, v_NBcm_N, w_NB_N);
  EXPECT_LE(max_diff, 1.0E-5);
}

}  // namespace
}  // namespace bowling_ball
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
