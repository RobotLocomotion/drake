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
  const double x0 = 0,  z0 = 0,  xDt0 = 0,  zDt0 = 0;
  const double wx0 = 0,  wy0 = 0,  wz0 = 0;
  BowlingBallAnalyticalSolution<double> ball;
  ball.SetInitialValues(x0, z0, xDt0, zDt0, wx0, wy0, wz0);

  // Expected special-case solution is no motion with rolling starting at t = 0.
  const Vector3<double> p_NoBcm_N(x0, ball.radius(), z0);
  const Vector3<double> v_NBcm_N(xDt0, 0, zDt0);
  const Vector3<double> w_NB_N(wx0, wy0, wz0);

  const double max_diff = ball.CalcMaxDifferenceWithAnalyticalSolution(
                               0, p_NoBcm_N, v_NBcm_N, w_NB_N);
  EXPECT_LE(max_diff, kEpsilon);
}


}  // namespace
}  // namespace bowling_ball
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
