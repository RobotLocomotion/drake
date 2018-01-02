#include "drake/common/trajectories/qp_spline/spline_generation.h"

#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace {

GTEST_TEST(testSplineGeneration, BasicTest) {
  int num_segments = 3;

  std::vector<double> segment_times;
  double t0 = 0.0;
  segment_times.push_back(t0);
  for (int i = 0; i < num_segments; ++i) {
    double duration = 1.0;
    segment_times.push_back(segment_times[i] + duration);
  }

  // These are random.
  double x0 = 0.45117;
  double xd0 = 0.75249;
  double xf = 0.41839;
  double xdf = 0.55227;
  double x1 = 0.81312;
  double x2 = 0.36320;

  Eigen::Vector2d xi(x1, x2);
  PiecewisePolynomial<double> result =
      nWaypointCubicSpline(segment_times, x0, xd0, xf, xdf, xi);

  for (int i = 0; i < num_segments; i++) {
    EXPECT_EQ(segment_times[i], result.getStartTime(i));
  }
  EXPECT_EQ(segment_times[num_segments], result.getEndTime(num_segments - 1));

  // Check value constraints.
  double tol = 1e-10;
  PiecewisePolynomial<double> derivative = result.derivative();
  PiecewisePolynomial<double> second_derivative = derivative.derivative();

  EXPECT_NEAR(result.scalarValue(result.getStartTime(0)), x0, tol);
  EXPECT_NEAR(derivative.scalarValue(result.getStartTime(0)), xd0, tol);

  EXPECT_NEAR(result.scalarValue(result.getEndTime(num_segments - 1)), xf, tol);
  EXPECT_NEAR(derivative.scalarValue(result.getEndTime(num_segments - 1)), xdf,
              tol);

  EXPECT_NEAR(result.scalarValue(result.getStartTime(1)), x1, tol);
  EXPECT_NEAR(result.scalarValue(result.getStartTime(2)), x2, tol);

  // Check continuity constraints.
  double eps = 1e-10;
  int num_knots = num_segments - 1;
  for (int i = 0; i < num_knots; i++) {
    double t_knot = result.getEndTime(i);
    EXPECT_NEAR(result.scalarValue(t_knot - eps),
                result.scalarValue(t_knot + eps), 1e-8);
    EXPECT_NEAR(derivative.scalarValue(t_knot - eps),
                derivative.scalarValue(t_knot + eps), 1e-8);
    EXPECT_NEAR(second_derivative.scalarValue(t_knot - eps),
                second_derivative.scalarValue(t_knot + eps), 1e-8);
  }
}

}  // namespace
}  // namespace drake
