#include "drake/automotive/curve2.h"

#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace automotive {
namespace {

typedef Vector1d V1;
using Point2d = typename WaypointT<double>::Point2d;

// Checks the PositionResult::position, Result::speed against the expected
// Waypoint.  PositionResult::position_deriv is always expected to be √2/2.
template <typename T = double>
void CheckResult(const PositionResult<T>& actual, const Waypoint& expected,
                 double exp_speed_dot) {
  EXPECT_TRUE(CompareMatrices(actual.waypoint.position, expected.position));
  EXPECT_EQ(actual.waypoint.speed, expected.speed);
  EXPECT_TRUE(CompareMatrices(actual.position_deriv,
                              Point2d{M_SQRT1_2, M_SQRT1_2}, 1e-12));
  EXPECT_NEAR(actual.speed_dot, exp_speed_dot, 1e-12);
}

// Checks the value() and derivatives() of the actual result against the
// expected Waypoint.  Derivatives for each entry should all be equivalent.
void CheckResult(const PositionResult<AutoDiffXd>& actual,
                 const Waypoint& expected,
                 double exp_speed_dot,
                 const Eigen::VectorXd& exp_pos_derivs,
                 const Eigen::VectorXd& exp_speed_derivs) {
  CheckResult(actual, expected, exp_speed_dot);
  EXPECT_TRUE(
      CompareMatrices(actual.waypoint.position[0].derivatives(), exp_pos_derivs,
                      1e-12));
  EXPECT_TRUE(
      CompareMatrices(actual.waypoint.position[1].derivatives(), exp_pos_derivs,
                      1e-12));
  EXPECT_TRUE(
      CompareMatrices(actual.waypoint.speed.derivatives(), exp_speed_derivs,
                      1e-12));
}

// An empty curve.
GTEST_TEST(Curve2Test, EmptyTest) {
  const std::vector<Waypoint> empty_waypoints{};
  const Curve2<double> empty_curve{empty_waypoints};
  EXPECT_DOUBLE_EQ(empty_curve.path_length(), 0.0);
}

// A single waypoint.
GTEST_TEST(Curve2Test, SingleWaypointTest) {
  const Waypoint point(Point2d{1.0, 2.0});
  EXPECT_TRUE(CompareMatrices(point.position, Point2d{1.0, 2.0}));
  EXPECT_EQ(point.speed, 0);
  const std::vector<Waypoint> single_waypoint{
      point,
  };
  EXPECT_THROW(Curve2<double>{single_waypoint}, std::exception);
}

// Negative speeds.
GTEST_TEST(Curve2Test, NegativeSpeedTest) {
  const Waypoint start_point(Point2d{1.0, 2.0}, -4.0);
  const Waypoint end_point(Point2d{2.0, 3.0}, -4.0);
  EXPECT_EQ(start_point.speed, -4.0);
  EXPECT_EQ(end_point.speed, -4.0);
  const std::vector<Waypoint> segment_waypoints{start_point, end_point};
  EXPECT_THROW(Curve2<double>{segment_waypoints}, std::exception);
}

// A single segment.
GTEST_TEST(Curve2Test, BasicTest) {
  using std::sqrt;

  const double kInitialSpeed = 4.0;
  const double kAcceleration = 5.0;
  const double kFinalSpeed =    // √(2ad + v0²)
      sqrt(2 * kAcceleration * M_SQRT2 + kInitialSpeed * kInitialSpeed);

  // Configure two Point2 positions spaced a unit-Manhattan distance (√2)
  // apart.
  const Waypoint start_point(Point2d{1.0, 2.0}, kInitialSpeed);
  const Waypoint end_point(Point2d{2.0, 3.0}, kFinalSpeed);
  const std::vector<Waypoint> segment_waypoints{start_point, end_point};
  const Curve2<double> segment{segment_waypoints};
  EXPECT_DOUBLE_EQ(segment.path_length(), M_SQRT2);
  auto waypoints = segment.waypoints();
  EXPECT_GE(waypoints.size(), 2);
  EXPECT_EQ(waypoints[0].position, start_point.position);
  EXPECT_EQ(waypoints[1].position, end_point.position);

  const PositionResult<double> before_start = segment.CalcPositionResult(-1.0);
  const PositionResult<double> at_start = segment.CalcPositionResult(0.0);
  const PositionResult<double> at_midpoint =
      segment.CalcPositionResult(0.5 * segment.path_length());
  const PositionResult<double> at_end =
      segment.CalcPositionResult(segment.path_length());
  const PositionResult<double> after_end = segment.CalcPositionResult(10.0);

  CheckResult(before_start, start_point, kAcceleration);
  CheckResult(at_start, start_point, kAcceleration);
  const Point2d mid_position =
      0.5 * (start_point.position + end_point.position);
  const double mid_speed = 0.5 * (start_point.speed + end_point.speed);
  CheckResult(at_midpoint, {mid_position, mid_speed}, kAcceleration);
  CheckResult(at_end, end_point, kAcceleration);
  CheckResult(after_end, end_point, kAcceleration);
}

// AutoDiffXd coherency.
GTEST_TEST(Curve2Test, AutoDiffTest) {
  using std::sqrt;

  const double kInitialSpeed = 4.0;
  const double kAcceleration = 5.0;
  const double kFinalSpeed =    // √(2ad + v0²)
      sqrt(2 * kAcceleration * M_SQRT2 + kInitialSpeed * kInitialSpeed);
  const double kDeltaSpeed = kFinalSpeed - kInitialSpeed;

  // Configure two Point2 positions spaced a unit-Manhattan distance (√2)
  // apart.
  const Waypoint start_point(Point2d{1.0, 2.0}, kInitialSpeed);
  const Waypoint end_point(Point2d{2.0, 3.0}, kFinalSpeed);
  const std::vector<Waypoint> segment_waypoints{start_point, end_point};
  const Curve2<AutoDiffXd> segment{segment_waypoints};
  auto waypoints = segment.waypoints();

  const V1 derivative{1.};
  const PositionResult<AutoDiffXd> before_start =
      segment.CalcPositionResult(AutoDiffXd(-1, derivative));
  const PositionResult<AutoDiffXd> at_start =
      segment.CalcPositionResult(AutoDiffXd(0.0, derivative));
  const PositionResult<AutoDiffXd> at_midpoint = segment.CalcPositionResult(
      AutoDiffXd(0.5 * segment.path_length(), derivative));
  const PositionResult<AutoDiffXd> at_end =
      segment.CalcPositionResult(AutoDiffXd(segment.path_length(), derivative));
  const PositionResult<AutoDiffXd> after_end =
      segment.CalcPositionResult(AutoDiffXd(10.0, derivative));

  CheckResult(before_start, start_point, kAcceleration, V1{0.}, V1{0.});
  CheckResult(at_start, start_point, kAcceleration, V1{0.}, V1{0.});
  const Point2d mid_position =
      0.5 * (start_point.position + end_point.position);
  const double mid_speed = 0.5 * (start_point.speed + end_point.speed);
  CheckResult(at_midpoint, {mid_position, mid_speed}, kAcceleration,
              V1{M_SQRT1_2}, V1{kDeltaSpeed * M_SQRT1_2});
  CheckResult(at_end, end_point, kAcceleration, V1{M_SQRT1_2},
              V1{kDeltaSpeed * M_SQRT1_2});
  CheckResult(after_end, end_point, kAcceleration, V1{0.}, V1{0.});
}

}  // namespace
}  // namespace automotive
}  // namespace drake
