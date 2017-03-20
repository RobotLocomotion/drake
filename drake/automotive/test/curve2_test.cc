#include "drake/automotive/curve2.h"

#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace automotive {
namespace {

typedef Curve2<double> Curve2d;
typedef Curve2d::Point2 Point2d;

// An empty curve.
GTEST_TEST(Curve2Test, EmptyTest) {
  const std::vector<Point2d> empty_waypoints{};
  const Curve2d empty_curve{empty_waypoints};
  EXPECT_DOUBLE_EQ(empty_curve.path_length(), 0.0);
}

// A single waypoint.
GTEST_TEST(Curve2Test, SingleWaypointTest) {
  const std::vector<Point2d> single_waypoint{
    Point2d{1.0, 2.0},
  };
  EXPECT_THROW(Curve2d{single_waypoint}, std::exception);
}

// A single segment.
GTEST_TEST(Curve2Test, BasicTest) {
  const Point2d start_point{1.0, 2.0};
  const Point2d end_point{2.0, 3.0};
  const std::vector<Point2d> segment_waypoints{start_point, end_point};
  const Curve2d segment{segment_waypoints};
  EXPECT_DOUBLE_EQ(segment.path_length(), M_SQRT2);

  auto before_start = segment.GetPosition(-1.0);
  auto at_start = segment.GetPosition(0.0);
  auto at_end = segment.GetPosition(segment.path_length());
  auto after_end = segment.GetPosition(10.0);

  EXPECT_EQ(before_start.position, start_point);
  EXPECT_EQ(at_start.position, start_point);
  EXPECT_EQ(at_end.position, end_point);
  EXPECT_EQ(after_end.position, end_point);

  for (auto it : {before_start, at_start, at_end, after_end}) {
    EXPECT_DOUBLE_EQ(it.position_dot(0), M_SQRT1_2);
    EXPECT_DOUBLE_EQ(it.position_dot(1), M_SQRT1_2);
  }
}

}  // namespace
}  // namespace automotive
}  // namespace drake
