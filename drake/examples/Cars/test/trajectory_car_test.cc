#include "drake/examples/Cars/trajectory_car.h"

#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "drake/core/Vector.h"
#include "drake/examples/Cars/curve2.h"

namespace drake {
namespace examples {
namespace cars {
namespace test {
namespace {

typedef Curve2<double> Curve2d;
typedef Curve2d::Point2 Point2d;

// Empty curves are rejected.
GTEST_TEST(TrajectoryCarTest, StationaryTest) {
  const std::vector<Point2d> empty_waypoints{};
  const Curve2d empty_curve{empty_waypoints};
  const double speed{99.0};
  EXPECT_THROW((TrajectoryCar{empty_curve, speed}), std::exception);
}

// Check the car's progress along some simple paths.  We just want to
// test our dynamics() method.  We can assume that Curve2 is correct,
// because has its own unit test.
GTEST_TEST(TrajectoryCarTest, SegmentTest) {
  struct Case {
    double heading;
    double distance;
    double speed;
  };
  const std::vector<Case> cases{
    // Various headings.
    {-0.999 * M_PI, 1.0, 1.0},
    {-0.875 * M_PI, 1.0, 1.0},
    {-0.500 * M_PI, 1.0, 1.0},
    {-0.125 * M_PI, 1.0, 1.0},
    { 0.000 * M_PI, 1.0, 1.0},
    { 0.125 * M_PI, 1.0, 1.0},
    { 0.500 * M_PI, 1.0, 1.0},
    { 0.875 * M_PI, 1.0, 1.0},
    { 0.999 * M_PI, 1.0, 1.0},

    // Various distance / speeds.
    { 0.125 * M_PI, 10.0, 1.0},
    { 0.125 * M_PI, 1.0, 10.0},
    { 0.125 * M_PI, 1.0, 0.1},
  };

  for (const auto& it : cases) {
    // Choose an arbitrary start point, then add just one segment
    // leaving the start point, based on the parameters in the Case.
    const Point2d start{20.0, 30.0};
    const Point2d heading_vector{std::cos(it.heading), std::sin(it.heading)};
    const std::vector<Point2d> waypoints{
      start,  // BR
      start + (heading_vector * it.distance),
    };
    const Curve2d curve{waypoints};
    const TrajectoryCar dut{curve, it.speed};

    // Check that the systems' outputs over time are correct over the
    // entire duration of the trajectory, but also including some time
    // before and after the start and end of the trajectory.
    const double finish_time = it.distance / it.speed;
    for (double time = -1.0; time <= (finish_time + 1.0); time += 0.1) {
      const double fractional_progress =
          std::min(std::max(0.0, time / finish_time), 1.0);
      const Point2d expected_position =
          start + (heading_vector * it.distance * fractional_progress);
      const double kMaxErrorRad = 1e-6;

      const Drake::NullVector<double> null_vector{};
      auto output = dut.output(time, null_vector, null_vector);
      EXPECT_DOUBLE_EQ(output.x(), expected_position(0));
      EXPECT_DOUBLE_EQ(output.y(), expected_position(1));
      EXPECT_NEAR(output.heading(), it.heading, kMaxErrorRad);
      EXPECT_DOUBLE_EQ(output.velocity(), it.speed);
    }
  }
}

}  // namespace
}  // namespace test
}  // namespace cars
}  // namespace examples
}  // namespace drake
