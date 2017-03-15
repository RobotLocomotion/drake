#include "drake/automotive/trajectory_car.h"

#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "drake/automotive/curve2.h"

namespace drake {
namespace automotive {
namespace {

typedef Curve2<double> Curve2d;
typedef Curve2d::Point2 Point2d;

using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;

// Empty curves are rejected.
GTEST_TEST(TrajectoryCarTest, StationaryTest) {
  const std::vector<Point2d> empty_waypoints{};
  const Curve2d empty_curve{empty_waypoints};
  const double speed{99.0};
  const double start_time{0.0};
  EXPECT_THROW((TrajectoryCar<double>{empty_curve, speed, start_time}),
               std::exception);
}

// Check the car's progress along some simple paths.  We just want to
// test our dynamics() method.  We can assume that Curve2 is correct,
// because has its own unit test.
GTEST_TEST(TrajectoryCarTest, SegmentTest) {
  struct Case {
    double heading;
    double distance;
    double speed;
    double start_time;
  };
  const std::vector<Case> cases{
    // Various headings.
    {-0.999 * M_PI, 1.0, 1.0, 0.0},
    {-0.875 * M_PI, 1.0, 1.0, 0.0},
    {-0.500 * M_PI, 1.0, 1.0, 0.0},
    {-0.125 * M_PI, 1.0, 1.0, 0.0},
    { 0.000 * M_PI, 1.0, 1.0, 0.0},
    { 0.125 * M_PI, 1.0, 1.0, 0.0},
    { 0.500 * M_PI, 1.0, 1.0, 0.0},
    { 0.875 * M_PI, 1.0, 1.0, 0.0},
    { 0.999 * M_PI, 1.0, 1.0, 0.0},

    // Various distance / speeds.
    { 0.125 * M_PI, 10.0,  1.0, 0.0},
    { 0.125 * M_PI,  1.0, 10.0, 0.0},
    { 0.125 * M_PI,  1.0,  0.1, 0.0},

    // Various start times.
    { 0.125 * M_PI, 1.0, 1.0, -100.0},
    { 0.125 * M_PI, 1.0, 1.0,  -10.0},
    { 0.125 * M_PI, 1.0, 1.0,   10.0},
    { 0.125 * M_PI, 1.0, 1.0,  100.0},
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
    // The "device under test".
    const TrajectoryCar<double> car_dut{curve, it.speed, it.start_time};

    // The test inputs (time) and outputs.
    std::unique_ptr<systems::Context<double>> context =
        car_dut.CreateDefaultContext();
    std::unique_ptr<systems::SystemOutput<double>> all_output =
        car_dut.AllocateOutput(*context);

    // Check that the systems' outputs over time are correct over the
    // entire duration of the trajectory, but also including some time
    // before and after the start and end of the trajectory.
    const double total_duration = (it.distance / it.speed);
    const double finish_time = it.start_time + total_duration;
    const double low_time = it.start_time - 1.0;
    const double high_time = finish_time + 1.0;
    for (double time = low_time; time <= high_time; time += 0.1) {
      const double fractional_progress =
          std::min(std::max(0.0, (time - it.start_time) / total_duration), 1.0);
      const Point2d expected_position =
          start + (heading_vector * it.distance * fractional_progress);
      const double kMaxErrorRad = 1e-6;

      context->set_time(time);
      car_dut.CalcOutput(*context, all_output.get());

      ASSERT_EQ(3, all_output->get_num_ports());

      // Tests the raw pose output.
      const SimpleCarState<double>* raw_pose =
          dynamic_cast<const SimpleCarState<double>*>(
              all_output->get_vector_data(
                  car_dut.raw_pose_output().get_index()));
      ASSERT_NE(nullptr, raw_pose);
      EXPECT_EQ(SimpleCarStateIndices::kNumCoordinates,
                raw_pose->size());

      EXPECT_DOUBLE_EQ(expected_position(0), raw_pose->x());
      EXPECT_DOUBLE_EQ(expected_position(1), raw_pose->y());
      EXPECT_NEAR(it.heading, raw_pose->heading(), kMaxErrorRad);
      EXPECT_DOUBLE_EQ(it.speed, raw_pose->velocity());

      // Tests the PoseVector output.
      const PoseVector<double>* pose =
          dynamic_cast<const PoseVector<double>*>(
              all_output->get_vector_data(
                  car_dut.pose_output().get_index()));
      ASSERT_NE(nullptr, pose);
      EXPECT_EQ(PoseVector<double>::kSize, pose->size());

      EXPECT_DOUBLE_EQ(expected_position(0),
                       pose->get_translation().translation().x());
      EXPECT_DOUBLE_EQ(expected_position(1),
                       pose->get_translation().translation().y());
      EXPECT_NEAR(std::cos(it.heading / 2), pose->get_rotation().w(),
                  kMaxErrorRad);
      EXPECT_NEAR(std::sin(it.heading / 2), pose->get_rotation().z(),
                  kMaxErrorRad);

      // Tests the FrameVelocity output.
      const FrameVelocity<double>* velocity =
          dynamic_cast<const FrameVelocity<double>*>(
              all_output->get_vector_data(
                  car_dut.velocity_output().get_index()));
      ASSERT_NE(nullptr, velocity);
      EXPECT_EQ(FrameVelocity<double>::kSize, velocity->size());

      EXPECT_NEAR(it.speed * cos(it.heading),
                  velocity->get_velocity().translational().x(), kMaxErrorRad);
      EXPECT_NEAR(it.speed * sin(it.heading),
                  velocity->get_velocity().translational().y(), kMaxErrorRad);
    }
  }
}

}  // namespace
}  // namespace automotive
}  // namespace drake
