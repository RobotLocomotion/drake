#include "drake/automotive/trajectory_car.h"

#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/curve2.h"
#include "drake/systems/analysis/simulator.h"

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
  const double start_position{0.0};
  EXPECT_THROW((TrajectoryCar<double>{empty_curve, speed, start_position}),
               std::exception);
}

// Check the car's progress along some simple paths.  We just want to test our
// ImplCalcFoo() methods.  We can assume that Curve2 is correct, because has its
// own unit test.
GTEST_TEST(TrajectoryCarTest, SegmentTest) {
  struct Case {
    double heading;
    double distance;
    double speed;
    double start_position;
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

    // Various distances / speeds.
    { 0.125 * M_PI, 10.0,  1.0, 0.0},
    { 0.125 * M_PI,  1.0,  10.0, 0.0},
    { 0.125 * M_PI,  1.0,  0.1, 0.0},

    // Various starting positions.
    { 0.125 * M_PI, 1.0, 1.0, 100.0},
    { 0.125 * M_PI, 1.0, 1.0,  10.0},
    { 0.125 * M_PI, 1.0, 1.0,   -10.0},
    { 0.125 * M_PI, 1.0, 1.0,  -100.0},
  };

  for (const auto& it : cases) {
    // Choose an arbitrary start point, then add just one segment
    // leaving the start point, based on the parameters in the test case.
    const Point2d start{20.0, 30.0};
    const Point2d heading_vector{std::cos(it.heading), std::sin(it.heading)};
    const std::vector<Point2d> waypoints{
      start,  // BR
      start + (heading_vector * it.distance),
    };
    const Curve2d curve{waypoints};
    // The "device under test".
    const TrajectoryCar<double> car_dut{curve, it.speed, it.start_position};

    // Check that the systems' outputs over time are correct over the
    // entire duration of the trajectory.
    const double total_distance = it.distance;
    const double finish_position = it.start_position + total_distance;
    const double start_time = it.start_position / it.speed;
    const double end_time = finish_position / it.speed;

    systems::Simulator<double> simulator(car_dut);
    const systems::Context<double>* context =
        simulator.get_mutable_context();
    std::unique_ptr<systems::SystemOutput<double>> all_output =
        car_dut.AllocateOutput(*context);

    simulator.Initialize();

    for (double time = start_time; time <= end_time; time += 0.1) {
      simulator.StepTo(time - start_time);

      const double fractional_progress =
          std::min(std::max(0.0, (time * it.speed) / total_distance), 1.0);
      const Point2d expected_position =
          start + (heading_vector * it.distance * fractional_progress);
      const double kMaxErrorPos = 1e-6;
      const double kMaxErrorRad = 1e-6;

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

      // N.B. We tolerate some small integration errors.
      EXPECT_NEAR(expected_position(0), raw_pose->x(), kMaxErrorPos);
      EXPECT_NEAR(expected_position(1), raw_pose->y(), kMaxErrorPos);
      EXPECT_NEAR(it.heading, raw_pose->heading(), kMaxErrorRad);
      EXPECT_DOUBLE_EQ(it.speed, raw_pose->velocity());

      // Tests the PoseVector output.
      const PoseVector<double>* pose =
          dynamic_cast<const PoseVector<double>*>(
              all_output->get_vector_data(
                  car_dut.pose_output().get_index()));
      ASSERT_NE(nullptr, pose);
      EXPECT_EQ(PoseVector<double>::kSize, pose->size());

      // N.B. We tolerate some small integration errors.
      EXPECT_NEAR(expected_position(0),
                  pose->get_translation().translation().x(), kMaxErrorPos);
      EXPECT_NEAR(expected_position(1),
                  pose->get_translation().translation().y(), kMaxErrorPos);
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

// TODO(jadecastro): Test when the car's speed is provided as an input, like in
// MaliputRailcar.


// TODO(jadecastro): Test ToAutoDiffXd.


// TODO(jadecastro): Test SetDefaultState.


}  // namespace
}  // namespace automotive
}  // namespace drake
