#include "drake/automotive/simple_car_to_euler_floating_joint.h"

#include <memory>

#include <gtest/gtest.h>

namespace drake {
namespace automotive {
namespace {

// Tests the conversion from the vehicle's state to the visualization's state
// when the origin of the vehicle's model's frame, Mo, is equal to the origin of
// the vehicle's visualization's frame, Vo.
GTEST_TEST(SimpleCarToEulerFloatingJointTest, BasicTest) {
  // The device under test.
  auto dut = std::make_unique<SimpleCarToEulerFloatingJoint<double>>();
  auto context = dut->CreateDefaultContext();
  auto output = dut->AllocateOutput(*context);

  // Set an input value.
  auto value = std::make_unique<SimpleCarState<double>>();
  value->set_x(11.0);
  value->set_y(22.0);
  value->set_heading(33.0);
  context->SetInputPortValue(
      0,
      std::make_unique<systems::FreestandingInputPortValue>(std::move(value)));

  // Grab a pointer to where the CalcOutput results end up.
  const EulerFloatingJointState<double>* const result =
      dynamic_cast<const EulerFloatingJointState<double>*>(
          output->get_vector_data(0));
  ASSERT_NE(nullptr, result);

  // Output matches the input.
  dut->CalcOutput(*context, output.get());
  // The following is hard-coded to match prius.sdf and prius_with_lidar.sdf.
  const double kp_MoVo{1.40948};
  EXPECT_EQ(11.0 + kp_MoVo * std::cos(33), result->x());
  EXPECT_EQ(22.0 + kp_MoVo * std::sin(33), result->y());
  EXPECT_EQ(0.0, result->z());
  EXPECT_EQ(33.0, result->yaw());
  EXPECT_EQ(0.0, result->pitch());
  EXPECT_EQ(0.0, result->roll());
}

// Tests the conversion from the vehicle's state to the visualization's state.
// This is necessary because prius_with_lidar.sdf has a non-zero distance
// between its origin (which is in the middle of the chassis_floor) and
// SimpleCar's origin (which is in the middle of the rear axle).
GTEST_TEST(SimpleCarToEulerFloatingJointTest, XOriginOffsetTest) {
  const double kX{5};
  const double kY{0};
  const double kZeroHeading{0};
  // The following is hard-coded to match prius.sdf and prius_with_lidar.sdf.
  const double kp_MoVo{1.40948};
  // The device under test.
  auto dut = std::make_unique<SimpleCarToEulerFloatingJoint<double>>();
  auto context = dut->CreateDefaultContext();
  auto output = dut->AllocateOutput(*context);

  // Grabs a pointer to where the CalcOutput results end up.
  const EulerFloatingJointState<double>* const result =
      dynamic_cast<const EulerFloatingJointState<double>*>(
          output->get_vector_data(0));
  ASSERT_NE(nullptr, result);

  // Configures M's +X axis to be coincident with the world frame's (W's)
  // +X axis. This should result in Vo's X-coordinate to be offset by kP_MoVo.
  {
    auto value = std::make_unique<SimpleCarState<double>>();
    value->set_x(kX);
    value->set_y(kY);
    value->set_heading(kZeroHeading);
    context->SetInputPortValue(
        0, std::make_unique<systems::FreestandingInputPortValue>(
               std::move(value)));
  }

  dut->CalcOutput(*context, output.get());
  EXPECT_EQ(result->x(), kX + kp_MoVo);
  EXPECT_EQ(result->y(), kY);
  EXPECT_EQ(result->z(), 0);
  EXPECT_EQ(result->yaw(), kZeroHeading);
  EXPECT_EQ(result->pitch(), 0);
  EXPECT_EQ(result->roll(), 0);

  // Configures M's +X axis to be coincident with the world frame's (W's)
  // +Y axis. This should result in Vo's Y-coordinate to be offset by kP_MoVo.
  const double kLeftHeading{M_PI_2};
  {
    auto value = std::make_unique<SimpleCarState<double>>();
    value->set_x(kX);
    value->set_y(kY);
    value->set_heading(kLeftHeading);
    context->SetInputPortValue(
        0, std::make_unique<systems::FreestandingInputPortValue>(
               std::move(value)));
  }

  dut->CalcOutput(*context, output.get());
  EXPECT_EQ(result->x(), kX);
  EXPECT_EQ(result->y(), kY + kp_MoVo);
  EXPECT_EQ(result->z(), 0);
  EXPECT_EQ(result->yaw(), kLeftHeading);
  EXPECT_EQ(result->pitch(), 0);
  EXPECT_EQ(result->roll(), 0);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
