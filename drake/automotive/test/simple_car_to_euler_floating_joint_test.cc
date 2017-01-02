#include "drake/automotive/simple_car_to_euler_floating_joint.h"

#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace automotive {
namespace {

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
  context->SetInputPort(
      0, std::make_unique<systems::FreestandingInputPort>(std::move(value)));

  // Grab a pointer to where the CalcOutput results end up.
  const EulerFloatingJointState<double>* const result =
      dynamic_cast<const EulerFloatingJointState<double>*>(
          output->get_vector_data(0));
  ASSERT_NE(nullptr, result);

  // Output matches the input.
  dut->CalcOutput(*context, output.get());
  EXPECT_EQ(11.0, result->x());
  EXPECT_EQ(22.0, result->y());
  EXPECT_EQ(0.0, result->z());
  EXPECT_EQ(33.0, result->yaw());
  EXPECT_EQ(0.0, result->pitch());
  EXPECT_EQ(0.0, result->roll());
}

}  // namespace
}  // namespace automotive
}  // namespace drake
