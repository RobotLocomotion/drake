#include "drake/automotive/car_simulation.h"

#include "gtest/gtest.h"

#include "drake/automotive/car_simulation.h"

namespace drake {
namespace automotive {
namespace {

GTEST_TEST(CarSimulationTest, SimpleCarVisualizationAdapter) {
  // The "device under test".
  auto dut = CreateSimpleCarVisualizationAdapter();

  const double time = 0.0;
  const drake::NullVector<double> state_vector{};
  SimpleCarState1<double> input_vector{};
  EulerFloatingJointState1<double> output_vector{};
  output_vector = dut->output(time, state_vector, input_vector);

  EXPECT_DOUBLE_EQ(output_vector.x(), 0.0);
  EXPECT_DOUBLE_EQ(output_vector.y(), 0.0);
  EXPECT_DOUBLE_EQ(output_vector.z(), 0.0);
  EXPECT_DOUBLE_EQ(output_vector.roll(), 0.0);
  EXPECT_DOUBLE_EQ(output_vector.pitch(), 0.0);
  EXPECT_DOUBLE_EQ(output_vector.yaw(), 0.0);

  input_vector.set_x(22.0);
  input_vector.set_y(33.0);
  input_vector.set_heading(1.0);
  input_vector.set_velocity(11.0);

  output_vector = dut->output(time, state_vector, input_vector);

  EXPECT_DOUBLE_EQ(output_vector.x(), 22.0);
  EXPECT_DOUBLE_EQ(output_vector.y(), 33.0);
  EXPECT_DOUBLE_EQ(output_vector.z(), 0.0);
  EXPECT_DOUBLE_EQ(output_vector.roll(), 0.0);
  EXPECT_DOUBLE_EQ(output_vector.pitch(), 0.0);
  EXPECT_DOUBLE_EQ(output_vector.yaw(), 1.0);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
