#include "drake/common/trajectories/wrapped_trajectory.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/trajectories/function_handle_trajectory.h"

namespace drake {
namespace trajectories {
namespace internal {
namespace {

Eigen::Vector2d Circle(const double& t) {
  return Eigen::Vector2d(std::sin(t), std::cos(t));
}

Eigen::Vector2d CircleDerivative(const double& t, int order) {
  DRAKE_DEMAND(order == 2);
  return -Circle(t);
}

std::shared_ptr<const Trajectory<double>> MakeFunctionHandleTrajectory() {
  const double start_time = 0;
  const double end_time = 1;
  auto result = std::make_shared<FunctionHandleTrajectory<double>>(
      &Circle, 2, 1, start_time, end_time);
  result->set_derivative(&CircleDerivative);
  return result;
}

GTEST_TEST(WrappedTrajectoryTest, BasicTest) {
  const WrappedTrajectory<double> dut(MakeFunctionHandleTrajectory());
  EXPECT_EQ(dut.rows(), 2);
  EXPECT_EQ(dut.cols(), 1);
  EXPECT_EQ(dut.start_time(), 0);
  EXPECT_EQ(dut.end_time(), 1);
  EXPECT_TRUE(dut.has_derivative());
  const double t = 0.25;
  EXPECT_EQ(dut.value(t), Circle(t));
  EXPECT_EQ(dut.EvalDerivative(t, 2), -Circle(t));
  EXPECT_EQ(dut.MakeDerivative(2)->value(t), -Circle(t));

  auto clone = dut.Clone();
  EXPECT_EQ(clone->rows(), 2);
  EXPECT_EQ(clone->cols(), 1);
  EXPECT_EQ(clone->start_time(), 0);
  EXPECT_EQ(clone->end_time(), 1);
  EXPECT_TRUE(clone->has_derivative());
  EXPECT_EQ(clone->value(t), Circle(t));
  EXPECT_EQ(clone->EvalDerivative(t, 2), -Circle(t));
  EXPECT_EQ(clone->MakeDerivative(2)->value(t), -Circle(t));
  // We want a FunctionHandleTrajectory, not wrapped. See comment in cc file.
  EXPECT_TRUE(
      dynamic_cast<const FunctionHandleTrajectory<double>*>(clone.get()));
}

}  // namespace
}  // namespace internal
}  // namespace trajectories
}  // namespace drake
