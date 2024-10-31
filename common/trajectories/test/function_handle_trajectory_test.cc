#include "drake/common/trajectories/function_handle_trajectory.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace trajectories {
namespace {

GTEST_TEST(FunctionHandleTrajectoryTest, BasicTest) {
  const auto circle_func = [](const double& t) {
    return Eigen::Vector2d(std::sin(t), std::cos(t));
  };
  FunctionHandleTrajectory<double> dut(circle_func, 2, 1, 0, 1);

  EXPECT_EQ(dut.rows(), 2);
  EXPECT_EQ(dut.cols(), 1);
  EXPECT_EQ(dut.start_time(), 0);
  EXPECT_EQ(dut.end_time(), 1);
  EXPECT_EQ(dut.has_derivative(), false);

  for (double t = 0; t < 1; t += 0.1) {
    EXPECT_TRUE(CompareMatrices(dut.value(t),
                                Eigen::Vector2d(std::sin(t), std::cos(t))));
  }

  auto clone = dut.Clone();
  EXPECT_NE(dynamic_cast<FunctionHandleTrajectory<double>*>(clone.get()),
            nullptr);
  EXPECT_EQ(clone->rows(), 2);
  EXPECT_EQ(clone->cols(), 1);
  EXPECT_EQ(clone->start_time(), 0);
  EXPECT_EQ(clone->end_time(), 1);
  EXPECT_EQ(clone->has_derivative(), false);

  for (double t = 0; t < 1; t += 0.1) {
    EXPECT_TRUE(CompareMatrices(clone->value(t),
                                Eigen::Vector2d(std::sin(t), std::cos(t))));
  }
}

GTEST_TEST(FunctionHandleTrajectoryTest, CopyCtor) {
  const auto sin_func = [](const double& t) {
    return Eigen::Matrix<double, 1, 1>(std::sin(t));
  };
  const auto t = 0.5;
  const auto x = Vector1d(std::sin(t));
  FunctionHandleTrajectory<double> dut(sin_func, 1, 1, 0, 1);
  EXPECT_EQ(dut.rows(), 1);
  EXPECT_EQ(dut.cols(), 1);
  EXPECT_TRUE(CompareMatrices(dut.value(t), x));

  // The 'other' copies the data from 'dut'.
  FunctionHandleTrajectory<double> other(dut);
  EXPECT_EQ(other.rows(), 1);
  EXPECT_EQ(other.cols(), 1);
  EXPECT_TRUE(CompareMatrices(other.value(t), x));
  EXPECT_NO_THROW(other.Clone());

  // The 'dut' is unchanged.
  EXPECT_EQ(dut.rows(), 1);
  EXPECT_EQ(dut.cols(), 1);
  EXPECT_TRUE(CompareMatrices(dut.value(t), x));
  EXPECT_NO_THROW(dut.Clone());
}

GTEST_TEST(FunctionHandleTrajectoryTest, MoveCtor) {
  const auto sin_func = [](const double& t) {
    return Eigen::Matrix<double, 1, 1>(std::sin(t));
  };
  FunctionHandleTrajectory<double> dut(sin_func, 1, 1, 0, 1);
  const auto t = 0.5;
  const auto x = Vector1d(std::sin(t));
  EXPECT_EQ(dut.rows(), 1);
  EXPECT_EQ(dut.cols(), 1);
  EXPECT_TRUE(CompareMatrices(dut.value(t), x));

  // The 'other' takes over the data from 'dut'.
  FunctionHandleTrajectory<double> other(std::move(dut));
  EXPECT_EQ(other.rows(), 1);
  EXPECT_EQ(other.cols(), 1);
  EXPECT_TRUE(CompareMatrices(other.value(t), x));
  EXPECT_NO_THROW(other.Clone());

  // The 'dut' is empty now.
  EXPECT_EQ(dut.rows(), 0);
  EXPECT_EQ(dut.cols(), 0);
  EXPECT_EQ(dut.value(t).rows(), 0);
  EXPECT_EQ(dut.value(t).cols(), 0);
  EXPECT_NO_THROW(dut.Clone());
}

GTEST_TEST(FunctionHandleTrajectoryTest, Invalid) {
  const auto sin_func = [](const double& t) {
    return Eigen::Matrix<double, 1, 1>(std::sin(t));
  };
  DRAKE_EXPECT_THROWS_MESSAGE(
      FunctionHandleTrajectory<double>(sin_func, -1, 1, 0, 1), ".*rows >= 0.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      FunctionHandleTrajectory<double>(sin_func, 1, -1, 0, 1), ".*cols >= 0.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      FunctionHandleTrajectory<double>(sin_func, 1, 1, 2, 1),
      ".*start_time <= end_time.*");
}

GTEST_TEST(FunctionHandleTrajectoryTest, CallbackReturnsWrongSize) {
  const auto my_func = [](const double& t) {
    return Eigen::Matrix<double, 2, 1>(std::sin(t), std::cos(t));
  };
  // The constructor throws if the sizes are wrong.
  DRAKE_EXPECT_THROWS_MESSAGE(
      FunctionHandleTrajectory<double>(my_func, 1, 1, 0, 1),
      ".*returned a matrix of size 2x1.*");
}

GTEST_TEST(FunctionHandleTrajectoryTest, CallbackReturnsWrongSize2) {
  const auto my_func = [](const double& t) {
    if (t < 0.5) {
      Eigen::MatrixXd result(1, 1);
      result << std::sin(t);
      return result;
    }
    Eigen::MatrixXd result(2, 1);
    result << std::sin(t), std::cos(t);
    return result;
  };
  // The constructor succeeds because the size is correct at start_time.
  FunctionHandleTrajectory<double> dut(my_func, /* rows */ 1, /*cols */ 1,
                                       /* start_time */ 0, /* end_time */ 1);
  // But the value() method throws because the size is wrong for t >= 0.5.
  DRAKE_EXPECT_THROWS_MESSAGE(dut.value(0.6),
                              ".*returned a matrix of size 2x1.*");
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
