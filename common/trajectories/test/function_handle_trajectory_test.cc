#include "drake/common/trajectories/function_handle_trajectory.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace trajectories {
namespace {

GTEST_TEST(FunctionHandleTrajectoryTest, BasicTest) {
  const auto sin_func = [](const double& t) {
    return Eigen::Matrix<double, 1, 1>(std::sin(t));
  };
  FunctionHandleTrajectory<double> sin_traj(sin_func, 1, 1, 0, 1);

  EXPECT_EQ(sin_traj.rows(), 1);
  EXPECT_EQ(sin_traj.cols(), 1);
  EXPECT_EQ(sin_traj.start_time(), 0);
  EXPECT_EQ(sin_traj.end_time(), 1);

  for (double t = 0; t < 1; t += 0.1) {
    EXPECT_EQ(CompareMatrices(sin_traj.value(t), Eigen::Matrix<double, 1, 1>(std::sin(t))));
  }
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
