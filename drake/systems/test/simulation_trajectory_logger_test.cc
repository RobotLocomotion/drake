#include "drake/systems/trajectory_logger.h"
#include <memory>
#include "gtest/gtest.h"

namespace drake {
namespace systems {
namespace test {
namespace {

GTEST_TEST(TestTrajectoryLogger, TestTrajectoryLogger) {
  const size_t kTrajDim = 2;
  auto traj_logger =
      std::make_shared<TrajectoryLogger<EigenVector<kTrajDim>::type>>(kTrajDim);

  const size_t kNt = 100;
  Eigen::Matrix<double, 1, kNt> t;
  NullVector<double> traj_logger_x;
  std::vector<EigenVector<kTrajDim>::type<double>> traj_val(kNt);
  for (size_t i = 0; i < kNt; ++i) {
    t(i) = i * 0.01;
    traj_val[i] << i, 2 * i;
    traj_logger->output(t(i), traj_logger_x, traj_val[i]);
  }
  auto traj = traj_logger->getTrajectory();
  EXPECT_EQ(traj.time.size(), kNt);
  EXPECT_EQ(traj.time.size(), traj.val.size());
  EXPECT_EQ(traj_logger->getNumOutputs(), static_cast<size_t>(kTrajDim));
  for (size_t i = 0; i < kNt; ++i) {
    EXPECT_TRUE(std::abs(traj.time[i] - t(i)) < 1E-5);
    EXPECT_TRUE(std::abs(traj.val[i](0) - traj_val[i](0)) <= 1E-5);
    EXPECT_TRUE(std::abs(traj.val[i](1) - traj_val[i](1)) <= 1E-5);
  }
}

}  // namespace
}  // namespace test
}  // namespace systems
}  // namespace drake
