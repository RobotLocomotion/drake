#include "drake/common/trajectories/discrete_time_trajectory.h"

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace trajectories {
namespace {

GTEST_TEST(DiscreteTimeTrajectoryTest, EigenConstructorTest) {
  Eigen::Vector3d times{0., 0.31, .52};
  Eigen::Matrix3d values;
  values << 2, 5, 67, 5, -4, 1, 34, 16, -23;

  DiscreteTimeTrajectory<double> traj(times, values);
  EXPECT_EQ(traj.num_times(), 3);
  EXPECT_EQ(traj.get_times()[1], times[1]);
  EXPECT_EQ(traj.value(times[1]), values.col(1));
  EXPECT_EQ(traj.rows(), 3);
  EXPECT_EQ(traj.cols(), 1);
  EXPECT_EQ(traj.start_time(), times[0]);
  EXPECT_EQ(traj.end_time(), times[2]);
}

GTEST_TEST(DiscreteTimeTrajectoryTest, StdVectorConstructorTest) {
  std::vector<double> times{0., 0.31, .52};
  std::vector<Eigen::MatrixXd> values(3, Eigen::Vector3d::Zero());
  values[0] << 2, 5, 67;
  values[1] << 5, -4, 1;
  values[2] << 34, 16, -23;

  DiscreteTimeTrajectory<double> traj(times, values);
  EXPECT_EQ(traj.num_times(), 3);
  EXPECT_EQ(traj.get_times()[1], times[1]);
  EXPECT_EQ(traj.value(times[1]), values[1]);
  EXPECT_EQ(traj.rows(), 3);
  EXPECT_EQ(traj.cols(), 1);
  EXPECT_EQ(traj.start_time(), times[0]);
  EXPECT_EQ(traj.end_time(), times[2]);
}

GTEST_TEST(DiscreteTimeTrajectoryTest, ValueThrowsTest) {
  Eigen::Vector3d times{0., 0.31, .52};
  Eigen::Matrix3d values;
  values << 2, 5, 67, 5, -4, 1, 34, 16, -23;

  DiscreteTimeTrajectory<double> traj(times, values);

  DRAKE_EXPECT_THROWS_MESSAGE(traj.value(-1), std::runtime_error,
                              "Value requested at time -1.0 does not match .*");
  DRAKE_EXPECT_THROWS_MESSAGE(traj.value(0.4), std::runtime_error,
                              "Value requested at time 0.4 does not match .*");
  DRAKE_EXPECT_THROWS_MESSAGE(traj.value(1), std::runtime_error,
                              "Value requested at time 1.0 does not match .*");

  const double kLooseTolerance = 0.1;
  DiscreteTimeTrajectory<double> traj_w_loose_tol(times, values,
                                                  kLooseTolerance);
  EXPECT_EQ(traj_w_loose_tol.time_comparison_tolerance(), kLooseTolerance);
  EXPECT_EQ(traj_w_loose_tol.value(0.4), values.col(1));
}

GTEST_TEST(DiscreteTimeTrajectoryTest, CloneTest) {
  Eigen::Vector3d times{0.24, 0.331, .526};
  Eigen::Matrix3d values;
  values << 6, 5, 67, 5, -4, 1, 34, 16, -23;

  const double kTimeComparisonTol = 0.01;
  DiscreteTimeTrajectory<double> traj(times, values, kTimeComparisonTol);
  auto clone = traj.Clone();

  EXPECT_EQ(clone->start_time(), traj.start_time());
  EXPECT_EQ(clone->end_time(), traj.end_time());
  EXPECT_EQ(clone->value(times[1]), values.col(1));
  EXPECT_EQ(clone->value(times[1] + kTimeComparisonTol/2.0), values.col(1));
}

template <typename T>
void ScalarTypeTest() {
  Vector3<T> times{0.24, 0.331, .526};
  Matrix3<T> values;
  values << 6, 5, 67, 5, -4, 1, 34, 16, -23;

  DiscreteTimeTrajectory<T> traj(times, values);
  EXPECT_EQ(traj.value(times[1]), values.col(1));
}

GTEST_TEST(DiscreteTimeTrajectoryTest, ScalarTypeTest) {
  ScalarTypeTest<double>();
  ScalarTypeTest<AutoDiffXd>();
  ScalarTypeTest<symbolic::Expression>();
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
