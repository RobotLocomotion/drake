#include "drake/common/trajectories/discrete_time_trajectory.h"

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace trajectories {
namespace {

GTEST_TEST(DiscreteTimeTrajectoryTest, EigenConstructorTest) {
  const Eigen::Vector3d times{0., 0.31, .52};
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
  const std::vector<double> times{0., 0.31, .52};
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
  const Eigen::Vector3d times{0., 0.31, .52};
  Eigen::Matrix3d values;
  values << 2, 5, 67, 5, -4, 1, 34, 16, -23;

  DiscreteTimeTrajectory<double> traj(times, values);

  DRAKE_EXPECT_THROWS_MESSAGE(
      traj.value(-1),
      "Value requested at time -1(\\.0)? does not match .*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      traj.value(0.4),
      "Value requested at time 0.4 does not match .*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      traj.value(1),
      "Value requested at time 1(\\.0)? does not match .*");

  const double kLooseTolerance = 0.1;
  DiscreteTimeTrajectory<double> traj_w_loose_tol(times, values,
                                                  kLooseTolerance);
  EXPECT_EQ(traj_w_loose_tol.time_comparison_tolerance(), kLooseTolerance);
  EXPECT_EQ(traj_w_loose_tol.value(0.4), values.col(1));
}

GTEST_TEST(DiscreteTimeTrajectoryTest, CloneTest) {
  const Eigen::Vector3d times{0.24, 0.331, .526};
  Eigen::Matrix3d values;
  values << 6, 5, 67, 5, -4, 1, 34, 16, -23;

  const double kTimeComparisonTol = 0.01;
  DiscreteTimeTrajectory<double> traj(times, values, kTimeComparisonTol);
  auto clone = traj.Clone();

  EXPECT_EQ(clone->start_time(), traj.start_time());
  EXPECT_EQ(clone->end_time(), traj.end_time());
  EXPECT_EQ(clone->value(times[1]), values.col(1));
  EXPECT_EQ(clone->value(times[1] + kTimeComparisonTol / 2.0), values.col(1));
}

template <typename T>
void ScalarTypeTest() {
  const Vector3<T> times{0.24, 0.331, .526};
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

// Confirm that Expression fails fast if times are not constants.
GTEST_TEST(DicreteTimeTrajectoryTest, SymbolicTimes) {
  using symbolic::Expression;
  symbolic::Variable t{"t"};
  Vector3<Expression> times{0.24, t, .526};
  Matrix3<Expression> values;
  values << 6, 5, 67, 5, -4, 1, 34, 16, -23;

  DRAKE_EXPECT_THROWS_MESSAGE(
      DiscreteTimeTrajectory<Expression>(times, values),
      ".*environment does not have an entry for the variable t.*\n*");

  times[1] = 0.4;
  DiscreteTimeTrajectory<Expression> traj(times, values);
  DRAKE_EXPECT_THROWS_MESSAGE(
      traj.value(t),
      ".*environment does not have an entry for the variable t.*\n*");
}

// Confirm that autodiff gradients on the time parameters have no impact on the
// trajectory values.
GTEST_TEST(DiscreteTimeTrajectoryTest, AutodiffTimes) {
  Vector3<AutoDiffXd> times{0.24, 0.34, .526};
  Matrix3<AutoDiffXd> values;
  values << 6, 5, 67, 5, -4, 1, 34, 16, -23;

  // Case 1: No gradients on the trajectory member variables, but gradients on
  // the sampled time.
  DiscreteTimeTrajectory<AutoDiffXd> traj(times, values);
  AutoDiffXd t{times[1]};
  t.derivatives() = Eigen::Vector2d(-2, 3);
  Vector3<AutoDiffXd> y = traj.value(t);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(y[i].derivatives().size(), 0);
  }

  // Case 2: Gradients on the time member variables.
  times[1].derivatives() = Eigen::Vector2d(24, 1.2);
  DiscreteTimeTrajectory<AutoDiffXd> traj2(times, values);
  y = traj2.value(t);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(y[i].derivatives().size(), 0);
  }

  // Case 3: Gradients on the value member variables DO persist.
  values(2, 0).derivatives() = Eigen::Vector3d(3, 4, 6);
  DiscreteTimeTrajectory<AutoDiffXd> traj3(times, values);
  y = traj3.value(times[0]);
  EXPECT_EQ(y[2].derivatives(), Eigen::Vector3d(3, 4, 6));
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
