#include "drake/common/trajectories/composite_trajectory.h"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/trajectories/bezier_curve.h"

namespace drake {
namespace trajectories {
namespace {

GTEST_TEST(CompositeTrajectoryTest, Basic) {
  Eigen::Matrix2d points;
  // clang-format off
  points << 1, 2,
            3, 7;
  // clang-format on

  std::vector<copyable_unique_ptr<Trajectory<double>>> segments(2);
  segments[0] = std::make_unique<BezierCurve<double>>(2, 3, points);

  points.col(0) = points.col(1);
  points.col(1) << 3, 10;
  segments[1] = std::make_unique<BezierCurve<double>>(3, 4, points);

  CompositeTrajectory<double> traj(segments);

  EXPECT_EQ(traj.start_time(), 2.0);
  EXPECT_EQ(traj.end_time(), 4.0);
  EXPECT_EQ(traj.get_number_of_segments(), 2);
  EXPECT_EQ(traj.segment(0).start_time(), 2.0);
  EXPECT_EQ(traj.segment(1).start_time(), 3.0);
  EXPECT_EQ(traj.segment(1).end_time(), 4.0);
  EXPECT_TRUE(CompareMatrices(traj.value(2.5), Eigen::Vector2d(1.5, 5), 1e-14));
  EXPECT_TRUE(
      CompareMatrices(traj.value(3.5), Eigen::Vector2d(2.5, 8.5), 1e-14));

  auto deriv = traj.MakeDerivative();
  EXPECT_EQ(deriv->start_time(), 2.0);
  EXPECT_EQ(deriv->end_time(), 4.0);
  EXPECT_TRUE(CompareMatrices(deriv->value(2.5), Eigen::Vector2d(1, 4), 1e-14));
  EXPECT_TRUE(CompareMatrices(deriv->value(3.5), Eigen::Vector2d(1, 3), 1e-14));

  EXPECT_TRUE(
      CompareMatrices(traj.EvalDerivative(2.5), Eigen::Vector2d(1, 4), 1e-14));
  EXPECT_TRUE(
      CompareMatrices(traj.EvalDerivative(3.5), Eigen::Vector2d(1, 3), 1e-14));

  auto clone = traj.Clone();
  EXPECT_EQ(clone->start_time(), 2.0);
  EXPECT_EQ(clone->end_time(), 4.0);
  EXPECT_TRUE(
      CompareMatrices(clone->value(2.5), Eigen::Vector2d(1.5, 5), 1e-14));
  EXPECT_TRUE(
      CompareMatrices(clone->value(3.5), Eigen::Vector2d(2.5, 8.5), 1e-14));
}

template <typename T>
void CheckScalarType() {
  Eigen::Matrix<T, 2, 3> points;
  // clang-format off
  points << 1, 0, 0,
            0, 0, 1;
  // clang-format on
  std::vector<copyable_unique_ptr<Trajectory<T>>> segments(1);
  segments[0] = std::make_unique<BezierCurve<T>>(0, 1, points);

  CompositeTrajectory<T> traj(segments);
  EXPECT_TRUE(
      CompareMatrices(traj.value(0.5), Eigen::Vector2d(0.25, 0.25), 1e-14));
}

GTEST_TEST(CompositeTrajectoryTest, ScalarTypes) {
  CheckScalarType<double>();
  CheckScalarType<AutoDiffXd>();
  CheckScalarType<symbolic::Expression>();
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
