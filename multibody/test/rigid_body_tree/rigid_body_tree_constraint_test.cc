#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace multibody {
namespace test {
namespace rigid_body_tree {
namespace {

using drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld;

GTEST_TEST(RigidBodyTreeConstraintTest, TestAddDistanceConstraint) {
  auto tree = std::make_unique<RigidBodyTree<double>>();

  int body1 = 1;
  int body2 = 2;
  Vector3<double> point1(0, 1, 2);
  Vector3<double> point2(4, 5, 6);
  double distance = 0.5;

  EXPECT_EQ(tree->getNumPositionConstraints(), 0);
  tree->addDistanceConstraint(body1, point1, body2, point2, distance);
  EXPECT_EQ(tree->getNumPositionConstraints(), 1);

  auto dc = tree->distance_constraints[0];

  EXPECT_EQ(dc.from_body, body1);
  EXPECT_EQ(dc.to_body, body2);
  EXPECT_TRUE(CompareMatrices(dc.from_point, point1, 1e-16,
                              MatrixCompareType::relative));
  EXPECT_TRUE(
      CompareMatrices(dc.to_point, point2, 1e-16, MatrixCompareType::relative));
  EXPECT_NEAR(dc.distance, distance, 1e-16);
}

GTEST_TEST(RigidBodyTreeConstraintTest, TestPositionConstraint) {
  RigidBodyTree<double> tree;
  AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/multibody/test/rigid_body_tree/"
                          "FallingBrick.urdf"),
      drake::multibody::joints::kRollPitchYaw, &tree);
  AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/multibody/test/rigid_body_tree/"
                          "FallingBrick.urdf"),
      drake::multibody::joints::kRollPitchYaw, &tree);

  Eigen::Matrix<double, 24, 1> state;
  state << 0, 0.6, 1, 0, 0, 0,  // x_0, rpy_0
      0, -0.6, 1, 0, 0, 0,      // x_1, rpy_1
      1, 1, 1, 0, 0, 0,         // x_dot_0, omega_0
      -1, -1, -1, 0, 0, 0;      // x_dot_1, omega_1

  Vector3<double> point1(1, 0, 0);
  Vector3<double> point2(1, 0, 0);
  tree.addDistanceConstraint(1, point1, 2, point2, 0.9);

  VectorX<double> q = state.topRows(12);
  VectorX<double> v = state.bottomRows(12);
  auto kinematics_cache = tree.doKinematics(q, v);

  auto constraint_violation = tree.positionConstraints(kinematics_cache);
  auto J_q = tree.positionConstraintsJacobian(kinematics_cache, true);
  auto J_v = tree.positionConstraintsJacobian(kinematics_cache, false);
  auto JdotTimesV = tree.positionConstraintsJacDotTimesV(kinematics_cache);

  EXPECT_EQ(constraint_violation.size(), 1);
  EXPECT_NEAR(constraint_violation[0], 0.3, 1e-16);

  Eigen::Matrix<double, 1, 12> J_check;
  J_check << 0, 1, 0, 0, 0, 1, 0, -1, 0, 0, 0, -1;

  EXPECT_EQ(J_q.rows(), 1);
  EXPECT_EQ(J_q.cols(), 12);
  EXPECT_TRUE(
      CompareMatrices(J_q, J_check, 1e-16, MatrixCompareType::relative));

  EXPECT_EQ(J_v.rows(), 1);
  EXPECT_EQ(J_v.cols(), 12);
  EXPECT_TRUE(
      CompareMatrices(J_v, J_check, 1e-16, MatrixCompareType::relative));

  EXPECT_EQ(JdotTimesV.size(), 1);
  EXPECT_NEAR(JdotTimesV[0], 6.66666667, 1e-8);
}

}  // namespace
}  // namespace rigid_body_tree
}  // namespace test
}  // namespace multibody
}  // namespace drake
