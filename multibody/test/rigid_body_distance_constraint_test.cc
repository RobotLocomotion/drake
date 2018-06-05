#include "drake/multibody/rigid_body_distance_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(RigidBodyDistanceConstraintTest, TestInitialization) {
  int body1 = 1;
  int body2 = 2;
  Vector3<double> point1(0, 1, 2);
  Vector3<double> point2(4, 5, 6);
  double distance = 0.5;

  RigidBodyDistanceConstraint dc(body1, point1, body2, point2, distance);
  EXPECT_EQ(dc.from_body, body1);
  EXPECT_EQ(dc.to_body, body2);
  EXPECT_TRUE(CompareMatrices(dc.from_point, point1,
                              1e-16, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(dc.to_point, point2,
                              1e-16, MatrixCompareType::relative));
  EXPECT_NEAR(dc.distance, distance, 1e-16);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
