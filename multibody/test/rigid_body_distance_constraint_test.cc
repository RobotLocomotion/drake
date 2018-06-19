#include "drake/multibody/rigid_body_distance_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(RigidBodyDistanceConstraintTest, TestInitialization) {
  const int body1 = 1;
  const int body2 = 2;
  const Vector3<double> point1(0, 1, 2);
  const Vector3<double> point2(4, 5, 6);
  const double distance = 0.5;

  RigidBodyDistanceConstraint dc(body1, point1, body2, point2, distance);
  EXPECT_EQ(dc.bodyA_index, body1);
  EXPECT_EQ(dc.bodyB_index, body2);
  EXPECT_EQ(dc.r_AP, point1);
  EXPECT_EQ(dc.r_BQ, point2);
  EXPECT_EQ(dc.distance, distance);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
