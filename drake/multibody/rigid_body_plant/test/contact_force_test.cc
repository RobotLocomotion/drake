#include "drake/multibody/rigid_body_plant/contact_force.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace systems {
namespace {

// Tests the getter methods in ContactForce.
GTEST_TEST(ContactForceTests, GetterTests) {
  Vector3<double> point(0, 1, 2);
  Vector3<double> normal(0, 0, 1);
  Vector3<double> force(6, 7, 8);
  Vector3<double> torque(9, 10, 11);

  ContactForce<double> contact_force(point, normal, force, torque);

  Vector3<double> normal_force(0, 0, 8);
  Vector3<double> tangent_force(6, 7, 0);

  EXPECT_TRUE(CompareMatrices(
        point, contact_force.get_application_point(),
        1e-20, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
        normal, contact_force.get_normal(),
        1e-20, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
        normal_force, contact_force.get_normal_force(),
        1e-20, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
        tangent_force, contact_force.get_tangent_force(),
        1e-20, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
        torque, contact_force.get_torque(),
        1e-20, MatrixCompareType::absolute));

  ContactForce<double> react_contact_force =
      contact_force.get_reaction_force();

  EXPECT_TRUE(CompareMatrices(
        point, react_contact_force.get_application_point(),
        1e-20, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
        -normal, react_contact_force.get_normal(),
        1e-20, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
        -force, react_contact_force.get_force(),
        1e-20, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
        -torque, react_contact_force.get_torque(),
        1e-20, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace systems
}  // namespace drake
