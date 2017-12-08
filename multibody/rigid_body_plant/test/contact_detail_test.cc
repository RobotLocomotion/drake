#include "drake/multibody/rigid_body_plant/contact_detail.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/rigid_body_plant/contact_force.h"
#include "drake/multibody/rigid_body_plant/point_contact_detail.h"

// Tests the ContactDetail class and its derivative classes.
namespace drake {
namespace systems {
namespace {

using std::make_unique;
using std::unique_ptr;

GTEST_TEST(ContactDetailTests, PointContactDetailClone) {
  Vector3<double> point, normal, force, torque;
  point << 1, 2, 3;
  normal << 1, 0, 0;
  force << 2, 0, 1;
  torque << -1, -2, -3;
  ContactForce<double> contact_force(point, normal, force, torque);
  auto detail1 = make_unique<PointContactDetail<double>>(contact_force);
  auto detail_copy = detail1->Clone();
  auto contact_copy = detail_copy->ComputeContactForce();

  // Confirms values match but objects are different.
  ASSERT_NE(detail1.get(), detail_copy.get());
  ASSERT_EQ(contact_force.get_application_point(),
            contact_copy.get_application_point());
  ASSERT_EQ(contact_force.get_force(), contact_copy.get_force());
  ASSERT_EQ(contact_force.get_torque(), contact_copy.get_torque());
  ASSERT_EQ(contact_force.get_normal(), contact_copy.get_normal());
}

}  // namespace
}  // namespace systems
}  // namespace drake
