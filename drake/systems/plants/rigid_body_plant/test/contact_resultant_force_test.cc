#include "drake/systems/plants/rigid_body_plant/contact_resultant_force_calculator.h"

#include <gtest/gtest.h>

#include "drake/systems/plants/rigid_body_plant/contact_force.h"

namespace drake {
namespace systems {
namespace {

// THe tests to perform
//  1. Single contact force is returned as itself.
//    a. variety w/ and w/o pure torques.
//  2. Multiple planar forces (i.e., application points on plane with normal
//      directions perpendicular to that plane) - normal components only.
//    a. Should produce proper center of pressure (lies on plane), no torque)
//  3. Multiple planar forces - tangent components only.
//    a. ???  What gets returned?
//  4. Multiple planar forces - tangent and normal
//    a. torque in wrench only due to shifting tangential forces.
//  5. Multiple forces, different normal directions, planar application points
//  6. Non-planar application, non-unified normal directions.  Paul's simple
//      example.

GTEST_TEST(ContactResultantForceTest, ContactForceTests) {
  // Case 1. Confirm default constructor zeros out the data.
  ContactForce<double> f0;
  ASSERT_EQ(f0.get_normal_force(), Vector3<double>::Zero());
  ASSERT_EQ(f0.get_tangent_force(), Vector3<double>::Zero());
  ASSERT_EQ(f0.get_pure_torque(), Vector3<double>::Zero());
  ASSERT_EQ(f0.get_application_point(), Vector3<double>::Zero());

  Vector3<double> norm, tan, torque, pos;
  norm << 0, 0, 1;
  tan << 0, 1, 0;
  torque << 1, 0, 0;
  pos << 3, 2, 1;

  // Case 2. No pure torque constructor.
  ContactForce<double> f1(pos, norm, tan);
  ASSERT_EQ(f1.get_normal_force(), norm);
  ASSERT_EQ(f1.get_tangent_force(), tan);
  ASSERT_EQ(f1.get_pure_torque(), Vector3<double>::Zero());
  ASSERT_EQ(f1.get_application_point(), pos);

  // Case 3. Fully-specified constructor.
  ContactForce<double> f2(pos, norm, tan, torque);
  ASSERT_EQ(f2.get_normal_force(), norm);
  ASSERT_EQ(f2.get_tangent_force(), tan);
  ASSERT_EQ(f2.get_pure_torque(), torque);
  ASSERT_EQ(f2.get_application_point(), pos);

  // Case 4. Confirm norm and tangent combined in full force
  ASSERT_EQ(f2.get_force(), norm + tan);
  WrenchVector<double> expected_wrench;
  expected_wrench.template head<3>() = torque;
  expected_wrench.template tail<3>() = norm + tan;
  ASSERT_EQ(f2.get_wrench(), expected_wrench);
}

}  // namespace
}  // namespace systems
}  // namespace drake
