#include "drake/systems/plants/rigid_body_plant/contact_resultant_force_calculator.h"

#include <gtest/gtest.h>

#include "drake/systems/plants/rigid_body_plant/contact_force.h"

namespace drake {
namespace systems {
namespace  {

GTEST_TEST(ContactResultantForceTest, DummyTest) {
  ContactForce<double> force;
  ASSERT_EQ(force.get_normal_force(), Vector3<double>::Zero());
}

}  // namespace
}  // namespace systems
}  // namespace drake