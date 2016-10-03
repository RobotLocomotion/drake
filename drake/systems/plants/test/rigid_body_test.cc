#include <iostream>

#include <gtest/gtest.h>

#include "drake/systems/plants/RigidBody.h"

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

// Tests whether an exception is thrown if RigidBody::getJoint() is called prior
// to a joint being set.
GTEST_TEST(RigidBodyTest, TestGetJointThatIsNotSet) {
  std::unique_ptr<RigidBody> rigid_body_ptr(new RigidBody());
  EXPECT_THROW(rigid_body_ptr->getJoint(), std::runtime_error);
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
