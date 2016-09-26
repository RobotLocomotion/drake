#include <iostream>

#include <gtest/gtest.h>

#include "drake/systems/plants/RigidBody.h"
#include "drake/systems/plants/joints/FixedJoint.h"
#include <drake/systems/plants/joints/QuaternionFloatingJoint.h>

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

// Confirms that the adjacency of bodies is evaluated correctly.  See
// RigidBody::adjacentTo for details
GTEST_TEST(RigidBodyTest, TestAdjacency) {
  // self adjacency
  std::unique_ptr<RigidBody> rigid_body_ptrA(new RigidBody());
  EXPECT_FALSE(rigid_body_ptrA->adjacentTo(*rigid_body_ptrA));

  // unconnected rigid bodies
  std::unique_ptr<RigidBody> rigid_body_ptrB(new RigidBody());
  EXPECT_FALSE(rigid_body_ptrA->adjacentTo(*rigid_body_ptrB));
  EXPECT_FALSE(rigid_body_ptrB->adjacentTo(*rigid_body_ptrA));

  // rigid bodies with parent relationship but no link
  rigid_body_ptrA->set_parent(rigid_body_ptrB.get());
  EXPECT_TRUE(rigid_body_ptrA->adjacentTo(*rigid_body_ptrB));
  EXPECT_TRUE(rigid_body_ptrB->adjacentTo(*rigid_body_ptrA));

  Eigen::Isometry3d transform_to_world;
  std::unique_ptr<DrakeJoint> joint( new FixedJoint("testJoint",
                                                    transform_to_world));
  rigid_body_ptrA->setJoint(move(joint));

  // rigid body has joint, but no parent relationship
  rigid_body_ptrA->set_parent(nullptr);
  EXPECT_FALSE(rigid_body_ptrA->adjacentTo(*rigid_body_ptrB));
  EXPECT_FALSE(rigid_body_ptrB->adjacentTo(*rigid_body_ptrA));

  // rigid bodies which *are* properly adjacent
  rigid_body_ptrA->set_parent(rigid_body_ptrB.get());
  EXPECT_TRUE(rigid_body_ptrA->adjacentTo(*rigid_body_ptrB));
  EXPECT_TRUE(rigid_body_ptrB->adjacentTo(*rigid_body_ptrA));

  // connected by floating link
  std::unique_ptr<RigidBody> rigid_body_ptrC(new RigidBody());
  std::unique_ptr<RigidBody> rigid_body_ptrD(new RigidBody());
  rigid_body_ptrC->set_parent(rigid_body_ptrD.get());
  std::unique_ptr<DrakeJoint> floating_joint(new QuaternionFloatingJoint(
      "",
      transform_to_world
  ));
  rigid_body_ptrC->setJoint(move(floating_joint));
  EXPECT_FALSE(rigid_body_ptrC->adjacentTo(*rigid_body_ptrD));
  EXPECT_FALSE(rigid_body_ptrD->adjacentTo(*rigid_body_ptrC));
};
}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
