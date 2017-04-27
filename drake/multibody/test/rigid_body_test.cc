#include "drake/multibody/rigid_body.h"

#include <iostream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/test/rigid_body_compare_to_clone.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

using std::make_unique;

// Tests whether an exception is thrown if RigidBody::getJoint() is called prior
// to a joint being set.
GTEST_TEST(RigidBodyTest, TestGetJointThatIsNotSet) {
  auto rigid_body_ptr = make_unique<RigidBody<double>>();
  EXPECT_THROW(rigid_body_ptr->getJoint(), std::runtime_error);
}

// Confirms that the adjacency of bodies is evaluated correctly.  See
// RigidBody::adjacentTo for details
GTEST_TEST(RigidBodyTest, TestAdjacency) {
  // self adjacency
  auto rigid_body_ptrA = make_unique<RigidBody<double>>();
  EXPECT_FALSE(rigid_body_ptrA->adjacentTo(*rigid_body_ptrA));

  // unconnected rigid bodies
  auto rigid_body_ptrB = make_unique<RigidBody<double>>();
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
  auto rigid_body_ptrC = make_unique<RigidBody<double>>();
  auto rigid_body_ptrD = make_unique<RigidBody<double>>();
  rigid_body_ptrC->set_parent(rigid_body_ptrD.get());
  std::unique_ptr<DrakeJoint> floating_joint(new QuaternionFloatingJoint(
      "",
      transform_to_world));
  rigid_body_ptrC->setJoint(move(floating_joint));
  EXPECT_FALSE(rigid_body_ptrC->adjacentTo(*rigid_body_ptrD));
  EXPECT_FALSE(rigid_body_ptrD->adjacentTo(*rigid_body_ptrC));
}

// Tests ability to clone a RigidBody.
GTEST_TEST(RigidBodyTest, TestClone) {
  const std::string kName = "MyRigidBody";
  const std::string kModelName = "MyModelName";
  const int kModelInstanceId = 738021;
  const int kBodyIndex = 2947;
  const int kPositionStartIndex = 81433;
  const int kVelocityStartIndex = 901261;
  const double kMass = 3.14;
  const Vector3d kCom(3.893, 8.223, 0.33);
  drake::SquareTwistMatrix<double> spatial_inertia =
      drake::SquareTwistMatrix<double>::Zero();
  spatial_inertia.block(3, 3, 3, 3) << kMass * Matrix3d::Identity();

  RigidBody<double> original_body;
  original_body.set_name(kName);
  original_body.set_model_name(kModelName);
  original_body.set_model_instance_id(kModelInstanceId);
  original_body.set_body_index(kBodyIndex);
  original_body.set_position_start_index(kPositionStartIndex);
  original_body.set_velocity_start_index(kVelocityStartIndex);
  original_body.set_mass(kMass);
  original_body.set_center_of_mass(kCom);
  original_body.set_spatial_inertia(spatial_inertia);

  auto cloned_body = original_body.Clone();
  EXPECT_TRUE(multibody::test::rigid_body::CompareToClone(original_body,
      *cloned_body));

  // Ensures that a modified clone does not match.
  cloned_body->set_mass(kMass + 1);
  EXPECT_FALSE(multibody::test::rigid_body::CompareToClone(original_body,
      *cloned_body));
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
