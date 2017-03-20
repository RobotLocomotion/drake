#include "drake/multibody/rigid_body_tree.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/fixed_joint.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Isometry3d;
using std::make_unique;
using std::move;
using std::unique_ptr;

// This tests the functionality responsible for determining that a rigid
// body is "complete" -- all bodies in the tree have a kinematic path
// back to the world.

unique_ptr<RigidBody<double>> MakeBody(int id, RigidBody<double>* parent) {
  auto rb1 = make_unique<RigidBody<double>>();
  rb1->set_model_name("robot");
  rb1->set_name("body" + std::to_string(id));
  unique_ptr<DrakeJoint> j(
      new FixedJoint("j" + std::to_string(id), Isometry3d::Identity()));
  rb1->setJoint(move(j));
  rb1->set_parent(parent);
  return rb1;
}

// Tests the simple case where the tree *is* properly connected.
GTEST_TEST(RigidBodyTreeCompleteness, FullyConnected) {
  RigidBodyTreed tree;

  RigidBody<double>& world = tree.world();

  auto rb1 = MakeBody(1, &world);
  auto rb2 = MakeBody(2, &world);
  auto rb3 = MakeBody(3, rb1.get());

  tree.add_rigid_body(move(rb1));
  tree.add_rigid_body(move(rb2));
  tree.add_rigid_body(move(rb3));

  // Expect no exception to be thrown.
  tree.compile();
}

// Tests the simple case where a body is missing a parent.
GTEST_TEST(RigidBodyTreeCompleteness, MissingParent) {
  RigidBodyTreed tree;

  RigidBody<double>& world = tree.world();

  auto rb1 = MakeBody(1, &world);
  auto rb2 = MakeBody(2, &world);
  auto rb3 = MakeBody(3, rb1.get());
  auto rb4 = MakeBody(4, nullptr);

  tree.add_rigid_body(move(rb1));
  tree.add_rigid_body(move(rb2));
  tree.add_rigid_body(move(rb3));
  tree.add_rigid_body(move(rb4));

  // Using try/catch to test the exception message, confirming that the right
  // information is being returned about the error.
  try {
    tree.compile();
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    std::string msg =
        "ERROR: RigidBodyTree::TestConnectedToWorld(): Rigid body \"body4\" in "
        "model robot is not connected to the world!";
    ASSERT_EQ(e.what(), msg);
  }
}

// Tests the simple case where a body is missing a joint and is caught during
// the "welding" stage.
GTEST_TEST(RigidBodyTreeCompleteness, MissingJointWeldingError) {
  RigidBodyTreed tree;

  RigidBody<double>& world = tree.world();

  auto rb1 = MakeBody(1, &world);
  auto rb2 = MakeBody(2, &world);
  auto rb3 = make_unique<RigidBody<double>>();
  rb3->set_name("body3");
  rb3->set_model_name("robot");
  rb3->set_parent(rb2.get());

  tree.add_rigid_body(move(rb1));
  tree.add_rigid_body(move(rb2));
  tree.add_rigid_body(move(rb3));

  // Using try/catch to test the exception message, confirming that the right
  // information is being returned about the error.
  // NOTE: This error message arises from the the welding work done by the
  // compile method.  The body rb3 has no inertia matrix and no children
  // so it will be "welded" to the world -- the missing joint throws the
  // given exception at that point.
  try {
    tree.compile();
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    std::string msg =
        "ERROR: RigidBody<T>::getJoint(): Rigid body \"body3\" in "
            "model robot does not have a joint!";
    ASSERT_EQ(e.what(), msg);
  }
}

// Tests the simple case where a body is missing a joint and it is caught in
// the setup of configuration space.
GTEST_TEST(RigidBodyTreeCompleteness, MissingJointConfigurationError) {
  RigidBodyTreed tree;

  RigidBody<double>& world = tree.world();

  auto rb1 = MakeBody(1, &world);
  auto rb2 = MakeBody(2, &world);
  auto rb3 = make_unique<RigidBody<double>>();
  rb3->set_name("body3");
  rb3->set_model_name("robot");
  rb3->set_parent(rb2.get());
  rb3->set_spatial_inertia(SquareTwistMatrix<double>::Identity());

  tree.add_rigid_body(move(rb1));
  tree.add_rigid_body(move(rb2));
  tree.add_rigid_body(move(rb3));

  // Using try/catch to test the exception message, confirming that the right
  // information is being returned about the error.
  // NOTE: This error message arises from the process of creating the
  // configuration vector.
  try {
    tree.compile();
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    std::string msg =
        "ERROR: RigidBody<T>::getJoint(): Rigid body \"body3\" in "
            "model robot does not have a joint!";
    ASSERT_EQ(e.what(), msg);
  }
}
}  // namespace
}  // namespace multibody
}  // namespace drake
