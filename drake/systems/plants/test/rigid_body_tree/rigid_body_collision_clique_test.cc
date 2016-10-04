#include <iostream>

#include <gtest/gtest.h>

#include <drake/systems/plants/joints/FixedJoint.h>
#include <drake/systems/plants/parser_common.h>
#include <drake/systems/plants/parser_model_instance_id_table.h>
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {
namespace plants {
namespace {

using drake::parsers::ModelInstanceIdTable;
using drake::parsers::AddFloatingJoint;
using drake::systems::plants::joints::kQuaternion;
using DrakeCollision::Element;
using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using std::make_unique;
using std::move;
using std::unique_ptr;

class RigidBodyTreeCollisionCliqueTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tree_ = make_unique<RigidBodyTree>();

    drake::SquareTwistMatrix<double> I =
        drake::SquareTwistMatrix<double>::Zero();
    I.block(3, 3, 3, 3) << Matrix3d::Identity();

    // This body requires a self-collision clique
    RigidBody* temp_body;
    tree_->add_rigid_body(unique_ptr<RigidBody>(temp_body = new RigidBody()));
    temp_body->set_model_name("robot1");
    temp_body->set_name("body1");
    temp_body->set_spatial_inertia(I);
    body1_collision_element_1_ = make_unique<Element>();
    body1_collision_element_2_ = make_unique<Element>();
    temp_body->AddCollisionElement("default", body1_collision_element_1_.get());
    temp_body->AddCollisionElement("default", body1_collision_element_2_.get());

    // These next bodies will *not* require self-collision clique
    tree_->add_rigid_body(unique_ptr<RigidBody>(body2_ = new RigidBody()));
    body2_->set_model_name("robot2");
    body2_->set_name("body2");
    body2_->set_spatial_inertia(I);
    body2_collision_element_ = make_unique<Element>();
    body2_->AddCollisionElement("default", body2_collision_element_.get());

    tree_->add_rigid_body(unique_ptr<RigidBody>(body3_ = new RigidBody()));
    body3_->set_model_name("robot3");
    body3_->set_name("body3");
    body3_->set_spatial_inertia(I);
  }

 protected:
  std::unique_ptr<RigidBodyTree> tree_;

  // Bodies are owned by the tree. These raw pointers allow post-hoc
  // manipulation.
  RigidBody* body2_{};
  RigidBody* body3_{};

  // The collision elements are owned by the test class, the bodies receive
  // raw pointers which they do *not* own.
  std::unique_ptr<Element> body1_collision_element_1_{};
  std::unique_ptr<Element> body1_collision_element_2_{};
  std::unique_ptr<Element> body2_collision_element_{};
};

// Confirms that only rigid bodies with multiple collision elements get self-
// collision cliques.
TEST_F(RigidBodyTreeCollisionCliqueTest, SelfCollisionClique) {
  tree_->compile();
  EXPECT_EQ(body1_collision_element_1_->get_num_cliques(), 1);
  EXPECT_EQ(body1_collision_element_2_->get_num_cliques(), 1);
  EXPECT_EQ(body2_collision_element_->get_num_cliques(), 0);
}

// Confirms that rigid bodies that cannot collide (i.e.,
// RigidBody::CanCollideWith returns false) form a clique.
TEST_F(RigidBodyTreeCollisionCliqueTest, CantCollideClique) {
  // Adding a parent relationship and a non-floating joint between bodies
  // two and three will cause a clique to be assigned to collision element 2.
  Eigen::Isometry3d transform_to_world;
  std::unique_ptr<DrakeJoint> joint(
      new FixedJoint("testJoint", transform_to_world));
  body2_->set_parent(body3_);
  body2_->setJoint(move(joint));

  tree_->compile();

  // Confirm that collision elements on body 1 are *still* only self-collision
  // cliques, determined by a single clique.
  EXPECT_EQ(body1_collision_element_1_->get_num_cliques(), 1);
  EXPECT_EQ(body1_collision_element_2_->get_num_cliques(), 1);
  // Confirm that collision element two has picked up a clique do to its body's
  // relationship with body three.
  EXPECT_EQ(body2_collision_element_->get_num_cliques(), 1);
}
}  // namespace
}  // namespace plants
}  // namespace systems
}  // namespace drake
