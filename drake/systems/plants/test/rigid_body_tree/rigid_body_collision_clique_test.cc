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

class RigidBodyTreeCollisionCliqueTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tree.reset(new RigidBodyTree());

    drake::SquareTwistMatrix<double> I =
        drake::SquareTwistMatrix<double>::Zero();
    I.block(3, 3, 3, 3) << Matrix3d::Identity();

    // This body requires a self-collision clique
    auto rb_ptr = std::make_unique<RigidBody>();
    rb_ptr->set_model_name("robot1");
    rb_ptr->set_name("body1");
    rb_ptr->set_spatial_inertia(I);
    c1a_ = std::make_unique<Element>();
    c1b_ = std::make_unique<Element>();
    rb_ptr->AddCollisionElement("default", c1a_.get());
    rb_ptr->AddCollisionElement("default", c1b_.get());
    tree->add_rigid_body(std::move(rb_ptr));

    // These next bodies will *not* require self-collision clique
    rb_ptr = std::make_unique<RigidBody>();
    rb_ptr->set_model_name("robot2");
    rb_ptr->set_name("body2");
    rb_ptr->set_spatial_inertia(I);
    c2_ = std::make_unique<Element>();
    rb_ptr->AddCollisionElement("default", c2_.get());
    r1b2_ = rb_ptr.get();
    tree->add_rigid_body(std::move(rb_ptr));

    rb_ptr = std::make_unique<RigidBody>();
    rb_ptr->set_model_name("robot3");
    rb_ptr->set_name("body3");
    rb_ptr->set_spatial_inertia(I);
    r1b3_ = rb_ptr.get();
    tree->add_rigid_body(std::move(rb_ptr));
  }

 protected:
  std::unique_ptr<RigidBodyTree> tree;

  // Bodies are owned by the tree. These raw pointers allow post-hoc
  // manipulation.
  RigidBody* r1b2_{};
  RigidBody* r1b3_{};

  // The collision elements are owned by the test class, the bodes receive
  // raw pointers which they do *not* own.
  std::unique_ptr<Element> c1a_{};
  std::unique_ptr<Element> c1b_{};
  std::unique_ptr<Element> c2_{};
};

// Confirms that only rigid bodies with multiple collision elements get self-
// collision cliques.
TEST_F(RigidBodyTreeCollisionCliqueTest, SelfCollisionClique) {
  tree->compile();
  EXPECT_EQ(c1a_->get_num_cliques(), 1);
  EXPECT_EQ(c1b_->get_num_cliques(), 1);
  EXPECT_EQ(c2_->get_num_cliques(), 0);
}

// Confirms that rigid bodies that cannot collide (i.e.,
// RigidBody::CanCollideWith returns false) form a clique.
TEST_F(RigidBodyTreeCollisionCliqueTest, CantCollideClique) {
  // Adding a parent relationship and a non-floating joint between bodies
  // two and three will cause a clique to be assigned to collision element 2.
  Eigen::Isometry3d transform_to_world;
  std::unique_ptr<DrakeJoint> joint(
      new FixedJoint("testJoint", transform_to_world));
  r1b2_->set_parent(r1b3_);
  r1b2_->setJoint(move(joint));

  tree->compile();

  // Confirm that collision elements on body 1 are *still* only self-collision
  // cliques, determined by a single clique.
  EXPECT_EQ(c1a_->get_num_cliques(), 1);
  EXPECT_EQ(c1b_->get_num_cliques(), 1);
  // Confirm that collision element two has picked up a clique do to its body's
  // relationship with body three.
  EXPECT_EQ(c2_->get_num_cliques(), 1);
}
}  // namespace
}  // namespace plants
}  // namespace systems
}  // namespace drake
