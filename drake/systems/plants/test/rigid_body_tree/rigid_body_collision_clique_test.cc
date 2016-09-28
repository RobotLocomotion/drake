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
  virtual void SetUp() {
    tree.reset(new RigidBodyTree());

    // Defines four rigid bodies.
    r1b1 = new RigidBody();
    r1b1->set_model_name("robot1");
    r1b1->set_name("body1");
    c1a = new Element();
    r1b1->AddCollisionElement("default", c1a);
    c1b = new Element();
    r1b1->AddCollisionElement("default", c1b);
    // assign inertia to avoid the welding code in RigidBodyTree::compile
    drake::SquareTwistMatrix<double> I =
        drake::SquareTwistMatrix<double>::Zero();
    I.block(3, 3, 3, 3) << Matrix3d::Identity();
    r1b1->set_spatial_inertia(I);

    r2b1 = new RigidBody();
    r2b1->set_model_name("robot2");
    r2b1->set_name("body1");
    c2 = new Element();
    r2b1->AddCollisionElement("default", c2);
    r2b1->set_spatial_inertia(I);

    r3b1 = new RigidBody();
    r3b1->set_model_name("robot3");
    r3b1->set_name("body1");
    r3b1->set_spatial_inertia(I);

    tree->add_rigid_body(std::unique_ptr<RigidBody>(r1b1));
    tree->add_rigid_body(std::unique_ptr<RigidBody>(r2b1));
    tree->add_rigid_body(std::unique_ptr<RigidBody>(r3b1));

    // add joints


    tree->compile();
  }

 public:
  std::unique_ptr<RigidBodyTree> tree;

  RigidBody* r1b1{};
  RigidBody* r2b1{};
  RigidBody* r3b1{};

  Element * c1a{};
  Element * c1b{};
  Element * c2{};
};

// Confirms that only rigid bodies with multiple collision elements get self-
// collision cliques.
TEST_F(RigidBodyTreeCollisionCliqueTest, SelfCollisionClique) {
  EXPECT_EQ(c1a->get_num_cliques(), 1);
  EXPECT_EQ(c1b->get_num_cliques(), 1);
  EXPECT_EQ(c2->get_num_cliques(), 0);
}

// Confirms that joints that cannot collide (i.e., RigidBody::CanCollideWith
// returns false) form a clique.
TEST_F(RigidBodyTreeCollisionCliqueTest, CantCollideClique) {
  Eigen::Isometry3d transform_to_world;
  std::unique_ptr<DrakeJoint> joint( new FixedJoint("testJoint",
                                                    transform_to_world));
  r2b1->set_parent(r3b1);
  r2b1->setJoint(move(joint));
  EXPECT_EQ(c1a->get_num_cliques(), 1);
  EXPECT_EQ(c1b->get_num_cliques(), 1);
  EXPECT_EQ(c2->get_num_cliques(), 0);
}
}  // namespace
}  // namespace plants
}  // namespace systems
}  // namespace drake
