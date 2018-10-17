#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/parser_common.h"

namespace drake {
namespace systems {
namespace plants {
namespace {

using drake::multibody::collision::Element;
using Eigen::Isometry3d;
using Eigen::Matrix3d;
using std::make_unique;
using std::move;
using std::unique_ptr;
using drake::multibody::joints::FloatingBaseType;
using drake::multibody::joints::kFixed;
using drake::multibody::joints::kExperimentalMultibodyPlantStyle;
using drake::multibody::joints::kRollPitchYaw;
using drake::multibody::joints::kQuaternion;

class RigidBodyTreeCollisionCliqueTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tree_ = make_unique<RigidBodyTree<double>>();

    RigidBody<double>& world = tree_->world();

    drake::SquareTwistMatrix<double> I =
        drake::SquareTwistMatrix<double>::Zero();
    I.block(3, 3, 3, 3) << Matrix3d::Identity();

    // This element is cloned each time it is added to the tree.
    drake::multibody::collision::Element element(DrakeShapes::Sphere(1.0),
                                                 Eigen::Isometry3d::Identity());

    // This body requires a self-collision clique
    body1_ = tree_->add_rigid_body(make_unique<RigidBody<double>>());
    body1_->set_model_name("robot1");
    body1_->set_name("body1");
    body1_->set_spatial_inertia(I);
    tree_->addCollisionElement(element, *body1_, "default");
    tree_->addCollisionElement(element, *body1_, "default");
    unique_ptr<DrakeJoint> j(
        new QuaternionFloatingJoint("j1", Isometry3d::Identity()));
    body1_->setJoint(move(j));
    body1_->set_parent(&world);

    // These next bodies will *not* require self-collision clique
    body2_ = tree_->add_rigid_body(make_unique<RigidBody<double>>());
    body2_->set_model_name("robot2");
    body2_->set_name("body2");
    body2_->set_spatial_inertia(I);
    tree_->addCollisionElement(element, *body2_, "default");
    j.reset(new QuaternionFloatingJoint("j1", Isometry3d::Identity()));
    body2_->setJoint(move(j));
    body2_->set_parent(&world);

    body3_ = tree_->add_rigid_body(make_unique<RigidBody<double>>());
    body3_->set_model_name("robot3");
    body3_->set_name("body3");
    body3_->set_spatial_inertia(I);
    j.reset(new QuaternionFloatingJoint("j1", Isometry3d::Identity()));
    body3_->setJoint(move(j));
    body3_->set_parent(&world);
  }

 protected:
  std::unique_ptr<RigidBodyTree<double>> tree_;

  // Bodies are owned by the tree. These raw pointers allow post-hoc
  // manipulation.
  RigidBody<double>* body1_{};
  RigidBody<double>* body2_{};
  RigidBody<double>* body3_{};
};

// Utility function to test that all of the collision elements associated with
// the given body have the expected number of collision cliques.
void ExpectCliqueCount(RigidBody<double>* body, int count) {
  for (auto itr = body->collision_elements_begin();
       itr != body->collision_elements_end(); ++itr) {
    EXPECT_EQ((*itr)->get_num_cliques(), count);
  }
}

// Adds a floating joint between the specified parent and child bodies.
void JoinBodies(const FloatingBaseType floating_base_type,
                RigidBody<double>* parent, RigidBody<double>* child) {
  unique_ptr<DrakeJoint> joint{};
  Eigen::Isometry3d transform_to_world;
  std::string joint_name("joint_" + child->get_name());
  switch (floating_base_type) {
    case kFixed: {
      joint.reset(new FixedJoint(joint_name, transform_to_world));
    } break;
    case kRollPitchYaw: {
      joint.reset(
          new RollPitchYawFloatingJoint(joint_name, transform_to_world));
    } break;
    case kQuaternion:
    case kExperimentalMultibodyPlantStyle: {
      joint.reset(new QuaternionFloatingJoint(joint_name, transform_to_world));
    } break;
    default:
      throw std::runtime_error("unknown floating base type");
  }
  child->set_parent(parent);
  child->setJoint(move(joint));
}

// Confirms that only rigid bodies with multiple collision elements get self-
// collision cliques.
TEST_F(RigidBodyTreeCollisionCliqueTest, SelfCollisionClique) {
  // Links all bodies to world.
  JoinBodies(FloatingBaseType::kQuaternion, &(tree_->world()), body1_);
  JoinBodies(FloatingBaseType::kQuaternion, &(tree_->world()), body2_);
  JoinBodies(FloatingBaseType::kQuaternion, &(tree_->world()), body3_);

  tree_->compile();
  ExpectCliqueCount(body1_, 1);
  ExpectCliqueCount(body2_, 0);
}

// Confirms that rigid bodies that cannot collide (i.e.,
// RigidBody::CanCollideWith returns false) form a clique.
TEST_F(RigidBodyTreeCollisionCliqueTest, CantCollideClique) {
  // Links body1_ and body3_ to the world.
  JoinBodies(FloatingBaseType::kQuaternion, &(tree_->world()), body1_);
  JoinBodies(FloatingBaseType::kQuaternion, &(tree_->world()), body3_);
  // Adds a parent relationship and a non-floating joint between body2_
  // and body3_, causing a clique to be assigned to collision elements of
  // body2_ (body3_ has no collision elements).
  JoinBodies(FloatingBaseType::kFixed, body3_, body2_);

  tree_->compile();

  // Confirms that collision elements on body 1 are *still* only self-collision
  // cliques, determined by a single clique.
  ExpectCliqueCount(body1_, 1);
  // Even though body 2 is adjacent to body 3, it does *not* get any cliques
  // because body 3 has no geometry (i.e., a clique isn't necessary).
  ExpectCliqueCount(body2_, 0);
}

}  // namespace
}  // namespace plants
}  // namespace systems
}  // namespace drake
