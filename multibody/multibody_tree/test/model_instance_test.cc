#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/weld_joint.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(ModelInstance, ModelInstanceTest) {
  // Create a tree with enough bodies to make two models, one with a
  // welded base and one free.
  MultibodyTree<double> tree;

  const ModelInstanceIndex instance1 = tree.AddModelInstance("instance1");

  const RigidBody<double>& body1 =
      tree.AddRigidBody("Body1", instance1, SpatialInertia<double>());
  const RigidBody<double>& body2 =
      tree.AddRigidBody("Body2", instance1, SpatialInertia<double>());
  const RigidBody<double>& body3 =
      tree.AddRigidBody("Body3", instance1, SpatialInertia<double>());

  tree.AddJoint<WeldJoint>(
      "weld1", tree.world_body(), Eigen::Isometry3d::Identity(),
      body1, Eigen::Isometry3d::Identity(),
      Eigen::Isometry3d::Identity());
  const Joint<double>& body1_body2 =
      tree.AddJoint<PrismaticJoint>(
          "prism1", body1, Eigen::Isometry3d::Identity(),
          body2, Eigen::Isometry3d::Identity(), Eigen::Vector3d(0, 0, 1));
  tree.AddJointActuator("act1", body1_body2);

  const Joint<double>& body2_body3 =
      tree.AddJoint<PrismaticJoint>(
          "prism2", body2, Eigen::Isometry3d::Identity(),
          body3, Eigen::Isometry3d::Identity(), Eigen::Vector3d(0, 0, 1));
  tree.AddJointActuator("act2", body2_body3);

  const ModelInstanceIndex instance2 = tree.AddModelInstance("instance2");

  const RigidBody<double>& body4 =
      tree.AddRigidBody("Body4", instance2, SpatialInertia<double>());
  const RigidBody<double>& body5 =
      tree.AddRigidBody("Body5", instance2, SpatialInertia<double>());

  const Joint<double>& body4_body5 =
      tree.AddJoint<PrismaticJoint>(
          "prism3", body4, Eigen::Isometry3d::Identity(),
          body5, Eigen::Isometry3d::Identity(), Eigen::Vector3d(0, 0, 1));
  tree.AddJointActuator("act3", body4_body5);

  tree.Finalize();

  EXPECT_EQ(tree.num_positions(instance1), 2);
  EXPECT_EQ(tree.num_velocities(instance1), 2);
  EXPECT_EQ(tree.num_actuated_dofs(instance1), 2);
  EXPECT_EQ(tree.num_positions(instance2), 8);
  EXPECT_EQ(tree.num_velocities(instance2), 7);
  EXPECT_EQ(tree.num_actuated_dofs(instance2), 1);

  Eigen::Vector3d act_vector(0, 0, 0);
  tree.set_actuation_vector(instance1, Eigen::Vector2d(1, 2), &act_vector);
  tree.set_actuation_vector(instance2, Vector1d(3), &act_vector);
  EXPECT_TRUE(CompareMatrices(act_vector, Eigen::Vector3d(1, 2, 3)));

  Eigen::VectorXd pos_vector(10);
  pos_vector << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;

  Eigen::VectorXd instance1_pos =
      tree.get_positions_from_array(instance1, pos_vector);
  EXPECT_TRUE(CompareMatrices(instance1_pos, Eigen::Vector2d(8, 10)));

  Eigen::VectorXd instance2_pos =
      tree.get_positions_from_array(instance2, pos_vector);

  Eigen::VectorXd instance2_pos_expected(8);
  instance2_pos_expected << 1, 2, 3, 4, 5, 6, 7, 9;
  EXPECT_TRUE(CompareMatrices(instance2_pos, instance2_pos_expected));

  Eigen::VectorXd vel_vector(9);
  vel_vector << 11, 12, 13, 14, 15, 16, 17, 18, 19;

  Eigen::VectorXd instance1_vel =
      tree.get_velocities_from_array(instance1, vel_vector);
  EXPECT_TRUE(CompareMatrices(instance1_vel, Eigen::Vector2d(17, 19)));

  Eigen::VectorXd instance2_vel =
      tree.get_velocities_from_array(instance2, vel_vector);
  Eigen::VectorXd instance2_vel_expected(7);
  instance2_vel_expected << 11, 12, 13, 14, 15, 16, 18;
  EXPECT_TRUE(CompareMatrices(instance2_vel, instance2_vel_expected));

  // Test that scalar conversion produces properly shaped results.
  std::unique_ptr<MultibodyTree<AutoDiffXd>> tree_ad =
      tree.CloneToScalar<AutoDiffXd>();

  EXPECT_EQ(tree_ad->num_positions(instance1), 2);
  EXPECT_EQ(tree_ad->num_velocities(instance1), 2);
  EXPECT_EQ(tree_ad->num_actuated_dofs(instance1), 2);
  EXPECT_EQ(tree_ad->num_positions(instance2), 8);
  EXPECT_EQ(tree_ad->num_velocities(instance2), 7);
  EXPECT_EQ(tree_ad->num_actuated_dofs(instance2), 1);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
