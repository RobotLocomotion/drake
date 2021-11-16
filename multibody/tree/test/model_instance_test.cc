#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(ModelInstance, ModelInstanceTest) {
  // Create a tree with enough bodies to make two models, one with a
  // welded base and one free.
  auto tree_pointer = std::make_unique<internal::MultibodyTree<double>>();
  internal::MultibodyTree<double>& tree = *tree_pointer;

  const ModelInstanceIndex instance1 = tree.AddModelInstance("instance1");

  const RigidBody<double>& body1 =
      tree.AddRigidBody("Body1", instance1, SpatialInertia<double>());
  const RigidBody<double>& body2 =
      tree.AddRigidBody("Body2", instance1, SpatialInertia<double>());
  const RigidBody<double>& body3 =
      tree.AddRigidBody("Body3", instance1, SpatialInertia<double>());

  tree.AddJoint<WeldJoint>(
      "weld1", tree.world_body(), math::RigidTransformd::Identity(),
      body1, math::RigidTransformd::Identity(),
      math::RigidTransformd::Identity());
  // Test minimal `AddJoint` overload.
  const Joint<double>& body1_body2 =
      tree.AddJoint(
          std::make_unique<PrismaticJoint<double>>(
              "prism1", body1.body_frame(), body2.body_frame(),
              Eigen::Vector3d(0, 0, 1)));
  tree.AddJointActuator("act1", body1_body2);

  const Joint<double>& body2_body3 =
      tree.AddJoint<PrismaticJoint>(
          "prism2", body2, math::RigidTransformd::Identity(),
          body3, math::RigidTransformd::Identity(), Eigen::Vector3d(0, 0, 1));
  tree.AddJointActuator("act2", body2_body3);

  const ModelInstanceIndex instance2 = tree.AddModelInstance("instance2");

  const RigidBody<double>& body4 =
      tree.AddRigidBody("Body4", instance2, SpatialInertia<double>());
  const RigidBody<double>& body5 =
      tree.AddRigidBody("Body5", instance2, SpatialInertia<double>());

  const Joint<double>& body4_body5 =
      tree.AddJoint<PrismaticJoint>(
          "prism3", body4, math::RigidTransformd::Identity(),
          body5, math::RigidTransformd::Identity(), Eigen::Vector3d(0, 0, 1));
  tree.AddJointActuator("act3", body4_body5);

  tree.Finalize();

  EXPECT_EQ(tree.num_positions(instance1), 2);
  EXPECT_EQ(tree.num_velocities(instance1), 2);
  EXPECT_EQ(tree.num_actuated_dofs(instance1), 2);
  EXPECT_EQ(tree.num_positions(instance2), 8);
  EXPECT_EQ(tree.num_velocities(instance2), 7);
  EXPECT_EQ(tree.num_actuated_dofs(instance2), 1);

  Eigen::Vector3d act_vector(0, 0, 0);
  tree.SetActuationInArray(instance1, Eigen::Vector2d(1, 2), &act_vector);
  tree.SetActuationInArray(instance2, Vector1d(3), &act_vector);
  EXPECT_TRUE(CompareMatrices(tree.GetActuationFromArray(instance1, act_vector),
                              Eigen::Vector2d(1, 2)));
  EXPECT_TRUE(CompareMatrices(tree.GetActuationFromArray(instance2, act_vector),
                              Vector1d(3)));
  EXPECT_TRUE(CompareMatrices(act_vector, Eigen::Vector3d(1, 2, 3)));

  // N.B. These tests assume a specific ordering of the state. Each body in the
  // model has an associated mobility. Mobilities are numbered in the order
  // mobilizers are added. For this simple model MultibodyTree adds mobilizers
  // for each joint in the order joints were added. Free floating mobilizers are
  // added last at Finalize(). This implies that DOFs for the first tree are
  // first, followed by DOFs of the second tree, etc.

  Eigen::VectorXd pos_vector(10);
  pos_vector << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;

  // DOFs for instance1 correspond to those DOFs in the first tree.
  Eigen::VectorXd instance1_pos =
      tree.GetPositionsFromArray(instance1, pos_vector);
  EXPECT_TRUE(CompareMatrices(instance1_pos, Eigen::Vector2d(1, 2)));

  // Change the positions for this instance and check again.
  const Eigen::Vector2d new_pos(21, 23);
  tree.SetPositionsInArray(instance1, new_pos, &pos_vector);
  instance1_pos = tree.GetPositionsFromArray(instance1, pos_vector);
  EXPECT_TRUE(CompareMatrices(instance1_pos, new_pos));

  Eigen::VectorXd instance2_pos =
      tree.GetPositionsFromArray(instance2, pos_vector);

  Eigen::VectorXd instance2_pos_expected(8);
  instance2_pos_expected << 3, 4, 5, 6, 7, 8, 9, 10;
  EXPECT_TRUE(CompareMatrices(instance2_pos, instance2_pos_expected));

  Eigen::VectorXd vel_vector(9);
  vel_vector << 11, 12, 13, 14, 15, 16, 17, 18, 19;

  Eigen::VectorXd instance1_vel =
      tree.GetVelocitiesFromArray(instance1, vel_vector);
  EXPECT_TRUE(CompareMatrices(instance1_vel, Eigen::Vector2d(11, 12)));

  // Change the velocities for this instance and check again.
  const Eigen::Vector2d new_vel(29, 31);
  tree.SetVelocitiesInArray(instance1, new_vel, &vel_vector);
  instance1_vel = tree.GetVelocitiesFromArray(instance1, vel_vector);
  EXPECT_TRUE(CompareMatrices(instance1_vel, new_vel));

  Eigen::VectorXd instance2_vel =
      tree.GetVelocitiesFromArray(instance2, vel_vector);
  Eigen::VectorXd instance2_vel_expected(7);
  instance2_vel_expected << 13, 14, 15, 16, 17, 18, 19;
  EXPECT_TRUE(CompareMatrices(instance2_vel, instance2_vel_expected));

  // Create a MultibodyTreeSystem so that we can get a context.
  internal::MultibodyTreeSystem<double> mb_system(std::move(tree_pointer));
  std::unique_ptr<systems::Context<double>> context = mb_system.
      CreateDefaultContext();

  // Clear the entire multibody state vector so that we can check the effect
  // of setting one instance at a time.
  tree.GetMutablePositionsAndVelocities(context.get()).setZero();
  EXPECT_EQ(tree.get_positions_and_velocities(*context).norm(), 0);

  // Validate setting the position and velocity through the multibody state
  // vector for an instance.
  Eigen::VectorXd instance1_x(tree.num_positions(instance1) +
      tree.num_velocities(instance1));
  instance1_x << instance1_pos, instance1_vel;
  tree.SetPositionsAndVelocities(instance1, instance1_x, context.get());
  EXPECT_EQ(tree.GetPositionsAndVelocities(*context, instance2).norm(), 0);
  const Eigen::VectorXd instance1_pos_from_array = tree.GetPositionsFromArray(
      instance1, pos_vector);
  const Eigen::VectorXd instance1_vel_from_array = tree.GetVelocitiesFromArray(
      instance1, vel_vector);
  EXPECT_TRUE(CompareMatrices(instance1_pos, instance1_pos_from_array));
  EXPECT_TRUE(CompareMatrices(instance1_vel, instance1_vel_from_array));

  // Test that scalar conversion produces properly shaped results.
  std::unique_ptr<internal::MultibodyTree<AutoDiffXd>> tree_ad =
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
