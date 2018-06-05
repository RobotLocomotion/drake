#include "drake/multibody/multibody_tree/multibody_plant/model_instance.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/weld_joint.h"

namespace drake {
namespace multibody {
namespace multibody_plant {
namespace {

GTEST_TEST(ModelInstance, ModelInstanceTest) {
  // Create a tree with enough bodies to make two models, one with a
  // welded base and one free.
  MultibodyTree<double> tree;

  std::vector<BodyIndex> instance1_bodies;
  const RigidBody<double>& body1 =
      tree.AddRigidBody("Body1", SpatialInertia<double>());
  instance1_bodies.push_back(body1.index());
  const RigidBody<double>& body2 =
      tree.AddRigidBody("Body2", SpatialInertia<double>());
  instance1_bodies.push_back(body2.index());
  const RigidBody<double>& body3 =
      tree.AddRigidBody("Body3", SpatialInertia<double>());
  instance1_bodies.push_back(body3.index());

  std::vector<JointActuatorIndex> instance1_actuators;
  tree.AddJoint<WeldJoint>(
      "weld1", tree.world_body(), Eigen::Isometry3d::Identity(),
      body1, Eigen::Isometry3d::Identity(),
      Eigen::Isometry3d::Identity());
  const Joint<double>& body1_body2 =
      tree.AddJoint<PrismaticJoint>(
          "prism1", body1, Eigen::Isometry3d::Identity(),
          body2, Eigen::Isometry3d::Identity(), Eigen::Vector3d(0, 0, 1));
  const JointActuator<double>& act1 =
      tree.AddJointActuator("act1", body1_body2);
  instance1_actuators.push_back(act1.index());

  const Joint<double>& body2_body3 =
      tree.AddJoint<PrismaticJoint>(
          "prism2", body2, Eigen::Isometry3d::Identity(),
          body3, Eigen::Isometry3d::Identity(), Eigen::Vector3d(0, 0, 1));
  const JointActuator<double>& act2 =
      tree.AddJointActuator("act2", body2_body3);
  instance1_actuators.push_back(act2.index());

  ModelInstance<double> instance1(instance1_bodies, instance1_actuators);

  std::vector<BodyIndex> instance2_bodies;
  const RigidBody<double>& body4 =
      tree.AddRigidBody("Body4", SpatialInertia<double>());
  instance2_bodies.push_back(body4.index());
  const RigidBody<double>& body5 =
      tree.AddRigidBody("Body5", SpatialInertia<double>());
  instance2_bodies.push_back(body5.index());

  std::vector<JointActuatorIndex> instance2_actuators;
  const Joint<double>& body4_body5 =
      tree.AddJoint<PrismaticJoint>(
          "prism3", body4, Eigen::Isometry3d::Identity(),
          body5, Eigen::Isometry3d::Identity(), Eigen::Vector3d(0, 0, 1));
  const JointActuator<double>& act3 =
      tree.AddJointActuator("act3", body4_body5);
  instance2_actuators.push_back(act3.index());

  ModelInstance<double> instance2(instance2_bodies, instance2_actuators);

  EXPECT_THROW(instance1.Finalize(tree), std::logic_error);
  tree.Finalize();
  instance1.Finalize(tree);
  EXPECT_EQ(instance1.num_positions(), 2);
  EXPECT_EQ(instance1.num_velocities(), 2);
  EXPECT_EQ(instance1.num_actuated_dofs(), 2);

  instance2.Finalize(tree);
  EXPECT_EQ(instance2.num_positions(), 8);
  EXPECT_EQ(instance2.num_velocities(), 7);
  EXPECT_EQ(instance2.num_actuated_dofs(), 1);

  Eigen::Vector3d act_vector(0, 0, 0);
  instance1.set_actuation_vector(Eigen::Vector2d(1, 2), &act_vector);
  instance2.set_actuation_vector(Vector1d(3), &act_vector);
  EXPECT_TRUE(CompareMatrices(act_vector, Eigen::Vector3d(1, 2, 3)));

  Eigen::VectorXd pos_vector(10);
  pos_vector << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;

  Eigen::VectorXd instance1_pos(2);
  instance1.get_positions_from_array(pos_vector, &instance1_pos);
  EXPECT_TRUE(CompareMatrices(instance1_pos, Eigen::Vector2d(8, 10)));

  Eigen::VectorXd instance2_pos(8);
  instance2.get_positions_from_array(pos_vector, &instance2_pos);
  Eigen::VectorXd instance2_pos_expected(8);
  instance2_pos_expected << 1, 2, 3, 4, 5, 6, 7, 9;
  EXPECT_TRUE(CompareMatrices(instance2_pos, instance2_pos_expected));

  Eigen::VectorXd vel_vector(9);
  vel_vector << 11, 12, 13, 14, 15, 16, 17, 18, 19;

  Eigen::VectorXd instance1_vel(2);
  instance1.get_velocities_from_array(vel_vector, &instance1_vel);
  EXPECT_TRUE(CompareMatrices(instance1_vel, Eigen::Vector2d(17, 19)));

  Eigen::VectorXd instance2_vel(7);
  instance2.get_velocities_from_array(vel_vector, &instance2_vel);
  Eigen::VectorXd instance2_vel_expected(7);
  instance2_vel_expected << 11, 12, 13, 14, 15, 16, 18;
  EXPECT_TRUE(CompareMatrices(instance2_vel, instance2_vel_expected));

  // Test that scalar conversion produces properly shaped results.
  std::unique_ptr<MultibodyTree<AutoDiffXd>> tree_ad =
      tree.CloneToScalar<AutoDiffXd>();

  ModelInstance<AutoDiffXd> instance1_ad(instance1);
  instance1_ad.Finalize(*tree_ad);
  EXPECT_EQ(instance1_ad.num_positions(), 2);
  EXPECT_EQ(instance1_ad.num_velocities(), 2);
  EXPECT_EQ(instance1_ad.num_actuated_dofs(), 2);

  ModelInstance<AutoDiffXd> instance2_ad(instance2);
  instance2_ad.Finalize(*tree_ad);
  EXPECT_EQ(instance2_ad.num_positions(), 8);
  EXPECT_EQ(instance2_ad.num_velocities(), 7);
  EXPECT_EQ(instance2_ad.num_actuated_dofs(), 1);
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
