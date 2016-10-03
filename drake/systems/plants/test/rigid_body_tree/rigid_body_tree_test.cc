#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/common/drake_path.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/joints/QuaternionFloatingJoint.h"
#include "drake/systems/plants/joints/RevoluteJoint.h"
#include "drake/systems/plants/parser_common.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_urdf.h"

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

using drake::parsers::ModelInstanceIdTable;
using drake::parsers::AddFloatingJoint;
using drake::systems::plants::joints::kQuaternion;
using Eigen::Isometry3d;
using Eigen::Vector3d;

class RigidBodyTreeTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    tree.reset(new RigidBodyTree());

    // Defines four rigid bodies.
    r1b1 = new RigidBody();
    r1b1->set_model_name("robot1");
    r1b1->set_name("body1");

    r2b1 = new RigidBody();
    r2b1->set_model_name("robot2");
    r2b1->set_name("body1");

    r3b1 = new RigidBody();
    r3b1->set_model_name("robot3");
    r3b1->set_name("body1");

    r4b1 = new RigidBody();
    r4b1->set_model_name("robot4");
    r4b1->set_name("body1");
  }

 public:
  // TODO(amcastro-tri): A stack object here (preferable to a pointer)
  // generates build issues on Windows platforms. See git-hub issue #1854.
  std::unique_ptr<RigidBodyTree> tree;
  // TODO(amcastro-tri): these pointers will be replaced by Sherm's
  // unique_ptr_reference's.
  RigidBody* r1b1{};
  RigidBody* r2b1{};
  RigidBody* r3b1{};
  RigidBody* r4b1{};
};

TEST_F(RigidBodyTreeTest, TestAddFloatingJointNoOffset) {
  // Adds rigid bodies r1b1 and r2b1 to the rigid body tree and verify they can
  // be found.

  // RigidBodyTree takes ownership of these bodies.
  // User still has access to these bodies through the raw pointers.
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r1b1));
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r2b1));

  EXPECT_TRUE(tree->FindBody("body1", "robot1") != nullptr);
  EXPECT_TRUE(tree->FindBody("body1", "robot2") != nullptr);
  EXPECT_THROW(tree->FindBody("body2", "robot1"), std::logic_error);
  EXPECT_THROW(tree->FindBody("body2", "robot2"), std::logic_error);

  // Adds floating joints that connect r1b1 and r2b1 to the rigid body tree's
  // world at zero offset.
  r1b1->add_joint(&tree->world(), std::make_unique<QuaternionFloatingJoint>(
      "Base", Isometry3d::Identity()));

  r2b1->add_joint(r1b1, std::make_unique<RevoluteJoint>(
      "Joint1", Isometry3d::Identity(), Vector3d::UnitZ()));

  // Verfies that the two rigid bodies are located in the correct place.
  const DrakeJoint& jointR1B1 = tree->FindBody("body1", "robot1")->getJoint();
  EXPECT_TRUE(jointR1B1.isFloating());
  EXPECT_TRUE(jointR1B1.getTransformToParentBody().matrix() ==
              Eigen::Isometry3d::Identity().matrix());

  const DrakeJoint& jointR2B1 = tree->FindBody("body1", "robot2")->getJoint();
  EXPECT_FALSE(jointR2B1.isFloating());
  EXPECT_TRUE(jointR2B1.getTransformToParentBody().matrix() ==
              Eigen::Isometry3d::Identity().matrix());
}

TEST_F(RigidBodyTreeTest, TestAddFloatingJointWithOffset) {
  // TODO(amcastro-tri): these pointers will be replaced by Sherm's
  // unique_ptr_reference's
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r1b1));
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r2b1));

  // Adds floating joints that connect r1b1 and r2b1 to the rigid body tree's
  // world at offset x = 1, y = 1, z = 1.
  Eigen::Isometry3d T_r1and2_to_world;
  {
    Eigen::Vector3d xyz, rpy;
    xyz << 1, 1, 1;
    rpy = Eigen::Vector3d::Zero();
    T_r1and2_to_world.matrix()
        << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "world", nullptr,
      T_r1and2_to_world);

  AddFloatingJoint(kQuaternion,
                   {r1b1->get_body_index(), r2b1->get_body_index()},
                   weld_to_frame, nullptr /* pose_map */, tree.get());

  // Verfies that the two rigid bodies are located in the correct place.
  const DrakeJoint& jointR1B1 = tree->FindBody("body1", "robot1")->getJoint();
  EXPECT_TRUE(jointR1B1.isFloating());
  EXPECT_TRUE(jointR1B1.getTransformToParentBody().matrix() ==
              T_r1and2_to_world.matrix());

  const DrakeJoint& jointR2B1 = tree->FindBody("body1", "robot2")->getJoint();
  EXPECT_TRUE(jointR2B1.isFloating());
  EXPECT_TRUE(jointR2B1.getTransformToParentBody().matrix() ==
              T_r1and2_to_world.matrix());
}

TEST_F(RigidBodyTreeTest, TestAddFloatingJointWeldToLink) {
  // Adds rigid body r1b1 to the rigid body tree and welds it to the world with
  // zero offset. Verifies that it is in the correct place.
  // TODO(amcastro-tri): these pointers will be replaced by Sherm's
  // unique_ptr_reference's
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r1b1));

  AddFloatingJoint(kQuaternion, {r1b1->get_body_index()},
                   nullptr /* weld_to_frame */, nullptr /* pose_map */,
                   tree.get());

  // Adds rigid body r2b1 to the rigid body tree and welds it to r1b1 with
  // offset x = 1, y = 1, z = 1. Verifies that it is in the correct place.
  // TODO(amcastro-tri): these pointers will be replaced by Sherm's
  // unique_ptr_reference's
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r2b1));

  Eigen::Isometry3d T_r2_to_r1;
  {
    Eigen::Vector3d xyz, rpy;
    xyz << 1, 1, 1;
    rpy = Eigen::Vector3d::Zero();
    T_r2_to_r1.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto r2b1_weld = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "body1",
      tree->FindBody("body1", "robot1"), T_r2_to_r1);

  AddFloatingJoint(kQuaternion, {r2b1->get_body_index()},
      r2b1_weld, nullptr /* pose_map */, tree.get());

  // Adds rigid body r3b1 and r4b1 to the rigid body tree and welds it to r2b1
  // with offset x = 2, y = 2, z = 2. Verifies that it is in the correct place.
  // TODO(amcastro-tri): these pointers will be replaced by Sherm's
  // unique_ptr_reference's
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r3b1));
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r4b1));

  Eigen::Isometry3d T_r3_and_r4_to_r2;
  {
    Eigen::Vector3d xyz, rpy;
    xyz << 2, 2, 2;
    rpy = Eigen::Vector3d::Zero();
    T_r3_and_r4_to_r2.matrix()
        << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto r3b1_and_r4b1_weld = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "body1",
      tree->FindBody("body1", "robot2"), T_r3_and_r4_to_r2);

  AddFloatingJoint(kQuaternion,
                   {r3b1->get_body_index(), r4b1->get_body_index()},
                   r3b1_and_r4b1_weld, nullptr /* pose_map */,
                   tree.get());

  EXPECT_TRUE(tree->FindBody("body1", "robot1")
                  ->getJoint()
                  .getTransformToParentBody()
                  .matrix() == Eigen::Isometry3d::Identity().matrix());

  EXPECT_TRUE(tree->FindBody("body1", "robot2")
                  ->getJoint()
                  .getTransformToParentBody()
                  .matrix() == T_r2_to_r1.matrix());

  EXPECT_TRUE(tree->FindBody("body1", "robot3")
                  ->getJoint()
                  .getTransformToParentBody()
                  .matrix() == T_r3_and_r4_to_r2.matrix());

  EXPECT_TRUE(tree->FindBody("body1", "robot4")
                  ->getJoint()
                  .getTransformToParentBody()
                  .matrix() == T_r3_and_r4_to_r2.matrix());
}

// Ensures RigidBodyTree::doKinemantics(q, v, bool) is explicitly instantiated
// with vector block input parameters. For more information, see:
// https://github.com/RobotLocomotion/drake/issues/2634.
TEST_F(RigidBodyTreeTest, TestDoKinematicsWithVectorBlocks) {
  std::string filename =
      drake::GetDrakePath() +
      "/systems/plants/test/rigid_body_tree/two_dof_robot.urdf";
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(filename, tree.get());

  VectorX<double> q;
  VectorX<double> v;
  q.resize(tree->get_num_positions());
  v.resize(tree->get_num_velocities());
  q.setZero();
  v.setZero();

  Eigen::VectorBlock<VectorX<double>> q_block = q.head(q.size());
  Eigen::VectorBlock<VectorX<double>> v_block = v.head(v.size());

  KinematicsCache<double> cache = tree->doKinematics(q_block, v_block);
  EXPECT_TRUE(cache.hasV());
}

// Ensure's the model's instance ID was saved in model_instance_id_table.
// Since only one model was added   (a 2-DOF robot), there should only be one
// model in the table. Furthermore, it should be called "two_dof_robot" and
// the model instance ID should be 1 (zero was assigned to the world model).
TEST_F(RigidBodyTreeTest, TestModelInstanceIdTable) {
  std::string filename =
      drake::GetDrakePath() +
      "/systems/plants/test/rigid_body_tree/two_dof_robot.urdf";

  ModelInstanceIdTable model_instance_id_table =
      drake::parsers::urdf::AddModelInstanceFromUrdfFile(filename, tree.get());

  const int kExpectedTableSize = 1;
  const int kExpectedModelInstanceId = 0;
  EXPECT_EQ(static_cast<int>(model_instance_id_table.size()),
      kExpectedTableSize);
  EXPECT_NE(model_instance_id_table.find("two_dof_robot"),
      model_instance_id_table.end());
  EXPECT_EQ(model_instance_id_table["two_dof_robot"], kExpectedModelInstanceId);
}

// Verifies that each rigid body in @p body_list appears exactly once in
// @p expected_names.
void VerifyBodyListIsCorrect(std::vector<const RigidBody*> body_list,
    std::vector<std::string> expected_names) {
  for (const RigidBody* body : body_list) {
    EXPECT_NE(std::find(expected_names.begin(), expected_names.end(),
        body->get_name()), expected_names.end());
    std::remove_if(expected_names.begin(), expected_names.end(),
        [body](std::string name) {return name == body->get_name();});
  }
}

// Tests the correct functionality of RigidBodyTree::FindModelInstanceBodies().
TEST_F(RigidBodyTreeTest, TestFindModelInstanceBodies) {
  std::string filename_2dof_robot =
      drake::GetDrakePath() +
      "/systems/plants/test/rigid_body_tree/two_dof_robot.urdf";

  std::string filename_3dof_robot =
      drake::GetDrakePath() +
      "/systems/plants/test/rigid_body_tree/three_dof_robot.urdf";

  std::string filename_4dof_robot =
      drake::GetDrakePath() +
      "/systems/plants/test/rigid_body_tree/four_dof_robot.urdf";

  ModelInstanceIdTable model_instance_id_table_1 =
      drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          filename_2dof_robot, tree.get());

  ModelInstanceIdTable model_instance_id_table_2 =
      drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          filename_3dof_robot, tree.get());

  ModelInstanceIdTable model_instance_id_table_3 =
      drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          filename_4dof_robot, tree.get());

  const std::string kTwoDofModelName = "two_dof_robot";
  const std::string kThreeDofModelName = "three_dof_robot";
  const std::string kFourDofModelName = "four_dof_robot";

  // Gets the model instance IDs.
  int two_dof_model_instance_id =
      model_instance_id_table_1.at(kTwoDofModelName);
  int three_dof_model_instance_id =
      model_instance_id_table_2.at(kThreeDofModelName);
  int four_dof_model_instance_id =
      model_instance_id_table_3.at(kFourDofModelName);

  // Gets the rigid bodies belonging to each model instance.
  std::vector<const RigidBody*> two_dof_robot_bodies =
      tree->FindModelInstanceBodies(two_dof_model_instance_id);

  std::vector<const RigidBody*> three_dof_robot_bodies =
      tree->FindModelInstanceBodies(three_dof_model_instance_id);

  std::vector<const RigidBody*> four_dof_robot_bodies =
      tree->FindModelInstanceBodies(four_dof_model_instance_id);

  // Verifies the sizes of the vectors of rigid bodies are correct.
  EXPECT_EQ(two_dof_robot_bodies.size(), 3u);
  EXPECT_EQ(three_dof_robot_bodies.size(), 4u);
  EXPECT_EQ(four_dof_robot_bodies.size(), 5u);

  // Verifies that the model instance IDs and model names are correct.
  for (const RigidBody* body : two_dof_robot_bodies) {
    EXPECT_EQ(body->get_model_instance_id(), two_dof_model_instance_id);
    EXPECT_EQ(body->get_model_name(), kTwoDofModelName);
  }

  for (const RigidBody* body : three_dof_robot_bodies) {
    EXPECT_EQ(body->get_model_instance_id(), three_dof_model_instance_id);
    EXPECT_EQ(body->get_model_name(), kThreeDofModelName);
  }

  for (const RigidBody* body : four_dof_robot_bodies) {
    EXPECT_EQ(body->get_model_instance_id(), four_dof_model_instance_id);
    EXPECT_EQ(body->get_model_name(), kFourDofModelName);
  }

  // Verifies that the names of the RigidBodies fall into the expected range of
  // values.
  VerifyBodyListIsCorrect(two_dof_robot_bodies,
      {"link1", "link2", "link3"});
  VerifyBodyListIsCorrect(three_dof_robot_bodies,
      {"link1", "link2", "link3", "link4"});
  VerifyBodyListIsCorrect(four_dof_robot_bodies,
      {"link1", "link2", "link3", "link4", "link5"});
}

// Verifies the correct functionality of RigidBodyTree::FindChildrenOfBody()
// and RigidBodyTree::FindBaseBodies(). This also tests
// RigidBodyTree::get_body() and RigidBodyTree::get_num_bodies().
TEST_F(RigidBodyTreeTest, TestFindChildrenOfBodyAndFindBaseBodies) {
  // Adds kNumModelInstances instances of a particular URDF model to the tree.
  // Stores the model instance IDs in model_instance_id_list.
  const int kNumModelInstances = 10;

  std::string file_name =
      drake::GetDrakePath() +
      "/systems/plants/test/rigid_body_tree/two_dof_robot.urdf";

  std::vector<int> model_instance_id_list;

  for (int i = 0; i < kNumModelInstances; ++i) {
    ModelInstanceIdTable model_instance_id_table =
      drake::parsers::urdf::AddModelInstanceFromUrdfFile(file_name, tree.get());
    model_instance_id_list.push_back(model_instance_id_table["two_dof_robot"]);
  }

  // Obtains a list of base bodies and verifies that all of the bodies in this
  // list are called "link1".
  std::vector<int> base_body_list = tree->FindBaseBodies();
  for (int index : base_body_list) {
    EXPECT_EQ(tree->get_body(index).get_name(), "link1");
  }

  // Obtains a list of the world's children. Verifies that this list is
  // identical to base_body_list.
  std::vector<int> children_of_world_list =
      tree->FindChildrenOfBody(RigidBodyTree::kWorldBodyIndex);

  EXPECT_EQ(base_body_list.size(), children_of_world_list.size());

  // There are three bodies per model instance plus one body for the world.
  EXPECT_EQ(tree->get_num_bodies(), 3 * kNumModelInstances + 1);

  for (int world_child_index : children_of_world_list) {
    bool found_child_in_base_body_list = false;
    for (int body_index : base_body_list) {
      if (body_index == world_child_index) {
        found_child_in_base_body_list = true;
      }
    }
    EXPECT_TRUE(found_child_in_base_body_list);
  }

  // Obtains a list of base bodies that belong to a particular model instance.
  // Verifies that this list has only one element, which is expected since, in
  // this case, each model instance only has one connection to the world. Also
  // verify that the name of this body is "link1".
  std::vector<int> base_body_specific_id_list = tree->FindBaseBodies(
      model_instance_id_list.at(0));

  EXPECT_EQ(base_body_specific_id_list.size(), 1u);
  EXPECT_EQ(tree->get_body(base_body_specific_id_list.at(0)).get_name(),
      "link1");

  // Obtains the children of the above "link1" body. Verify that there is only
  // one child and it is called "link2".
  std::vector<int> children_of_one_base_body = tree->FindChildrenOfBody(
      base_body_specific_id_list.at(0));

  EXPECT_EQ(children_of_one_base_body.size(), 1u);

  EXPECT_EQ(tree->get_body(children_of_one_base_body.at(0)).get_name(),
      "link2");

  // Verifies that an empty list is returned if a non-matching model instance
  // ID is provided.
  int body_index = base_body_specific_id_list.at(0);
  int non_matching_model_instance_id =
      tree->get_body(body_index).get_model_instance_id() + 1;

  std::vector<int> list_of_children_bad_instance_id =
      tree->FindChildrenOfBody(body_index, non_matching_model_instance_id);

  EXPECT_EQ(list_of_children_bad_instance_id.size(), 0u);
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
