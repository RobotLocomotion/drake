#include "drake/multibody/plant/contact_permutation.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using Eigen::Vector3d;

namespace drake {
namespace multibody {

// Builds a complex enough model leading to a forest topology with four trees.
// The model consists of two Kuka arms mounted on top of a robot table. A second
// table is used for objects. The model also includes two free floating mug
// models.
// A few things to notice about this model:
// 1. We add each component of the model in a rather arbitrary, probably not
//    common order. That is, we add a robot then a mug, followed by a robot and
//    yet another mug. Instead of adding all robots first followed by objects.
//    We do this intentionally to "mix" the dofs and make the unit test more
//    interesting.
// 2. The two robots are directly welded to the world rather than to the table,
//    as it'd be in real life. Though equivalent, this leads to two tables
//    anchored to the world. We want to test these do not show up in our
//    permutation since they are not even considered part of a "tree".
void AddScenarioWithTwoArms(MultibodyPlant<double>* plant) {
  DRAKE_DEMAND(!plant->is_finalized());

  const std::string iiwa_sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/"
      "iiwa14_no_collision.sdf");

  const std::string table_sdf_path = FindResourceOrThrow(
      "drake/examples/kuka_iiwa_arm/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf");

  const std::string mug_sdf_path =
      FindResourceOrThrow("drake/examples/simple_gripper/simple_mug.sdf");

  // Load a model of a table for the robot.
  Parser parser(&*plant);
  const ModelInstanceIndex robot_table_model =
      parser.AddModelFromFile(table_sdf_path, "robot_table");
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("link", robot_table_model));

  // Load the robot and weld it on top of the robot table.
  const ModelInstanceIndex arm1_model =
      parser.AddModelFromFile(iiwa_sdf_path, "robot1");

  // Add a floating mug.
  parser.AddModelFromFile(mug_sdf_path, "mug1");

  const ModelInstanceIndex arm2_model =
      parser.AddModelFromFile(iiwa_sdf_path, "robot2");

  // Add a second floating mug.
  parser.AddModelFromFile(mug_sdf_path, "mug2");

  // Though only the topology is important for this test, we place the robot
  // somewhere reasonble on top of the robot table.
  const double table_top_z_in_world =
      // table's top height
      0.736 +
      // table's top width
      0.057 / 2;
  const RigidTransformd X_WRobot1Link0(
      Vector3d(0.0, 0.0, table_top_z_in_world));
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("iiwa_link_0", arm1_model),
                    X_WRobot1Link0);

  // Second arm.
  const RigidTransformd X_WRobot2Link0(
      Vector3d(0.5, 0.0, table_top_z_in_world));
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("iiwa_link_0", arm2_model),
                    X_WRobot2Link0);

  // Load a second table for objects.
  const ModelInstanceIndex objects_table_model =
      parser.AddModelFromFile(table_sdf_path, "objects_table");
  const RigidTransformd X_WT(Vector3d(0.8, 0.0, 0.0));
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("link", objects_table_model), X_WT);

  plant->Finalize();
}

int VerifyModelInstanceIsATreeAndReturnTreeIndex(
    const MultibodyPlant<double>& plant, const std::string& model_instance_name,
    const std::vector<int>& body_to_tree_map) {
  const std::vector<BodyIndex> model_bodies =
      plant.GetBodyIndices(plant.GetModelInstanceByName(model_instance_name));
  DRAKE_DEMAND(model_bodies.size() > 0);
  // For the purpose of these tests, verify the entire model instance belongs to
  // the same tree.
  const int t = body_to_tree_map[model_bodies[0]];  // tree for the first body.
  for (const BodyIndex& body_index : model_bodies) {
    EXPECT_EQ(body_to_tree_map[body_index], t);
  }
  return t;
}

void VerifyDofsWhenModelInstanceIsATree(
    const MultibodyPlant<double>& plant, const std::string& model_instance_name,
    const std::vector<std::string>& joint_names,
    const std::vector<int>& tree_permutation, bool is_floating = false) {
  const ModelInstanceIndex model_instance =
      plant.GetModelInstanceByName(model_instance_name);
  const int nt = tree_permutation.size();
  if (is_floating) {
    // We expect 1 dof per joint + the floating base.
    EXPECT_EQ(tree_permutation.size(), joint_names.size() + 6);

    const auto& base_body = plant.GetUniqueFreeBaseBodyOrThrow(model_instance);
    const int nq = plant.num_positions();
    for (int i = 0; i < 6; ++i) {
      // Dof index for a vector of velocities, therefore we subtract nq.
      const int v = base_body.floating_velocities_start() + i - nq;
      // N.B. Since we use a "reverse DFS order", the six dofs of a floating
      // base will always be at the end of a tree's dofs.
      EXPECT_EQ(tree_permutation[nt - 6 + i], v);
    }

  } else {
    // In these tests we expect one dof per joint.
    EXPECT_EQ(tree_permutation.size(), joint_names.size());
  }

  for (size_t i = 0; i < joint_names.size(); ++i) {
    const auto& joint = plant.GetJointByName(joint_names[i], model_instance);
    // Here we are implicitly assuming 1 dof per joint. Verify this.
    ASSERT_EQ(joint.num_velocities(), 1);
    // Verify dof.
    EXPECT_EQ(tree_permutation[i], joint.velocity_start());
  }
}

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;
  static const internal::MultibodyTreeTopology& get_topology(
      const MultibodyPlant<double>& plant) {
    DRAKE_DEMAND(plant.is_finalized());
    return plant.internal_tree().get_topology();
  }
};

namespace {

GTEST_TEST(DofsPermutation, VelocitiesPermutation) {
  MultibodyPlant<double> plant(0.0);
  AddScenarioWithTwoArms(&plant);
  // Two 7 dofs arms + two 6 dofs free bodies.
  EXPECT_EQ(plant.num_velocities(), 26);

  std::vector<std::vector<int>> velocity_permutation;
  std::vector<int> body_to_tree_map;

  const internal::MultibodyTreeTopology& topology =
      MultibodyPlantTester::get_topology(plant);
  drake::multibody::internal::ComputeDfsPermutation(
      topology, &velocity_permutation, &body_to_tree_map);
  // We expect four trees in the forest.
  EXPECT_EQ(velocity_permutation.size(), 4);

  // The permutation is a bijection. Verify the number of permuted dofs matches
  // the total number of velocities in the model.
  const int num_permuted_dofs =
      std::accumulate(velocity_permutation.begin(), velocity_permutation.end(),
                      0, [](int current, const std::vector<int>& tperm) {
                        return current + tperm.size();
                      });
  EXPECT_EQ(num_permuted_dofs, plant.num_velocities());

  // The world body does not belong to any tree.
  EXPECT_LT(body_to_tree_map[0], 0);

  // We form a vector of joint names in the order we expect them to be in
  // (reversed) DFS order.
  std::vector<std::string> joint_names = {
      "iiwa_joint_7", "iiwa_joint_6", "iiwa_joint_5", "iiwa_joint_4",
      "iiwa_joint_3", "iiwa_joint_2", "iiwa_joint_1"};

  // Verify permutation for robot1.
  const int robot1_tree = VerifyModelInstanceIsATreeAndReturnTreeIndex(
      plant, "robot1", body_to_tree_map);
  ASSERT_GE(robot1_tree, 0);  // non-zero dofs tree.
  VerifyDofsWhenModelInstanceIsATree(plant, "robot1", joint_names,
                                     velocity_permutation[robot1_tree]);

  // Verify permutation for robot2.
  const int robot2_tree = VerifyModelInstanceIsATreeAndReturnTreeIndex(
      plant, "robot2", body_to_tree_map);
  ASSERT_GE(robot2_tree, 0);  // non-zero dofs tree.
  VerifyDofsWhenModelInstanceIsATree(plant, "robot2", joint_names,
                                     velocity_permutation[robot2_tree]);

  // We expect anchored models to have a negative tree index.
  const int robot_table = VerifyModelInstanceIsATreeAndReturnTreeIndex(
      plant, "robot_table", body_to_tree_map);
  EXPECT_LT(robot_table, 0);  // zero dofs tree.
  const int objects_table = VerifyModelInstanceIsATreeAndReturnTreeIndex(
      plant, "objects_table", body_to_tree_map);
  EXPECT_LT(objects_table, 0);  // zero dofs tree.

  // Verify permutation for mug1.
  const int mug1_tree = VerifyModelInstanceIsATreeAndReturnTreeIndex(
      plant, "mug1", body_to_tree_map);
  ASSERT_GE(mug1_tree, 0);  // non-zero dofs tree.
  VerifyDofsWhenModelInstanceIsATree(plant, "mug1", {},
                                     velocity_permutation[mug1_tree], true);

  // Verify permutation for mug2.
  const int mug2_tree = VerifyModelInstanceIsATreeAndReturnTreeIndex(
      plant, "mug2", body_to_tree_map);
  ASSERT_GE(mug2_tree, 0);  // non-zero dofs tree.
  VerifyDofsWhenModelInstanceIsATree(plant, "mug2", {},
                                     velocity_permutation[mug2_tree], true);
}

GTEST_TEST(DofsPermutation, AllegroHands) {
  const std::string right_hand_model_path = FindResourceOrThrow(
      "drake/manipulation/models/"
      "allegro_hand_description/sdf/allegro_hand_description_right.sdf");

  const std::string left_hand_model_path = FindResourceOrThrow(
      "drake/manipulation/models/"
      "allegro_hand_description/sdf/allegro_hand_description_left.sdf");

  MultibodyPlant<double> plant(0);
  Parser parser(&plant);
  parser.AddModelFromFile(right_hand_model_path, "right hand");
  const ModelInstanceIndex left_hand =
      parser.AddModelFromFile(left_hand_model_path, "left hand");

  // We weld the left hand to the world while the right hand is free.
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("hand_root", left_hand));
  plant.Finalize();

  // Compute BFS to DFS permutation.
  std::vector<std::vector<int>> velocity_permutation;
  std::vector<int> body_to_tree_map;
  const internal::MultibodyTreeTopology& topology =
      MultibodyPlantTester::get_topology(plant);
  drake::multibody::internal::ComputeDfsPermutation(
      topology, &velocity_permutation, &body_to_tree_map);

  // We expect two trees in the forest, one for each hand.
  EXPECT_EQ(velocity_permutation.size(), 2);

  // The permutation is bijective. Verify the number of permuted dofs matches
  // the total number of velocities in the model.
  const int num_permuted_dofs =
      velocity_permutation[0].size() + velocity_permutation[1].size();
  EXPECT_EQ(num_permuted_dofs, plant.num_velocities());

  // The world body does not belong to any tree.
  EXPECT_LT(body_to_tree_map[0], 0);

  // N.B. The order in which specific fingers show in the model depends on how
  // the model was constructed. This ultimately has to do with the specific
  // order given in the original SDF file. Therefore the ordering of the names
  // in this test must be kept in sync with the original SDF file model.
  std::vector<std::string> left_hand_joint_names;
  // Ring finger
  left_hand_joint_names.push_back("joint_11");
  left_hand_joint_names.push_back("joint_10");
  left_hand_joint_names.push_back("joint_9");
  left_hand_joint_names.push_back("joint_8");

  // Thumb finger
  left_hand_joint_names.push_back("joint_15");
  left_hand_joint_names.push_back("joint_14");
  left_hand_joint_names.push_back("joint_13");
  left_hand_joint_names.push_back("joint_12");

  // Middle finger
  left_hand_joint_names.push_back("joint_7");
  left_hand_joint_names.push_back("joint_6");
  left_hand_joint_names.push_back("joint_5");
  left_hand_joint_names.push_back("joint_4");

  // Index finger
  left_hand_joint_names.push_back("joint_3");
  left_hand_joint_names.push_back("joint_2");
  left_hand_joint_names.push_back("joint_1");
  left_hand_joint_names.push_back("joint_0");

  std::vector<std::string> right_hand_joint_names;
  // Index finger
  right_hand_joint_names.push_back("joint_3");
  right_hand_joint_names.push_back("joint_2");
  right_hand_joint_names.push_back("joint_1");
  right_hand_joint_names.push_back("joint_0");

  // Thumb finger
  right_hand_joint_names.push_back("joint_15");
  right_hand_joint_names.push_back("joint_14");
  right_hand_joint_names.push_back("joint_13");
  right_hand_joint_names.push_back("joint_12");

  // Middle finger
  right_hand_joint_names.push_back("joint_7");
  right_hand_joint_names.push_back("joint_6");
  right_hand_joint_names.push_back("joint_5");
  right_hand_joint_names.push_back("joint_4");

  // Ring finger
  right_hand_joint_names.push_back("joint_11");
  right_hand_joint_names.push_back("joint_10");
  right_hand_joint_names.push_back("joint_9");
  right_hand_joint_names.push_back("joint_8");

  // Verify permutation for the left hand.
  const int left_hand_tree = VerifyModelInstanceIsATreeAndReturnTreeIndex(
      plant, "left hand", body_to_tree_map);
  VerifyDofsWhenModelInstanceIsATree(plant, "left hand", left_hand_joint_names,
                                     velocity_permutation[left_hand_tree]);

  // Verify permutation for the right hand. The right hand is floating.
  const int right_hand_tree = VerifyModelInstanceIsATreeAndReturnTreeIndex(
      plant, "right hand", body_to_tree_map);
  VerifyDofsWhenModelInstanceIsATree(
      plant, "right hand", right_hand_joint_names,
      velocity_permutation[right_hand_tree], true);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
