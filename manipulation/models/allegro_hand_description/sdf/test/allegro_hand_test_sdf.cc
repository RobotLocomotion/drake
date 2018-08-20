#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace {

void test_hand_model(const RigidBodyTreed& tree) {
  EXPECT_EQ(tree.get_num_model_instances(), 1);
  EXPECT_EQ(tree.get_num_positions(), 16);
  EXPECT_EQ(tree.get_num_velocities(), 16);
  EXPECT_EQ(tree.get_num_actuators(), 16);
  EXPECT_EQ(tree.get_num_bodies(), 18);
}

// This unit test also verifies that the SDF can be parsed by the SDF parser.
GTEST_TEST(AllegroHandTest, TestTree) {
  const std::string kPathRight(FindResourceOrThrow(
      "drake/manipulation/models/allegro_hand_description/sdf/"
      "allegro_hand_description_right.sdf"));
  const std::string kPathLeft(FindResourceOrThrow(
      "drake/manipulation/models/allegro_hand_description/sdf/"
      "allegro_hand_description_left.sdf"));

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::sdf::AddModelInstancesFromSdfFile(
     kPathRight, multibody::joints::kFixed, nullptr, tree.get());
  test_hand_model(*tree.get());
  tree = std::make_unique<RigidBodyTree<double>>();
  parsers::sdf::AddModelInstancesFromSdfFile(
     kPathLeft, multibody::joints::kFixed, nullptr, tree.get());
  test_hand_model(*tree.get());
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
