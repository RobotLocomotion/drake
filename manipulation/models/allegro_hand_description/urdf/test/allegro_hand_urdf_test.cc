#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace {

void test_hand_model(const RigidBodyTreed& tree) {
  EXPECT_EQ(tree.get_num_model_instances(), 1);
  EXPECT_EQ(tree.get_num_positions(), 16);
  EXPECT_EQ(tree.get_num_velocities(), 16);
  EXPECT_EQ(tree.get_num_actuators(), 16);
  EXPECT_EQ(tree.get_num_bodies(), 23);
}

// This unit test also verifies that the URDF can be parsed by the URDF parser.
// TODO(@wenzhenyuan-tri): Migrate to MultibodyPlant once URDF parsing is
// supported.
GTEST_TEST(AllegroHandTest, TestTree) {
  const std::string kPathRight(FindResourceOrThrow(
      "drake/manipulation/models/allegro_hand_description/urdf/"
      "allegro_hand_description_right.urdf"));
  const std::string kPathLeft(FindResourceOrThrow(
      "drake/manipulation/models/allegro_hand_description/urdf/"
      "allegro_hand_description_left.urdf"));

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
     kPathRight, multibody::joints::kFixed, tree.get());
  test_hand_model(*tree.get());
  tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
     kPathLeft, multibody::joints::kFixed, tree.get());
  test_hand_model(*tree.get());
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
