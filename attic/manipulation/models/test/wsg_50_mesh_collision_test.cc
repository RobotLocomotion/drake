#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace {

bool ends_with(const std::string& s, const std::string& suffix) {
  return s.rfind(suffix) == (s.size() - suffix.size());
}

// Tests that wsg_50_mesh_collision.urdf can be loaded into a RigidBodyTree.
// This unit test also verifies that the URDF can be parsed by the URDF parser.
GTEST_TEST(Wsg50DescriptionTest, TestMeshCollisionModelLoadTree) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/wsg_50_description/urdf/"
      "wsg_50_mesh_collision.urdf"));

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
     kPath, multibody::joints::kFixed, tree.get());

  const std::vector<std::string> expected_body_names {
      "world",
      "base_link",
      "gripper_left",
      "finger_left",
      "gripper_right",
      "finger_right"};

  EXPECT_EQ(tree->get_num_bodies(), 6);

  for (int i = 0; i < tree->get_num_bodies(); ++i) {
    EXPECT_TRUE(ends_with(tree->get_body(i).get_name(),
                          expected_body_names.at(i)));
  }

  EXPECT_EQ(tree->get_num_model_instances(), 1);
  EXPECT_EQ(tree->get_num_positions(), 2);
  EXPECT_EQ(tree->get_num_velocities(), 2);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
