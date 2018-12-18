#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace {

// Tests that dual_iiwa14_polytope_collision.urdf can be loaded into a
// RigidBodyTree. This unit test also verifies that the URDF can be parsed by
// the URDF parser.
GTEST_TEST(DualIiwa14PolytopeCollisionTest, TestLoadTree) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "dual_iiwa14_polytope_collision.urdf"));

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
     kPath, multibody::joints::kFixed, tree.get());

  // Each robot has 8 bodies and an end effector with 2 bodies. In addition,
  // there are two bodies that are not part of either robot: world and base.
  // Since there are two robots, there should be a total of
  // 2 * (8 + 2) + 2 = 22 bodies in the tree.
  EXPECT_EQ(tree->get_num_bodies(), 22);

  // TODO(liang.fok) Once the URDF parser is able to recognize body and joint
  // name prefixes, the following test should expect that there be two model
  // instances. For more details, see #5317.
  EXPECT_EQ(tree->get_num_model_instances(), 1);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
