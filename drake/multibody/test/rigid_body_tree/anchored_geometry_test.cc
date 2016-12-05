#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/parser_sdf.h"
#include "drake/multibody/rigid_body_tree.h"

// This tests the functionality for classifying geometry in a RigidBodyTree
// as "anchored".  It confirms that the DrakeCollision::Element exhibits the
// property and that the corresponding bullet collision element likewise
// exhibits this property.

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace rigid_body_tree {
namespace {

using drake::parsers::sdf::AddModelInstancesFromSdfFile;

// Utility function for asserting that a body's collision elements are marked as
// static.
void ExpectAnchored(RigidBody<double>* body, size_t collision_element_num,
                    bool is_anchored) {
  EXPECT_EQ(collision_element_num, body->get_collision_element_ids().size());
  for (auto itr = body->collision_elements_begin();
       itr != body->collision_elements_end(); ++itr) {
    EXPECT_EQ(is_anchored, (*itr)->is_static());
  }
}

// Confirms that parentless links are which are *fixed* to world as a parse
// setting are marked as anchored.
GTEST_TEST(SdfAnchoredGeometry, ParentlessLinkFixedToWorld) {
  RigidBodyTree<double> tree;
  // NOTE: nullptr for weld_to_frame implies welding to the world frame.
  AddModelInstancesFromSdfFile(drake::GetDrakePath() + "/multibody/test/rigid_body_tree/anchored_parentless_link.sdf",
                               drake::multibody::joints::kFixed, nullptr,
                               &tree);
  auto body = tree.FindBody("parentless_body");
  ExpectAnchored(body, 1, true);
}

// Confirms that parentless links are which are *floated* on world as a parse
// setting are *not* marked as anchored.
GTEST_TEST(SdfAnchoredGeometry, ParentlessLinkFloatOnWorld) {
  RigidBodyTree<double> tree;
  // NOTE: nullptr for weld_to_frame implies welding to the world frame.
  AddModelInstancesFromSdfFile(drake::GetDrakePath() + "/multibody/test/rigid_body_tree/anchored_parentless_link.sdf",
                               drake::multibody::joints::kRollPitchYaw, nullptr,
                               &tree);
  auto body = tree.FindBody("parentless_body");
  ExpectAnchored(body, 1, false);
}

}  // namespace
}  // namespace rigid_body_tree
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
