/* clang-format off */
#include "drake/multibody/rigid_body_tree.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"

// This tests the functionality for classifying geometry in a RigidBodyTree
// as "anchored".  It confirms that, after calling RigidBodyTree::compile, the
// correct collision elements have been flagged as anchored.

namespace drake {
namespace multibody {
namespace test {
namespace rigid_body_tree {
namespace {

using drake::parsers::sdf::AddModelInstancesFromSdfFile;
using drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld;
using Eigen::Isometry3d;
using Eigen::Vector3d;

// Utility function for asserting that a body's collision elements are marked as
// static.
void ExpectAnchored(RigidBody<double>* body, size_t collision_element_num,
                    bool is_anchored) {
  EXPECT_EQ(collision_element_num, body->get_collision_element_ids().size());
  for (auto itr = body->collision_elements_begin();
       itr != body->collision_elements_end(); ++itr) {
    EXPECT_EQ((*itr)->is_anchored(), is_anchored);
    // TODO(SeanCurtis-TRI): It would be good to confirm that the underlying
    // collision object (btCollisionObject) has the appropriate
    // collision_filter_group based on this: e.g., StaticFilter for anchored,
    // DefaultFilter for not.
  }
}

// Confirms that parentless links, which are specified to be *fixed* to the
// world by the parser, are marked as anchored from SDF file.
GTEST_TEST(SdfAnchoredGeometry, ParentlessLinkFixedToWorld) {
  RigidBodyTree<double> tree;
  // NOTE: nullptr for weld_to_frame implies welding to the world frame.
  AddModelInstancesFromSdfFile(
      drake::GetDrakePath() +
          "/multibody/test/rigid_body_tree/anchored_parentless_link.sdf",
      drake::multibody::joints::kFixed, nullptr, &tree);
  auto body = tree.FindBody("parentless_body");
  ExpectAnchored(body, 1, true);
}

// Confirms that parentless links, which are specified to be *floating* to the
// world by the parser, are *not* marked as anchored from SDF file.
GTEST_TEST(SdfAnchoredGeometry, ParentlessLinkFloatOnWorld) {
  RigidBodyTree<double> tree;
  // NOTE: nullptr for weld_to_frame implies welding to the world frame.
  AddModelInstancesFromSdfFile(
      drake::GetDrakePath() +
          "/multibody/test/rigid_body_tree/anchored_parentless_link.sdf",
      drake::multibody::joints::kRollPitchYaw, nullptr, &tree);
  auto body = tree.FindBody("parentless_body");
  ExpectAnchored(body, 1, false);
}

// Confirms that a body that is rigidly fixed to an anchored body is likewise
// marked as anchored from SDF file.
GTEST_TEST(SdfAnchoredGeometry, LinkedToAnchoredIsAnchored) {
  RigidBodyTree<double> tree;
  // NOTE: nullptr for weld_to_frame implies welding to the world frame.
  AddModelInstancesFromSdfFile(
      drake::GetDrakePath() +
          "/multibody/test/rigid_body_tree/anchored_fixed_to_parent.sdf",
      drake::multibody::joints::kFixed, nullptr, &tree);
  auto body = tree.FindBody("parentless_body");
  ExpectAnchored(body, 1, true);
  body = tree.FindBody("fixed_body");
  ExpectAnchored(body, 1, true);
}

// Confirms that a body that is rigidly fixed to a dynamic body is *not*
// marked as anchored from SDF file.
GTEST_TEST(SdfAnchoredGeometry, LinkedToFloatdIsNotAnchored) {
  RigidBodyTree<double> tree;
  // NOTE: nullptr for weld_to_frame implies welding to the world frame.
  AddModelInstancesFromSdfFile(
      drake::GetDrakePath() +
          "/multibody/test/rigid_body_tree/anchored_fixed_to_parent.sdf",
      drake::multibody::joints::kQuaternion, nullptr, &tree);
  auto body = tree.FindBody("parentless_body");
  ExpectAnchored(body, 1, false);
  body = tree.FindBody("fixed_body");
  ExpectAnchored(body, 1, false);
}



// Confirms that parentless links, which are specified to be *fixed* to the
// world by the parser, are marked as anchored from URDF file.
GTEST_TEST(UrdfAnchoredGeometry, ParentlessLinkFixedToWorld) {
  RigidBodyTree<double> tree;
  // NOTE: nullptr for weld_to_frame implies welding to the world frame.
  AddModelInstanceFromUrdfFileToWorld(
      drake::GetDrakePath() +
          "/multibody/test/rigid_body_tree/anchored_parentless_link.urdf",
      drake::multibody::joints::kFixed, &tree);
  auto body = tree.FindBody("parentless_body");
  ExpectAnchored(body, 1, true);
}

// Confirms that parentless links, which are specified to be *floating* to the
// world by the parser, are *not* marked as anchored from URDF file.
GTEST_TEST(UrdfAnchoredGeometry, ParentlessLinkFloatOnWorld) {
  RigidBodyTree<double> tree;
  // NOTE: nullptr for weld_to_frame implies welding to the world frame.
  AddModelInstanceFromUrdfFileToWorld(
      drake::GetDrakePath() +
          "/multibody/test/rigid_body_tree/anchored_parentless_link.urdf",
      drake::multibody::joints::kRollPitchYaw, &tree);
  auto body = tree.FindBody("parentless_body");
  ExpectAnchored(body, 1, false);
}

// Confirms that a body that is rigidly fixed to an anchored body is likewise
// marked as anchored from URDF file.
GTEST_TEST(UrdfAnchoredGeometry, LinkedToAnchoredIsAnchored) {
  RigidBodyTree<double> tree;
  // NOTE: nullptr for weld_to_frame implies welding to the world frame.
  AddModelInstanceFromUrdfFileToWorld(
      drake::GetDrakePath() +
          "/multibody/test/rigid_body_tree/anchored_fixed_to_parent.urdf",
      drake::multibody::joints::kFixed, &tree);
  auto body = tree.FindBody("parentless_body");
  ExpectAnchored(body, 1, true);
  body = tree.FindBody("fixed_body");
  ExpectAnchored(body, 1, true);
}

// Confirms that a body that is rigidly fixed to a dyanmic body is *not*
// marked as anchored from URDF file.
GTEST_TEST(UrdfAnchoredGeometry, LinkedToFloatdIsNotAnchored) {
  RigidBodyTree<double> tree;
  // NOTE: nullptr for weld_to_frame implies welding to the world frame.
  AddModelInstanceFromUrdfFileToWorld(
      drake::GetDrakePath() +
          "/multibody/test/rigid_body_tree/anchored_fixed_to_parent.urdf",
      drake::multibody::joints::kQuaternion, &tree);
  auto body = tree.FindBody("parentless_body");
  ExpectAnchored(body, 1, false);
  body = tree.FindBody("fixed_body");
  ExpectAnchored(body, 1, false);
}



// Tests the case where collision elements are explicitly added to the world.
// Those elements *should* be marked anchored.
GTEST_TEST(ByHandAnchoredGeometry, WorldCollisionElementIsAnchored) {
  RigidBodyTree<double> tree;
  RigidBody<double>& world = tree.world();
  DrakeShapes::Box geom(Vector3d(1.0, 1.0, 1.0));
  DrakeCollision::Element element(geom, Isometry3d::Identity(), &world);
  tree.addCollisionElement(element, world, "rigid_body");
  tree.compile();

  ExpectAnchored(&world, 1, true);
}

}  // namespace
}  // namespace rigid_body_tree
}  // namespace test
}  // namespace multibody
}  // namespace drake
