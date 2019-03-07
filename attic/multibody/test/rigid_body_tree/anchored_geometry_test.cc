/* clang-format off to disable clang-format-includes */
#include "drake/multibody/rigid_body_tree.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
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

// Utility function for asserting that no pairs of collision elements on two
// bodies are checked for collisions.
void ExpectAllPairsIgnored(RigidBody<double>* first_body,
                           RigidBody<double>* second_body) {
  for (auto first_itr = first_body->collision_elements_begin();
       first_itr != first_body->collision_elements_end(); ++first_itr) {
    for (auto second_itr = second_body->collision_elements_begin();
         second_itr != second_body->collision_elements_end(); ++second_itr) {
      EXPECT_FALSE((*first_itr)->CanCollideWith(*second_itr));
    }
  }
}

// Confirms that parentless links, which are specified to be *fixed* to the
// world by the parser, are marked as anchored from SDF file.
GTEST_TEST(SdfAnchoredGeometry, ParentlessLinkFixedToWorld) {
  RigidBodyTree<double> tree;
  // NOTE: nullptr for weld_to_frame implies welding to the world frame.
  AddModelInstancesFromSdfFile(
      FindResourceOrThrow(
          "drake/multibody/test/rigid_body_tree/anchored_parentless_link.sdf"),
      drake::multibody::joints::kFixed, nullptr, &tree);
  auto body = tree.FindBody("parentless_body");
  ExpectAnchored(body, 1, true);
}

// Confirms that parentless links, which are specified to be *floating* to the
// world by the parser, are *not* marked as anchored from SDF file.
GTEST_TEST(SdfAnchoredGeometry, ParentlessLinkFloatOnWorld) {
  // N.B. We test what happens when welding using the "world" frame that has the
  // actual world body specified.
  for (bool clone_initial : {false, true}) {
    for (bool use_world_frame : {false, true}) {
      auto tree = std::make_unique<RigidBodyTree<double>>();
      if (clone_initial) {
        tree = tree->Clone();
      }
      std::shared_ptr<RigidBodyFrame<double>> parent_frame{nullptr};
      if (use_world_frame) {
        parent_frame = tree->findFrame("world");
        EXPECT_FALSE(parent_frame->has_as_rigid_body(nullptr));
      }
      // NOTE: nullptr for weld_to_frame implies welding to the world frame.
      AddModelInstancesFromSdfFile(
          FindResourceOrThrow(
              "drake/multibody/test/rigid_body_tree/"
              "anchored_parentless_link.sdf"),
          drake::multibody::joints::kRollPitchYaw, parent_frame, tree.get());
      auto body = tree->FindBody("parentless_body");
      ExpectAnchored(body, 1, false);
    }
  }
}

// Confirms that a body that is rigidly fixed to an anchored body is likewise
// marked as anchored from SDF file.
GTEST_TEST(SdfAnchoredGeometry, LinkedToAnchoredIsAnchored) {
  RigidBodyTree<double> tree;
  // NOTE: nullptr for weld_to_frame implies welding to the world frame.
  AddModelInstancesFromSdfFile(
      FindResourceOrThrow(
          "drake/multibody/test/rigid_body_tree/anchored_fixed_to_parent.sdf"),
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
      FindResourceOrThrow(
          "drake/multibody/test/rigid_body_tree/anchored_fixed_to_parent.sdf"),
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
      FindResourceOrThrow(
          "drake/multibody/test/rigid_body_tree/anchored_parentless_link.urdf"),
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
      FindResourceOrThrow(
          "drake/multibody/test/rigid_body_tree/anchored_parentless_link.urdf"),
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
      FindResourceOrThrow(
          "drake/multibody/test/rigid_body_tree/anchored_fixed_to_parent.urdf"),
      drake::multibody::joints::kFixed, &tree);
  auto body = tree.FindBody("parentless_body");
  ExpectAnchored(body, 1, true);
  body = tree.FindBody("fixed_body");
  ExpectAnchored(body, 1, true);
}

// Confirms that a body that is rigidly fixed to a dynamic body is *not*
// marked as anchored from URDF file.
GTEST_TEST(UrdfAnchoredGeometry, LinkedToFloatdIsNotAnchored) {
  RigidBodyTree<double> tree;
  // NOTE: nullptr for weld_to_frame implies welding to the world frame.
  AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(
          "drake/multibody/test/rigid_body_tree/anchored_fixed_to_parent.urdf"),
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
  drake::multibody::collision::Element element(geom, Isometry3d::Identity(),
                                               &world);
  tree.addCollisionElement(element, world, "rigid_body");
  tree.compile();

  ExpectAnchored(&world, 1, true);
}

// Tests that anchored collision geometries are not checked against each other.
GTEST_TEST(AnchoredElementsIgnoreEachOther, AnchoredElementsIgnoreEachOther) {
  RigidBodyTree<double> tree;
  auto add_anchored_body = [](RigidBodyTree<double>* model) -> int {
    return AddModelInstanceFromUrdfFileToWorld(
               FindResourceOrThrow("drake/multibody/test/rigid_body_tree/"
                                   "anchored_fixed_to_parent.urdf"),
               drake::multibody::joints::kFixed, model)
        .begin()
        ->second;
  };
  const int first_model_instance_id = add_anchored_body(&tree);
  const int second_model_instance_id = add_anchored_body(&tree);
  auto body1 = tree.FindBody("parentless_body", "", first_model_instance_id);
  ExpectAnchored(body1, 1, true);
  auto body2 = tree.FindBody("parentless_body", "", second_model_instance_id);
  ExpectAnchored(body2, 1, true);
  ExpectAllPairsIgnored(body1, body2);
}

}  // namespace
}  // namespace rigid_body_tree
}  // namespace test
}  // namespace multibody
}  // namespace drake
