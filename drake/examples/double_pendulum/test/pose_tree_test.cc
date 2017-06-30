#include "drake/examples/double_pendulum/pose_tree.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace double_pendulum {
namespace {

/// Makes sure that PoseTree correctly keeps
/// frames and the transforms that relate them.
GTEST_TEST(PoseTreeTest, TransformTest) {
  PoseTree<double> pose_tree("world");
  // Define a world->body transform.
  typename Isometry3<double>::TranslationType
      world_to_body_translation(1.0, -2.0, 0);
  AngleAxis<double>
      world_to_body_rotation(M_PI, Vector3<double>::UnitZ());
  Isometry3<double> world_to_body_transform =
      world_to_body_rotation * world_to_body_translation;

  pose_tree.Update(
      "world", "body", world_to_body_transform);
  // Define a body->arm transform.
  typename Isometry3<double>::TranslationType
      body_to_arm_translation(0.5, 0.0, 1.8);
  AngleAxis<double>
      body_to_arm_rotation(0.1*M_PI, Vector3<double>::UnitY());
  Isometry3<double> body_to_arm_transform =
      body_to_arm_rotation * body_to_arm_translation;

  pose_tree.Update(
      "body", "arm", body_to_arm_transform);

  // Check for proper transforms.
  EXPECT_TRUE(pose_tree.Transform("world", "arm").isApprox(
      world_to_body_transform * body_to_arm_transform));
}

/// Makes sure that a PoseTree throws when trying
/// to update using a non existant target frame
/// or trying to transform using non existant frames.
GTEST_TEST(PoseTreeTest, NonExistantFramesTest) {
  PoseTree<double> pose_tree("root");
  ASSERT_THROW({
      pose_tree.Update(
          "not-a-frame", "neither-a-frame",
          Isometry3<double>::Identity());
    }, std::runtime_error);
  ASSERT_THROW({
      pose_tree.Transform(
          "root", "not-a-frame");
    }, std::runtime_error);
  ASSERT_THROW({
      pose_tree.Transform(
          "neither-a-frame", "root");
    }, std::runtime_error);
}

}  // namespace
}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
