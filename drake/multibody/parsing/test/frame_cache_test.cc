#include "drake/multibody/parsing/frame_cache.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace {

// Makes sure that FrameCache correctly keeps frames and the transforms that
// relate them.
GTEST_TEST(FrameCacheTest, TransformTest) {
  FrameCache<double> frame_cache("world");
  // Define body frame's (B) translation in world frame (W).
  const typename Isometry3<double>::TranslationType T_WB(1.0, -2.0, 0);
  // Define body frame's (B) rotation in world frame (W).
  const AngleAxis<double> R_WB(M_PI, Vector3<double>::UnitZ());
  // Compose body frame's (B) pose in world frame (W) from translation
  // and rotation matrices.
  const Isometry3<double> X_WB = R_WB * T_WB;
  // Update tree with body frame's (B) pose in world frame (W).
  frame_cache.Update("world", "body", X_WB);

  // Define arm frame's (A) translation in body frame (B).
  const Isometry3<double>::TranslationType T_BA(0.5, 0.0, 1.8);
  // Define arm frame's (A) rotation in body frame (B).
  const AngleAxis<double> R_BA(0.1 * M_PI, Vector3<double>::UnitY());
  // Compose arm frame's (A) pose in body frame (B) from translation
  // and rotation matrices.
  const Isometry3<double> X_BA = R_BA * T_BA;
  // Update tree with arm frame's (A) pose in body frame (B).
  frame_cache.Update("body", "arm", X_BA);

  // Check that pose resolution works as intended.
  EXPECT_TRUE(frame_cache.Transform("world", "arm").isApprox(X_WB * X_BA));

  // Translate body frame's (B) pose in world frame (W)..
  const Isometry3<double>::TranslationType T_B(0, -1.0, 0);
  const Isometry3<double> X_WB2 = T_B * X_WB;
  // Update tree with body frame's (B) pose in world frame (W).
  frame_cache.Update("world", "body", X_WB2);

  // Check that affecting the body frame's (B) pose did not alter
  // the arm frame's (A) pose in that same frame.
  EXPECT_TRUE(frame_cache.Transform("body", "arm").isApprox(X_BA));

  // Check that the arm frame's (A) pose in world frame (W) is correctly
  // re-computed.
  EXPECT_TRUE(frame_cache.Transform("world", "arm").isApprox(X_WB2 * X_BA));

  // Check that any attempt to introduce a frame cycle into the cache
  // results in an exception being thrown.
  ASSERT_THROW({
      frame_cache.Update("arm", "body", Isometry3<double>::Identity());
    }, std::runtime_error);

  ASSERT_THROW({
      frame_cache.Update("arm", "world", Isometry3<double>::Identity());
    }, std::runtime_error);

  // Check that any attempt to update a frame's pose in itself
  // results in an exception being thrown, except when the identity
  // transform is given.
  ASSERT_NO_THROW({
      frame_cache.Update("arm", "arm", Isometry3<double>::Identity());
    });

  ASSERT_THROW({
      frame_cache.Update("arm", "arm", Isometry3<double>(
          AngleAxis<double>(M_PI, Vector3<double>::UnitZ())));
    }, std::runtime_error);
}

// Makes sure that a FrameCache throws when trying to update using a non
// existent target frame or trying to transform using non existent frames.
GTEST_TEST(FrameCacheTest, NonExistentFramesTest) {
  FrameCache<double> frame_cache("root");
  ASSERT_THROW({
      frame_cache.Update(
          "not-a-frame", "neither-a-frame",
          Isometry3<double>::Identity());
    }, std::runtime_error);
  ASSERT_THROW({
      frame_cache.Transform(
          "root", "not-a-frame");
    }, std::runtime_error);
  ASSERT_THROW({
      frame_cache.Transform(
          "neither-a-frame", "root");
    }, std::runtime_error);
}

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
