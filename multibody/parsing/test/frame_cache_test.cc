#include "drake/multibody/parsing/frame_cache.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace {

// Makes sure that FrameCache correctly keeps frames and the transforms that
// relate them.
GTEST_TEST(FrameCacheTest, TransformTest) {
  FrameCache<double> frame_cache("world");
  // Form body-frame B's pose in world frame W from rotation and translation.
  const math::RigidTransformd X_WB(math::RotationMatrixd::MakeZRotation(M_PI),
                                   Eigen::Vector3d(1.0, -2.0, 0));

  // Update tree with body frame's (B) pose in world frame (W).
  frame_cache.Update("world", "body", X_WB.GetAsIsometry3());

  // Form arm frame A's pose in body frame B from rotation and translation.
  const math::RigidTransformd X_BA(
      math::RotationMatrixd::MakeYRotation(0.1 * M_PI),
      Eigen::Vector3d(0.5, 0.0, 1.8));

  // Update tree with arm frame's (A) pose in body frame (B).
  frame_cache.Update("body", "arm", X_BA.GetAsIsometry3());

  // Check that pose resolution works as intended.
  const double kTolerance = 16 * std::numeric_limits<double>::epsilon();
  math::RigidTransformd X_WA = X_WB * X_BA;
  EXPECT_TRUE(frame_cache.RigidTransform("world", "arm").IsNearlyEqualTo(X_WA,
              kTolerance));

  // Translate body frame's (B) pose in world frame (W)..
  const math::RigidTransformd T_B(Eigen::Vector3d(0, -1.0, 0));
  const math::RigidTransformd X_WB2 = T_B * X_WB;
  // Update tree with body frame's (B) pose in world frame (W).
  frame_cache.Update("world", "body", X_WB2.GetAsIsometry3());

  // Check that affecting the body frame's (B) pose did not alter
  // the arm frame's (A) pose in that same frame.
  EXPECT_TRUE(frame_cache.RigidTransform("body", "arm").IsNearlyEqualTo(X_BA,
              kTolerance));

  // Ensure arm frame's (A) pose in world frame (W) is correctly re-computed.
  X_WA = X_WB2 * X_BA;
  EXPECT_TRUE(frame_cache.RigidTransform("world", "arm").IsNearlyEqualTo(X_WA,
              kTolerance));

  // Check that any attempt to introduce a frame cycle into the cache
  // results in an exception being thrown.
  const math::RigidTransformd X_identity;
  ASSERT_THROW({
      frame_cache.Update("arm", "body", X_identity.GetAsIsometry3());
    }, std::runtime_error);

  ASSERT_THROW({
      frame_cache.Update("arm", "world", X_identity.GetAsIsometry3());
    }, std::runtime_error);

  // Check that any attempt to update a frame's pose in itself
  // results in an exception being thrown, except when the identity
  // transform is given.
  ASSERT_NO_THROW({
      frame_cache.Update("arm", "arm", X_identity.GetAsIsometry3());
    });

  const math::RigidTransformd X_non_identity(
      math::RotationMatrixd::MakeZRotation(M_PI));
  ASSERT_THROW({
      frame_cache.Update("arm", "arm", X_non_identity.GetAsIsometry3());
    }, std::runtime_error);
}

// Makes sure that a FrameCache throws when trying to update using a non
// existent target frame or trying to transform using non existent frames.
GTEST_TEST(FrameCacheTest, NonExistentFramesTest) {
  FrameCache<double> frame_cache("root");
  ASSERT_THROW({
      frame_cache.Update(
          "not-a-frame", "neither-a-frame",
          math::RigidTransform<double>::Identity().GetAsIsometry3());
    }, std::runtime_error);
  ASSERT_THROW({
      frame_cache.RigidTransform("root", "not-a-frame");
    }, std::runtime_error);
  ASSERT_THROW({
      frame_cache.RigidTransform("neither-a-frame", "root");
    }, std::runtime_error);
}

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
