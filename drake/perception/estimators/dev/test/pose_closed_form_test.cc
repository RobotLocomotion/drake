#include "drake/perception/estimators/dev/pose_closed_form.h"

#include <gtest/gtest.h>

#include "drake/math/roll_pitch_yaw.h"
#include "drake/perception/estimators/dev/test/test_util.h"

using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::Isometry3d;

namespace drake {
namespace perception {
namespace estimators {
namespace {

/*
 * This tests PCA and SVD against ground truth individually, using the same
 * data.
 */
GTEST_TEST(PoseClosedForm, PcaAndSvd) {
  // Use a bound whose principal axes (in order) are NOT (x, y, z), but
  // rather (z, x, y).
  const Bounds box(
      Interval(-0.05, 0.05),
      Interval(-0.01, 0.01),
      Interval(-0.2, 0.2));
  const double spacing = 0.01;
  // Generate points in body frame.
  const Matrix3Xd points_B = GenerateBoxPointCloud(spacing, box);
  // Expect principal axes for PCA, where `P` is the body frame that aligns
  // with the principal axes.
  Isometry3d X_BP = Isometry3d::Identity();
  X_BP.linear() << Vector3d::UnitZ(), Vector3d::UnitX(), Vector3d::UnitY();
  // Show the points in the world frame with identity transform.
  // Note that `Pb` is the body frame inferred by PCA in this instance.
  const Isometry3d X_BPb_pca = ComputePcaBodyPose(points_B);
  const double tol = 1e-5;
  // Since our model is geometrically centered, this should be at or near
  // identity with PCA, with possible permutations on the axis signs.
  EXPECT_TRUE(CompareTransformWithoutAxisSign(X_BP, X_BPb_pca, tol));
  // Transform points to a pose in the world, such that the ground truth pose
  // is X_WB.
  const Vector3d xyz(0.1, 0.2, 0.3);
  const Vector3d rpy(kPi / 3, kPi / 11, kPi / 12);
  Isometry3d X_WB;
  X_WB.setIdentity();
  X_WB.linear() << drake::math::rpy2rotmat(rpy);
  X_WB.translation() << xyz;
  const Matrix3Xd points_B_W = X_WB * points_B;
  // Quick meta-test on CompareTransformWithoutAxisSign.
  EXPECT_FALSE(CompareTransformWithoutAxisSign(X_BP, X_WB, tol));
  // Compute PCA for body. Note that frame `Pw` is also inferred by PCA, and
  // may not represent the same frame as `Pb` due to axis sign permutations.
  const Isometry3d X_WPw_pca = ComputePcaBodyPose(points_B_W);
  // Compute frame `Bp` aligned with intended body frame, rather than
  // principal axes. This will either be `B` if `Pb` and `Pw` are the
  // same, or will have axes aligned with `B`, but possibly with different
  // signs.
  const Isometry3d X_WBp_pca = X_WPw_pca * X_BPb_pca.inverse();
  EXPECT_TRUE(CompareTransformWithoutAxisSign(X_WB, X_WBp_pca, tol));
  // Compute SVD for the body pose. Transformations should match, sign and all.
  const Isometry3d X_WB_svd = ComputeSvdBodyPose(points_B, points_B_W);
  EXPECT_TRUE(CompareTransforms(X_WB, X_WB_svd, tol));
  // For visualization, use `show_frames`:
  //   drake_visualizer --script
  //       ./drake/multibody/rigid_body_plant/visualization/show_frames.py
  PointCloudVisualizer vis;
  vis.PublishCloud(points_B, "B");
  vis.PublishCloud(points_B_W, "BW");
  const Isometry3d X_BB = Isometry3d::Identity();
  vis.PublishFrames({{"X_BB", X_BB},
                     {"X_BP", X_BP},
                     {"X_WB", X_WB},
                     {"X_BPb_pca", X_BPb_pca},
                     {"X_WPw_pca", X_WPw_pca},
                     {"X_WBp_pca", X_WBp_pca},
                     {"X_WB_svd", X_WB_svd}});
}

}  // namespace
}  // namespace estimators
}  // namespace perception
}  // namespace drake
