#include "drake/perception/estimators/dev/pose_closed_form.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_geometry_compare.h"
#include "drake/common/text_logging.h"
#include "drake/math/rotation_matrix.h"
#include "drake/perception/estimators/dev/test/test_util.h"

using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::Isometry3d;

using ::testing::TestWithParam;

namespace drake {
namespace perception {
namespace estimators {
namespace {

class PoseClosedFormTest : public TestWithParam<ObjectTestType> {
 public:
  void SetUp() override {
    ObjectTestType type = GetParam();
    ObjectTestSetup setup;
    GetObjectTestSetup(type, &setup);
    drake::log()->debug("urdf: {}", setup.urdf_file);
    points_B_ = setup.points_B;
    X_WB_ = setup.X_WB;

    X_BP_expected_ = Isometry3d::Identity();
    switch (type) {
      case kSimpleCuboid: {
        // Provide expected principal axes and centroid.
        is_simple_shape_ = true;
        X_BP_expected_.linear() << Vector3d::UnitZ(), Vector3d::UnitX(),
                                   Vector3d::UnitY();
        break;
      }
      case kBlueFunnelScan: {
        is_simple_shape_ = false;
      }
    }
  }

 protected:
  Matrix3Xd points_B_;
  Isometry3d X_WB_;
  // Indicates that this is a simple shape, and we should check the expected
  // principal axes and the centroid.
  bool is_simple_shape_;
  // Expected principal axes and geometric centroid (frame P, w.r.t. body
  // frame B).
  Isometry3d X_BP_expected_;
};

/*
 * This tests PCA and SVD against ground truth individually, using the same
 * data.
 */
TEST_P(PoseClosedFormTest, PcaAndSvd) {
  // Show the points in the world frame with identity transform.
  // Note that `Pb` is the body frame inferred by PCA in this instance.
  const Isometry3d X_BPb_pca = ComputePcaBodyPose(points_B_);
  const double tol = 1e-5;
  if (is_simple_shape_) {
    // Check expected PCA pose w.r.t. the body frame.
    EXPECT_TRUE(
        CompareTransformWithoutAxisSign(X_BP_expected_, X_BPb_pca, tol));
  }
  // Transform points to a pose in the world, such that the ground truth pose
  // is X_WB.
  const Matrix3Xd points_B_W = X_WB_ * points_B_;
  // Quick meta-test on CompareTransformWithoutAxisSign.
  EXPECT_FALSE(CompareTransformWithoutAxisSign(X_BP_expected_, X_WB_, tol));
  // Compute PCA for body. Note that frame `Pw` is also inferred by PCA, and
  // may not represent the same frame as `Pb` due to axis sign permutations.
  const Isometry3d X_WPw_pca = ComputePcaBodyPose(points_B_W);
  // Transform `Pw` to be expressed in B, since we have X_WB as ground truth,
  // and cannot combine X_WPw and X_BPb with a non-zero centroid, since the
  // sign permutations may cause the transformed centroids to be mismatched.
  // TODO(eric.cousineau): Revert logic back to computing `X_WBp`, since that
  // is useful in actual applications.
  // This *could* be achieved by checking transforms with minimal
  // distance from identity, or some other constraints to try and eliminate the
  // redundant solutions from PCA (except for transforms equidistant from
  // identity).
  const Isometry3d X_BPw_pca = X_WB_.inverse() * X_WPw_pca;
  EXPECT_TRUE(CompareTransformWithoutAxisSign(X_BPb_pca, X_BPw_pca, tol));
  // Compute SVD for the body pose. Transformations should match, sign and all.
  const Isometry3d X_WB_svd = ComputeSvdBodyPose(points_B_, points_B_W);
  EXPECT_TRUE(CompareTransforms(X_WB_, X_WB_svd, tol));
  // For visualization, use `show_frames`:
  //   drake_visualizer --script
  //       ./drake/multibody/rigid_body_plant/visualization/show_frames.py
  PointCloudVisualizer vis;
  vis.PublishCloud(points_B_, "B");
  vis.PublishCloud(points_B_W, "BW");
  const Isometry3d X_BB = Isometry3d::Identity();
  vis.PublishFrames({{"X_BB", X_BB},
                     {"X_BP_expected", X_BP_expected_},
                     {"X_WB", X_WB_},
                     {"X_BPb_pca", X_BPb_pca},
                     {"X_WPw_pca", X_WPw_pca},
                     {"X_BPw_pca", X_BPw_pca},
                     {"X_WB_svd", X_WB_svd}});
}

// Instantiate parameterized test cases.
INSTANTIATE_TEST_CASE_P(test, PoseClosedFormTest, ObjectTestTypes);

}  // namespace
}  // namespace estimators
}  // namespace perception
}  // namespace drake
