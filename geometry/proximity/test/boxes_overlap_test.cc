#include "drake/geometry/proximity/boxes_overlap.h"

#include <string>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;

// We want to compute X_AB such that B is posed relative to A as documented in
// BoxesOverlapTest. We can do so by generating the rotation component, R_AB,
// such that Bq has a minimum value along the chosen axis, and we can solve for
// the translation component, p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A.
// The boxes are represented by triples of their half sizes (measured in each
// box's canonical frame).
RigidTransformd CalcCornerTransform(const Vector3d& a_half,
                                    const Vector3d& b_half, const int axis,
                                    const bool expect_overlap) {
  const int axis1 = (axis + 1) % 3;
  const int axis2 = (axis + 2) % 3;
  // Construct the rotation matrix, R_AB, that has meaningful (non-zero)
  // values everywhere for the remaining 2 axes and no symmetry.
  const RotationMatrixd R_AB =
      RotationMatrixd(AngleAxisd(M_PI / 5, Vector3d::Unit(axis1))) *
      RotationMatrixd(AngleAxisd(-M_PI / 5, Vector3d::Unit(axis2)));

  // We define p_BqBo in Frame A from box b's minimum corner Q to its center.
  const Vector3d p_BqBo_A = R_AB * b_half;
  // Reality check that the minimum corner and the center are strictly
  // increasing along the given axis because we chose the rotation R_AB to
  // support this property.
  DRAKE_DEMAND(p_BqBo_A[axis] > 0.);

  // We construct Bq to be a small relative offset either side of Af along the
  // given A[axis], depending on whether we expect the boxes to overlap.
  Vector3d p_AfBq_A{0, 0, 0};
  p_AfBq_A[axis] = expect_overlap ? -0.01 : 0.01;

  // We construct Af by taking the maximum corner and offsetting it along the
  // remaining 2 axes, e.g. by a quarter across. This ensures we thoroughly
  // exercise all bits instead of simply using any midpoints or corners.
  //
  //           A[axis1]
  //           ^
  //   --------|--------
  //   |       |       |
  //   |       |       Af
  //   |       |       |
  //   |      Ao------->A[axis]
  //   |               |
  //   |               |
  //   |               |
  //   -----------------
  //
  Vector3d p_AoAf_A = a_half;
  p_AoAf_A[axis1] /= 2;
  p_AoAf_A[axis2] /= 2;

  const Vector3d p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A;
  return RigidTransformd(R_AB, p_AoBo_A);
}

// We want to compute X_AB such that B is posed relative to A as documented in
// BoxesOverlapTest. We can do so by generating the rotation component, R_AB,
// such that Bq lies on the minimum edge along the chosen axis, and we can solve
// for the translation component, p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A.
RigidTransformd CalcEdgeTransform(const Vector3d& a_half,
                                  const Vector3d& b_half, const int a_axis,
                                  const int b_axis, const bool expect_overlap) {
  const int a_axis1 = (a_axis + 1) % 3;
  const int a_axis2 = (a_axis + 2) % 3;
  const int b_axis1 = (b_axis + 1) % 3;
  const int b_axis2 = (b_axis + 2) % 3;
  // Construct a rotation matrix that has meaningful (non-zero) values
  // everywhere for the remaining 2 axes and no symmetry. Depending on the
  // combination of axes, we need to rotate around different axes to ensure
  // the edge remains as the minimum.
  RotationMatrixd R_AB;
  const double theta = M_PI / 5;
  // For cases Ax × Bx, Ay × By, and Az × Bz.
  if (a_axis == b_axis) {
    R_AB = RotationMatrixd(AngleAxisd(theta, Vector3d::Unit(b_axis1))) *
           RotationMatrixd(AngleAxisd(theta, Vector3d::Unit(b_axis2)));
    // For cases Ax × By, Ay × Bz, and Az × Bx.
  } else if (a_axis1 == b_axis) {
    R_AB = RotationMatrixd(AngleAxisd(theta, Vector3d::Unit(b_axis1))) *
           RotationMatrixd(AngleAxisd(-theta, Vector3d::Unit(b_axis2)));
    // For cases Ax × Bz, Ay × Bx, and Az × By.
  } else {
    R_AB = RotationMatrixd(AngleAxisd(-theta, Vector3d::Unit(b_axis2))) *
           RotationMatrixd(AngleAxisd(theta, Vector3d::Unit(b_axis1)));
  }

  // We define p_BqBo in Frame B taking a point on the minimum edge aligned
  // with the given axis, offset it to be without symmetry, then convert it
  // to Frame A by applying the rotation.
  Vector3d p_BqBo_B = b_half;
  p_BqBo_B[b_axis] -= b_half[b_axis] / 2;
  const Vector3d p_BqBo_A = R_AB * p_BqBo_B;
  // Reality check that the point Bq and the center Bo are strictly
  // increasing along the remaining 2 axes because we chose the rotation R_AB
  // to support this property.
  DRAKE_DEMAND(p_BqBo_A[a_axis1] > 0);
  DRAKE_DEMAND(p_BqBo_A[a_axis2] > 0);

  // We construct Bq to be a small relative offset either side of Af along the
  // given axis, depending on whether we expect the boxes to overlap.
  Vector3d p_AfBq_A{0, 0, 0};
  const double offset = expect_overlap ? -0.01 : 0.01;
  p_AfBq_A[a_axis1] = offset;
  p_AfBq_A[a_axis2] = offset;

  // We construct Af by taking the maximum corner and offsetting it along the
  // given edge to thoroughly exercise all bits.
  Vector3d p_AoAf_A = a_half;
  p_AoAf_A[a_axis] -= a_half[a_axis] / 2;

  Vector3d p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A;
  // Finally we combine the components to form the transform X_AB.
  return RigidTransformd(R_AB, p_AoBo_A);
}

class BoxesOverlapTest : public ::testing::Test {
 public:
  // TODO(SeanCurtis-TRI): Make this a parameterized unit test and parameterize
  // the various test cases in AllCases so a user can choose to execute just the
  // tests they care about. This is the static set up variant in support of that
  // glorious day.
  static void SetUpTestSuite() {
    meshcat_ = std::make_unique<Meshcat>();
    // To have the pause below take effect, invoke the test as
    // ./bazel-bin/geometry/proximity/boxes_overlap_test
    common::MaybePauseForUser(
        "Open a browser to the meshcat URL above and make sure it's connected "
        "before continuing. When the test is complete, the browser will "
        "probably appear empty, but the path drake/BoxesOverlapTest/ should "
        "contain all of the test cases.");
  }

 protected:
  // Tests to see if the two oriented bounding boxes overlap. The boxes are
  // represented with vectors containing their half sizes as measured in their
  // own frames. This tests A against B and B against A. If the two queries
  // return different results, that is an error condition. Otherwise, reports
  // the result of BoxesOverlap().
  static bool InvokeBoxesOverlap(const Vector3d& a_half, const Vector3d& b_half,
                                 const RigidTransformd& X_AB,
                                 const std::string& label) {
    const bool a_to_b = BoxesOverlap(a_half, b_half, X_AB);
    const bool b_to_a = BoxesOverlap(b_half, a_half, X_AB.inverse());
    DrawCase(a_half, b_half, X_AB, "BoxesOverlapTest/" + label);
    DRAKE_DEMAND(a_to_b == b_to_a);
    return a_to_b;
  }

  static void DrawCase(const Vector3d& a_half, const Vector3d& b_half,
                       const RigidTransformd& X_AB, const std::string& path) {
    DRAKE_DEMAND(meshcat_ != nullptr);
    const std::string a_path = path + "/a";
    meshcat_->SetObject(a_path, Box(a_half * 2), Rgba(0.2, 0.2, 0.8, 0.8));
    const std::string b_path = path + "/b";
    meshcat_->SetObject(b_path, Box(b_half * 2), Rgba(0.8, 0.5, 0.2, 0.8));
    meshcat_->SetTransform(b_path, X_AB);
    meshcat_->SetProperty(path, "visible", false);
  }

  static std::unique_ptr<Meshcat> meshcat_;
};

std::unique_ptr<Meshcat> BoxesOverlapTest::meshcat_{nullptr};

// Tests whether OBBs overlap. There are 15 unique possibilities for
// non-overlap. Therefore, there are 15 cases to test, each covering a different
// separating axis between the two bounding boxes. The first 3 cases use the
// axes of Frame A, the next 3 cases use the axes of Frame B, and the remaining
// 9 cases use the axes defined by the cross product of axes from Frame A and
// Frame B. We also test that it is robust for the case of parallel boxes.
TEST_F(BoxesOverlapTest, AllCases) {
  // Each box simply gets posed in a common frame, G.
  const char* axes[] = {"x", "y", "z"};

  // One box is fully contained in the other and they are parallel.
  // (This transform will get repeatedly overwritten below.)
  RigidTransformd X_AB(Vector3d(0.2, 0.4, 0.2));
  Vector3d a(1, 2, 1);
  Vector3d b(0.5, 1, 0.5);
  EXPECT_TRUE(InvokeBoxesOverlap(a, b, X_AB, "parallel_and_contained"));

  // The first cases are where the separating plane is perpendicular to an axis
  // of frame A. So, we pose box B along each axis. For example, in the case of
  // axis Ax, in a 2D view they could look like:
  //           Ay
  //           ^
  //   --------|--------      ⋰ ⋱       ↗By
  //   |       |       |   ⋰      ⋱  ↗
  //   |       |       Af Bq       ↗⋱
  //   |       |       |     ⋱   Bo   ⋱
  //   |      Ao--------->Ax   ⋱    ↘   ⋱
  //   |               |         ⋱    ↘⋰
  //   |               |           ⋱⋰   ↘Bx
  //   |               |
  //   -----------------
  //
  //
  // For this test, we define Point Bq as the minimum corner of the box B (i.e.,
  // center - half width). We want to pose box B so Bq is the uniquely closest
  // point to box A at a Point Af in the interior of +Ax face. The rest of
  // the box B extends farther along the +Ax axis (as suggested in the above
  // illustration). Point Bq will be a small epsilon away from the nearby face
  // either outside (if expect_overlap is false) or inside (if true).
  a = Vector3d(2, 4, 3);
  b = Vector3d(3.5, 2, 1.5);
  for (int axis = 0; axis < 3; ++axis) {
    X_AB = CalcCornerTransform(a, b, axis, false /* expect_overlap */);
    EXPECT_FALSE(InvokeBoxesOverlap(a, b, X_AB,
                                    fmt::format("A{} separated", axes[axis])));
    X_AB = CalcCornerTransform(a, b, axis, true /* expect_overlap */);
    EXPECT_TRUE(InvokeBoxesOverlap(a, b, X_AB,
                                   fmt::format("A{} colliding", axes[axis])));
  }

  // To cover the local axes out of B, we can use the same method by swapping
  // the order of the boxes and then using the inverse of the transform.
  for (int axis = 0; axis < 3; ++axis) {
    X_AB =
        CalcCornerTransform(b, a, axis, false /* expect_overlap */).inverse();
    EXPECT_FALSE(InvokeBoxesOverlap(a, b, X_AB,
                                    fmt::format("B{} separated", axes[axis])));
    X_AB = CalcCornerTransform(b, a, axis, true /* expect_overlap */).inverse();
    EXPECT_TRUE(InvokeBoxesOverlap(a, b, X_AB,
                                   fmt::format("B{} colliding", axes[axis])));
  }

  // To cover the remaining 9 cases, we need to pose an edge from box B along
  // an edge from box A. The axes that the edges are aligned with form the
  // two inputs into the cross product for the separating axis. For example,
  // in the following illustration, Af lies on the edge aligned with A's y-axis.
  // Assuming that Bq lies on an edge aligned with B's x-axis, this would form
  // the case testing the separating axis Ay × Bx.
  //                       _________
  //   +z                 /________/\              .
  //    ^                 \        \ \             .
  //    |   ______________ Bq       \ \            .
  //    |  |\             Af \  Bo   \ \           .
  //    |  | \ _____________\ \       \ \          .
  // +y |  | |      Ao      |  \_______\/          .
  //  \ |  \ |              |                      .
  //   \|   \|______________|                      .
  //    -----------------------------------> +x
  //
  // For this test, we define point Bq on the minimum edge of the box in its
  // own frame (i.e., center - half width + an offset along the edge). We want
  // to pose box B so Bq is the uniquely closest point to A at a Point Af on the
  // edge between the +x and +z face of box A. The rest of box B extends
  // farther along the +Ax and +Az axis (as suggested in the above
  // illustration). Point Bq will be a small epsilon away from the nearby
  // edge either outside (if expect_overlap is false) or inside (if true).
  for (int a_axis = 0; a_axis < 3; ++a_axis) {
    for (int b_axis = 0; b_axis < 3; ++b_axis) {
      X_AB =
          CalcEdgeTransform(a, b, a_axis, b_axis, false /* expect_overlap */);
      // Separate along a's y-axis and b's x-axis.
      EXPECT_FALSE(InvokeBoxesOverlap(
          a, b, X_AB,
          fmt::format("A{}-B{} separated", axes[a_axis], axes[b_axis])));
      X_AB = CalcEdgeTransform(a, b, a_axis, b_axis, true /* expect_overlap */);
      // Separate along a's y-axis and b's x-axis.
      EXPECT_TRUE(InvokeBoxesOverlap(
          a, b, X_AB,
          fmt::format("A{}-B{} colliding", axes[a_axis], axes[b_axis])));
    }
  }
  ReportStatistics();
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
