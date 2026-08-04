#include "drake/geometry/proximity/boxes_overlap.h"

#include <memory>
#include <string>

#include "hwy/tests/hwy_gtest.h"
#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/hwy_dynamic.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/test_utilities/boxes_overlap_transforms.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;

// TODO(jwnimmer-tri) This similar of fixture (using hwy_gtest.h) is duplicated
// in two other places (//common and //geometry/proximity). We should probably
// seek a more elegant way to do this, instead of copying it to more places.

/* This hwy-infused test fixture replicates every test case to be run against
every target architecture variant (e.g., SSE4, AVX2, AVX512VL, etc). When run,
it filters the suite to only run tests that the current CPU can handle. */
class BoxesOverlapTest : public hwy::TestWithParamTarget {
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
  void SetUp() override {
    // Reset Drake's dispatcher, to be sure that we run all of the target
    // architectures.
    drake::internal::HwyDynamicReset();
    hwy::TestWithParamTarget::SetUp();
  }

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

// Instatiate the suite for all CPU targets (using the HWY macro).
HWY_TARGET_INSTANTIATE_TEST_SUITE_P(BoxesOverlapTest);

// Tests whether OBBs overlap. There are 15 unique possibilities for
// non-overlap. Therefore, there are 15 cases to test, each covering a different
// separating axis between the two bounding boxes. The first 3 cases use the
// axes of Frame A, the next 3 cases use the axes of Frame B, and the remaining
// 9 cases use the axes defined by the cross product of axes from Frame A and
// Frame B. We also test that it is robust for the case of parallel boxes.
TEST_P(BoxesOverlapTest, AllCases) {
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
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
