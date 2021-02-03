#include <benchmark/benchmark.h>

#include "drake/common/drake_assert.h"
#include "drake/geometry/proximity/obb.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;

struct TestData {
  TestData(const Obb& a_G_in, const Obb& b_H_in, const RigidTransformd& X_GH_in,
           const bool& expect_overlap_in)
      : a_G(a_G_in),
        b_H(b_H_in),
        X_GH(X_GH_in),
        expect_overlap(expect_overlap_in) {}

  Obb a_G;
  Obb b_H;
  RigidTransformd X_GH;
  // I can use this field to separate the overlapping cases from the
  // non-overlapping cases. The two set of inputs could have very different
  // performance. In general, non-overlapping cases will return earlier than
  // the overlapping cases.
  bool expect_overlap;
};

void Obb_HasOverlap(const std::vector<TestData>& test_data) {
  for (const TestData& t : test_data) {
    DRAKE_DEMAND(t.expect_overlap == Obb::HasOverlap(t.a_G, t.b_H, t.X_GH));
  }
}

// We want to compute X_AB such that B is posed relative to A as documented in
// TestObbOverlap. We can do so by generating the rotation component, R_AB, such
// that Bq has a minimum value along the chosen axis, and we can solve for
// the translation component, p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A.
RigidTransformd CalcCornerTransform(const Obb& a, const Obb& b, const int axis,
                                    const bool expect_overlap);

// We want to compute X_AB such that B is posed relative to A as documented
// in TestObbOverlap. We can do so by generating the rotation component, R_AB,
// such that Bq lies on the minimum edge along the chosen axis, and we can solve
// for the translation component, p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A.
RigidTransformd CalcEdgeTransform(const Obb& a, const Obb& b, const int a_axis,
                                  const int b_axis, const bool expect_overlap);


// TODO(DamrongGuoy): Explain why the return vector has 31 records.
//  It is 2 * 15 + 1.  The number 2 is true or false (overlap or not).
//  The number 15 is directions <=> Ax,Ay,Az, Bx,By,Bz, {Ax,Ay,Az}x{Bx,By,Bz}.
//  The number 1 is the single parallel case.

// Set up data for OBBs overlap. There are 15 cases to test, each covering a
// separating axis between the two bounding boxes. The first 3 cases use the
// axes of Frame A, the next 3 cases use the axes of Frame B, and the remaining
// 9 cases use the axes defined by the cross product of axes from Frame A and
// Frame B. We also test that it is robust for the case of parallel boxes.
std::vector<TestData> SetUp31Records() {
  std::vector<TestData> test_data;
  // Frame strategy. For the canonical frame A of box `a` and the
  // canonical frame B of box `b`, we want to control the pose X_AB between
  // the two boxes. However, we need to give HasOverlap() the pose X_GH
  // between the hierarchy frames G and H to which box `a` and box `b`
  // belong. Our strategy is to control the tests through X_AB and then compose
  // X_GH as:
  //                 X_GH = X_GA * X_AB * X_BH.
  // Frame A of box `a` is arbitrarily posed in the hierarchy frame G.
  const RigidTransformd X_GA{
      RotationMatrixd(RollPitchYawd(2. * M_PI / 3., M_PI_4, -M_PI / 3.)),
      Vector3d(1, 2, 3)};
  // Frame B of box `b` is arbitrarily posed in the hierarchy frame H.
  const RigidTransformd X_HB{
      RotationMatrixd(RollPitchYawd(M_PI_4, M_PI / 5., M_PI / 6.)),
      Vector3d(2, 0.5, 4)};
  const RigidTransformd X_BH = X_HB.inverse();
  // One box is fully contained in the other and they are parallel. We make
  // them parallel by setting X_AB to identity.
  RigidTransformd X_AB = RigidTransformd::Identity();
  Obb a(X_GA, Vector3d(1, 2, 1));
  Obb b(X_HB, Vector3d(0.5, 1, 0.5));
  test_data.emplace_back(a, b, X_GA * X_AB * X_BH /* X_GH */, true);
  // To cover the cases of the axes of Frame A, we need to pose box B along
  // each axis. For example, in the case of the Ax-axis, in a 2D view they would
  // look like:
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
  //                                Hy
  //                                ⇑
  // Gy             Gx              ⇑
  //   ⇖           ⇗                ⇑
  //     ⇖       ⇗                  ⇑
  //       ⇖   ⇗                    ⇑
  //         Go                     Ho ⇒ ⇒ ⇒ ⇒ ⇒ Hx
  //
  //
  // For this test, we define Point Bq as the minimum corner of the box B (i.e.,
  // center - half width). We want to pose box B so Bq is the uniquely closest
  // point to box A at a Point Af in the interior of +Ax face. The rest of
  // the box B extends farther along the +Ax axis (as suggested in the above
  // illustration). Point Bq will be a small epsilon away from the nearby face
  // either outside (if expect_overlap is false) or inside (if true).
  a = Obb(X_GA, Vector3d(2, 4, 3));
  b = Obb(X_HB, Vector3d(3.5, 2, 1.5));
  for (int axis = 0; axis < 3; ++axis) {
    X_AB = CalcCornerTransform(a, b, axis, false /* expect_overlap */);
    test_data.emplace_back(a, b, X_GA * X_AB * X_BH /* X_GH */, false);
    X_AB = CalcCornerTransform(a, b, axis, true /* expect_overlap */);
    test_data.emplace_back(a, b, X_GA * X_AB * X_BH /* X_GH */, true);
  }
  // To cover the local axes out of B, we can use the same method by swapping
  // the order of the boxes and then using the inverse of the transform.
  for (int axis = 0; axis < 3; ++axis) {
    X_AB =
        CalcCornerTransform(b, a, axis, false /* expect_overlap */).inverse();
    test_data.emplace_back(a, b, X_GA * X_AB * X_BH /* X_GH */, false);
    X_AB = CalcCornerTransform(b, a, axis, true /* expect_overlap */).inverse();
    test_data.emplace_back(a, b, X_GA * X_AB * X_BH /* X_GH */, true);
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
      test_data.emplace_back(a, b, X_GA * X_AB * X_BH  /* X_GH */, false);
      X_AB =
          CalcEdgeTransform(a, b, a_axis, b_axis, true /* expect_overlap */);
      // Separate along a's y-axis and b's x-axis.
      test_data.emplace_back(a, b, X_GA * X_AB * X_BH  /* X_GH */, true);
    }
  }

  return test_data;
}

RigidTransformd CalcCornerTransform(const Obb& a, const Obb& b, const int axis,
                                      const bool expect_overlap) {
  const int axis1 = (axis + 1) % 3;
  const int axis2 = (axis + 2) % 3;
  // Construct the rotation matrix, R_AB, that has meaningful (non-zero)
  // values everywhere for the remaining 2 axes and no symmetry.
  const RotationMatrixd R_AB =
      RotationMatrixd(AngleAxisd(M_PI / 5, Vector3d::Unit(axis1))) *
          RotationMatrixd(AngleAxisd(-M_PI / 5, Vector3d::Unit(axis2)));

  // We define p_BqBo in Frame A from box b's minimum corner Q to its center.
  const Vector3d p_BqBo_A = R_AB * b.half_width();
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
  Vector3d p_AoAf_A = a.half_width();
  p_AoAf_A[axis1] /= 2;
  p_AoAf_A[axis2] /= 2;

  const Vector3d p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A;
  return RigidTransformd(R_AB, p_AoBo_A);
}

// We want to compute X_AB such that B is posed relative to A as documented
// in TestObbOverlap. We can do so by generating the rotation component, R_AB,
// such that Bq lies on the minimum edge along the chosen axis, and we can solve
// for the translation component, p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A.
RigidTransformd CalcEdgeTransform(const Obb& a, const Obb& b, const int a_axis,
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
  Vector3d p_BqBo_B = b.half_width();
  p_BqBo_B[b_axis] -= b.half_width()[b_axis] / 2;
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
  Vector3d p_AoAf_A = a.half_width();
  p_AoAf_A[a_axis] -= a.half_width()[a_axis] / 2;

  Vector3d p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A;
  // Finally we combine the components to form the transform X_AB.
  return RigidTransformd(R_AB, p_AoBo_A);
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
static void BM_ObbHasOverlap_31_Cases(benchmark::State& state) {
  // Perform setup here
  std::vector<TestData> test_data = SetUp31Records();
  int num_data = test_data.size();
  DRAKE_DEMAND(num_data == 31);

  for (auto _ : state) {
    // This code gets timed
    Obb_HasOverlap(test_data);
  }
}
// Register the function as a benchmark
BENCHMARK(BM_ObbHasOverlap_31_Cases);

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
static void BM_ObbHasOverlap_16_OverlapCases(benchmark::State& state) {
  // Perform setup here
  std::vector<TestData> superset = SetUp31Records();
  std::vector<TestData> test_data;
  for (const TestData& t : superset)
    if (t.expect_overlap) test_data.push_back(t);
  int num_data = test_data.size();
  DRAKE_DEMAND(num_data == 16);

  for (auto _ : state) {
    // This code gets timed
    Obb_HasOverlap(test_data);
  }
}
// Register the function as a benchmark
BENCHMARK(BM_ObbHasOverlap_16_OverlapCases);

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
static void BM_ObbHasOverlap_15_NonOverlapCases(benchmark::State& state) {
  // Perform setup here
  std::vector<TestData> superset = SetUp31Records();
  std::vector<TestData> test_data;
  for (const TestData& t : superset)
    if (!t.expect_overlap) test_data.push_back(t);
  int num_data = test_data.size();
  DRAKE_DEMAND(num_data == 15);

  for (auto _ : state) {
    // This code gets timed
    Obb_HasOverlap(test_data);
  }
}
// Register the function as a benchmark
BENCHMARK(BM_ObbHasOverlap_15_NonOverlapCases);

}  // namespace internal
}  // namespace geometry
}  // namespace drake

BENCHMARK_MAIN();

