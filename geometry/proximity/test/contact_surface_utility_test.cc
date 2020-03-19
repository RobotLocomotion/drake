#include "drake/geometry/proximity/contact_surface_utility.h"

#include <algorithm>
#include <limits>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RotationMatrixd;
using math::RigidTransformd;
using std::vector;

class ContactSurfaceUtilityTest : public ::testing::Test {
 protected:
  void SetUp() {
    X_FMs_.push_back(RigidTransformd::Identity());
    X_FMs_.push_back(RigidTransformd{Vector3d{10.5, -13.25, 7.75}});
    X_FMs_.push_back(
        RigidTransformd{AngleAxisd{0.001, Vector3d{1, 2, 3}.normalized()},
                        Vector3d{-0.5, 1.25, 0.75}});
    X_FMs_.push_back(RigidTransformd{
        AngleAxisd{-9 * M_PI / 7, Vector3d{1, 2, 3}.normalized()},
        Vector3d{0.5, 0.75, -1.25}});
  }

  // Compute the average vertex position of the given `polygon`.
  static Vector3d AverageVertex(
      const vector<SurfaceVertexIndex>& polygon,
      const vector<SurfaceVertex<double>>& vertices_F) {
    Vector3d v = Vector3d::Zero();
    for (const auto& i : polygon) {
      v += vertices_F[i].r_MV();
    }
    return v / polygon.size();
  }

  // A number of transforms so that we can test the robustness of arbitrary
  // positions and orientations.
  vector<RigidTransformd> X_FMs_;
};

// This first test validates the computation of a polygon centroid. There
// are specific cases in which the centroid is simply the average vertex
// position. They are:
//
//   - polygon is a triangle
//   - polygon is radially symmetric.
//   - polygon has duplicate vertices such that the _unique_ set is simply a
//     triangle.
//   - polygon that has _very_ short edges, such that the long edges define a
//     triangle.
//
// This test confirms all of those cases, and then _pokes_ at the case where the
// centroid _isn't_ the average vertex position as a representative sample.
//
// In frame M, we construct a set of vertex locations on the z = 0 plane in
// Frame M. This will make the Mz vector normal to the triangle. We construct
// various polygons from the set of vertices with corresponding expected
// outcomes.
//
//                                   y
//
//                                   ^
//                                   │
//                                   │               v4
//                                   │
//                                   │
//                                   v2 v5      v3
//                                   │
//                                   │
//                                   │
//                                   │
//    ─────────v7────────────────────v0─────────v1────────────> x
//                         v6        │
//                                   │
//
// Note: v5 is just epsilon away from v2.
// Note: the triangle formed by v7, v0, v2 has area 1 as does the quad formed
// by v0, v1, v3, v2.
//
// For the following geometries, the centroid is defined as indicated:
//
//  - well-formed triangle: v6, v1, v2 -> average vertex.
//  - radially symmetric quad: v0, v1, v3, v2 -> average vertex.
//  - quad duplicate vertices: v0, v1, v1, v2 -> triangle average vertex.
//  - quad with epsilon edge: v6, v1, v5, v2 -> triangle average vertex (within
//    epsilon).
//  - asymmetric quad: v6, v1, v4, v2 -> _not_ average vertex.
//
// We then transform these vertices into various frames to make sure that it
// works with rotation matrices that have no zero-values.
TEST_F(ContactSurfaceUtilityTest, PolygonCentroidTest) {
  using V = SurfaceVertex<double>;
  using VIndex = SurfaceVertexIndex;

  const double kEps = std::numeric_limits<double>::epsilon();

  // Create a number of vertices in mesh frame M. All vertices lie on the Mz = 0
  // plane (such that Mz is a good normal direction). We'll construct polygons
  // on these vertices.
  const vector<V> vertices_M{V{Vector3d{0, 0, 0}}, V{Vector3d{1, 0, 0}},
                       V{Vector3d{0, 1, 0}}, V{Vector3d{1, 1, 0}},
                       V{Vector3d{1.5, 1.5, 0}}, V{Vector3d{kEps, 1, 0}},
                       V{Vector3d{-1, -0.25, 0}}, V{Vector3d{-2, 0, 0}}};

  for (const auto& X_FM : X_FMs_) {
    // Now put the vertices in an arbitrary frame F.
    vector<V> vertices_F;
    std::transform(vertices_M.begin(), vertices_M.end(),
                   std::back_inserter(vertices_F),
                   [&X_FM](const V& v_M) -> V { return V{X_FM * v_M.r_MV()}; });
    const Vector3d& nhat_F = X_FM.rotation().matrix().col(2);

    {
      // Well-formed triangle; it's simply the average vertex location.
      const vector<VIndex> triangle{VIndex{6}, VIndex{1}, VIndex{2}};
      const Vector3d p_FC = CalcPolygonCentroid(triangle, nhat_F, vertices_F);
      const Vector3d p_FC_expected = AverageVertex(triangle, vertices_F);
      EXPECT_TRUE(CompareMatrices(p_FC, p_FC_expected));
    }
    {
      // Radially symmetric quad; the centroid is simply the average vertex
      // position.
      const vector<VIndex> quad{VIndex{0}, VIndex{1}, VIndex{3}, VIndex{2}};
      const Vector3d p_FC = CalcPolygonCentroid(quad, nhat_F, vertices_F);
      const Vector3d p_FC_expected = AverageVertex(quad, vertices_F);
      EXPECT_TRUE(CompareMatrices(p_FC, p_FC_expected, 2 * kEps));
    }
    {
      // Quad with a duplicate vertex; Centroid is the same as for the
      // corresponding triangle.
      const vector<VIndex> quad{VIndex{0}, VIndex{1}, VIndex{1}, VIndex{2}};
      const vector<VIndex> triangle{VIndex{0}, VIndex{1}, VIndex{2}};
      const Vector3d p_FC = CalcPolygonCentroid(quad, nhat_F, vertices_F);
      const Vector3d p_FC_expected = AverageVertex(triangle, vertices_F);
      EXPECT_TRUE(CompareMatrices(p_FC, p_FC_expected));
    }
    {
      // Quad with a negligible edge; Centroid is _almost_ the same as for the
      // corresponding triangle.
      const vector<VIndex> quad{VIndex{6}, VIndex{1}, VIndex{5}, VIndex{2}};
      const vector<VIndex> triangle{VIndex{6}, VIndex{1}, VIndex{2}};
      const Vector3d p_FC = CalcPolygonCentroid(quad, nhat_F, vertices_F);
      const Vector3d p_FC_expected = AverageVertex(triangle, vertices_F);
      EXPECT_TRUE(CompareMatrices(p_FC, p_FC_expected, kEps));
    }
    {
      // Make an asymmetric quad from v7, v1, v3, v2. This can be decomposed
      // into a triangle and a square with equal areas. So, we can compute each
      // centroid by averaging their vertex positions and simply average them to
      // get the expected centroid.
      const vector<VIndex> quad{VIndex{7}, VIndex{1}, VIndex{3}, VIndex{2}};
      const vector<VIndex> triangle{VIndex{7}, VIndex{0}, VIndex{2}};
      const vector<VIndex> square{VIndex{0}, VIndex{1}, VIndex{3}, VIndex{2}};

      const Vector3d p_FCt = AverageVertex(triangle, vertices_F);
      const Vector3d p_FCs = AverageVertex(square, vertices_F);
      const Vector3d p_FC_expected = (p_FCt + p_FCs) / 2;

      const Vector3d p_FC = CalcPolygonCentroid(quad, nhat_F, vertices_F);
      EXPECT_TRUE(CompareMatrices(p_FC, p_FC_expected, 10 * kEps));
    }
  }
}

// The centroid for a zero-area polygon is problematic; the area-weighted
// calculation means that the weights are zero and the total weight is zero.
// This would give us a centroid consisting of NaNs. This confirms that the
// value is:
//   a) _not_ NaN
//   b) the average vertex position.
//
// Note: this test is done without any particular frame noted because the frame
// is irrelevant. The vertex positions imply a frame and the centroid is
// computed in that same frame.
TEST_F(ContactSurfaceUtilityTest, ZeroAreaPolygon) {
  using V = SurfaceVertex<double>;
  using VIndex = SurfaceVertexIndex;
  constexpr double kEps = 4 * std::numeric_limits<double>::epsilon();

  // We'll use two primitives from two points:
  // Triangle: (a, a, b)
  // Quad: (a, a, b, b)
  const vector<V> vertices{V{Vector3d{0.25, 1.5, -3}},
                           V{Vector3d{-0.75, 0.25, 1.25}}};
  const VIndex a{0};
  const VIndex b{1};

  const Vector3d p_AB = vertices[b].r_MV() - vertices[a].r_MV();
  const Vector3d normal = Vector3d{0, p_AB(2), -p_AB(1)}.normalized();

  {
    const vector<VIndex> triangle{a, a, b};
    const Vector3d centroid = CalcPolygonCentroid(triangle, normal, vertices);
    const Vector3d expected_centroid =
        (2 * vertices[a].r_MV() + vertices[b].r_MV()) / 3.0;
    EXPECT_TRUE(CompareMatrices(centroid, expected_centroid, kEps));
  }

  {
    const vector<VIndex> quad{a, a, b, b};
    const Vector3d centroid = CalcPolygonCentroid(quad, normal, vertices);
    const Vector3d expected_centroid =
        (2 * vertices[a].r_MV() + 2 * vertices[b].r_MV()) / 4;
    EXPECT_TRUE(CompareMatrices(centroid, expected_centroid, kEps));
  }
}

// This confirms that with assertions armed, non-planar polygons are detected.
TEST_F(ContactSurfaceUtilityTest, NonPlanarPolygon) {
  using V = SurfaceVertex<double>;
  using VIndex = SurfaceVertexIndex;
  constexpr double kEps = 11 * std::numeric_limits<double>::epsilon();
  if (kDrakeAssertIsArmed) {
    const Vector3d Mz = Vector3d::UnitZ();
    const vector<V> vertices_M{
        V{Vector3d{-1.5, -0.25, kEps}}, V{Vector3d{1, 0, -kEps}},
        V{Vector3d{0.75, 1.25, -kEps}}, V{Vector3d{0, 1, -kEps}}};
    const vector<VIndex> quad{VIndex(0), VIndex(1), VIndex(2), VIndex(3)};
    DRAKE_EXPECT_THROWS_MESSAGE(
        CalcPolygonCentroid(quad, Mz, vertices_M), std::runtime_error,
        "CalcPolygonCentroid: input polygon is not planar");
  }
}

// This confirms that with assertions armed, normal perpendicularity is ignored
// for degenerate polygons (lines and points).
TEST_F(ContactSurfaceUtilityTest, EvalNormalWithDegeneratePolygon) {
  using V = SurfaceVertex<double>;
  using VIndex = SurfaceVertexIndex;

  if (kDrakeAssertIsArmed) {
    const vector<V> vertices_M{
        V{Vector3d{-1.5, -0.25, 0}}, V{Vector3d{1, 0, 0}},
        V{Vector3d{0.75, 1.25, 0}}, V{Vector3d{0, 1, 0}}};
    const Vector3d My = Vector3d::UnitY();

    {
      // Base case: polygon is _not_ degenerate, but the normal is parallel
      // with the polygon's plane; it throws.
      const vector<VIndex> quad{VIndex(0), VIndex(1), VIndex(2), VIndex(3)};
      DRAKE_EXPECT_THROWS_MESSAGE(
          CalcPolygonCentroid(quad, My, vertices_M), std::runtime_error,
          "CalcPolygonCentroid: the given normal is not perpendicular to the "
          "polygon's plane.*");
    }

    // In this case, we're just confirming that the parallel normal doesn't
    // throw with assert armed -- because the degeneracy is more important. The
    // actual behavior of what it does with assert _unarmed_ is handled in the
    // PolygonCentroidTest_NormalUse test.
    {
      // Case: same normal, subset of the vertices span a line.
      const vector<VIndex> quad{VIndex(0), VIndex(1), VIndex(1), VIndex(0)};
      EXPECT_NO_THROW(CalcPolygonCentroid(quad, My, vertices_M));
    }

    {
      // Case: same normal, subset of the vertices span a point.
      const vector<VIndex> quad{VIndex(0), VIndex(0), VIndex(0), VIndex(0)};
      EXPECT_NO_THROW(CalcPolygonCentroid(quad, My, vertices_M));
    }
  }
}

// This confirms several invariants on the input normal.
//
//   1. The scale of the normal doesn't particularly matter.
//   2. The "normalness" doesn't really matter that much
//   3. A zero vector destroys the answer.
TEST_F(ContactSurfaceUtilityTest, PolygonCentroidTest_NormalUse) {
  using V = SurfaceVertex<double>;
  using VIndex = SurfaceVertexIndex;

  const double kEps = std::numeric_limits<double>::epsilon();

  // Vertices sufficient to support a well-defined quad.
  const vector<V> vertices_M{V{Vector3d{-1.5, -0.25, 0}}, V{Vector3d{1, 0, 0}},
                             V{Vector3d{0.75, 1.25, 0}}, V{Vector3d{0, 1, 0}}};
  // A quad formed with a duplicate index; this forces the function to perform
  // full area calculations, but it should still have the expected value of
  // the corresponding triangle.
  const vector<VIndex> pseudo_triangle{VIndex{0}, VIndex{1}, VIndex{1},
                                       VIndex{2}};
  const Vector3d p_MC_expected = AverageVertex(
      vector<VIndex>{VIndex{0}, VIndex{1}, VIndex{2}}, vertices_M);

  {
    // A literal zero normal vector.
    const vector<VIndex> quad{VIndex{0}, VIndex{1}, VIndex{2}, VIndex{3}};
    Vector3d kZeroVec = Vector3d::Zero();
    if (kDrakeAssertIsArmed) {
      // With assertions armed; we throw.
      DRAKE_EXPECT_THROWS_MESSAGE(
          CalcPolygonCentroid(quad, kZeroVec, vertices_M), std::runtime_error,
          "CalcPolygonCentroid: given normal is too small; .*");
    } else {
      // Without assertions, we compute the average vertex position.
      const Vector3d expected_centroid =
          (vertices_M[quad[0]].r_MV() + vertices_M[quad[1]].r_MV() +
           vertices_M[quad[2]].r_MV() + vertices_M[quad[3]].r_MV()) /
          4;
      EXPECT_TRUE(CompareMatrices(
          CalcPolygonCentroid(quad, kZeroVec, vertices_M), expected_centroid));
    }
  }

  {
    // A close-to-zero normal vector.
    const vector<VIndex> quad{VIndex{0}, VIndex{1}, VIndex{2}, VIndex{3}};
    Vector3d kZeroVec{kEps, kEps, kEps};
    if (kDrakeAssertIsArmed) {
      // With assertions armed; we throw.
      DRAKE_EXPECT_THROWS_MESSAGE(
          CalcPolygonCentroid(quad, kZeroVec, vertices_M), std::runtime_error,
          "CalcPolygonCentroid: given normal is too small; .*");
    } else {
      // Without assertions, we compute the triangle's centroid.
      EXPECT_TRUE(CompareMatrices(
          CalcPolygonCentroid(pseudo_triangle, kZeroVec, vertices_M),
          p_MC_expected));
    }
  }

  {
    // A "normal" parallel to the polygon's plane is bad.
    const Vector3d Fy{0, 1, 0};
    if (kDrakeAssertIsArmed) {
      // With assertions armed; we throw.
      DRAKE_EXPECT_THROWS_MESSAGE(
          CalcPolygonCentroid(pseudo_triangle, Fy, vertices_M),
          std::runtime_error,
          "CalcPolygonCentroid: the given normal is not perpendicular to the "
          "polygon's plane.*");
    } else {
      // Without assertions, we compute the average vertex position.
      // Note: Generally, it will be difficult produce a normal that is exactly
      // parallel with the plane such that all of the dot products are exactly
      // zero. In those cases, there will be non-zero weights and _a_ centroid
      // will be computed. This tests the magical condition of exactly zero.
      const Vector3d expected_centroid =
          (vertices_M[pseudo_triangle[0]].r_MV() +
           vertices_M[pseudo_triangle[1]].r_MV() +
           vertices_M[pseudo_triangle[2]].r_MV() +
           vertices_M[pseudo_triangle[3]].r_MV()) /
          4;
      EXPECT_TRUE(
          CompareMatrices(CalcPolygonCentroid(pseudo_triangle, Fy, vertices_M),
                          expected_centroid));
    }
  }

  for (const auto& X_FM : X_FMs_) {
    vector<V> vertices_F;
    std::transform(vertices_M.begin(), vertices_M.end(),
                   std::back_inserter(vertices_F),
                   [&X_FM](const V& v_M) -> V { return V{X_FM * v_M.r_MV()}; });
    const Vector3d p_FC_expected = X_FM * p_MC_expected;
    const Vector3d& Mz_F = X_FM.rotation().matrix().col(2);

    if (kDrakeAssertIsDisarmed) {
      // A "normal" that is too long or too short still produce the right
      // centroid with assertions disarmed.
      EXPECT_TRUE(CompareMatrices(
          CalcPolygonCentroid<double>(pseudo_triangle, 2 * Mz_F, vertices_F),
          p_FC_expected, kEps));
      EXPECT_TRUE(CompareMatrices(
          CalcPolygonCentroid<double>(pseudo_triangle, 0.5 * Mz_F, vertices_F),
          p_FC_expected, kEps));
    }

    // A "normal" that isn't perpendicular (and isn't parallel) works.
    const Vector3d My_F = X_FM.rotation().matrix().col(1);
    const Vector3d not_normal_F = (My_F + 2 * Mz_F).normalized();
    // Larger epsilon reflects loss of precision as the "normal" becomes
    // less and less normal.
    EXPECT_TRUE(CompareMatrices(CalcPolygonCentroid<double>(
                                    pseudo_triangle, not_normal_F, vertices_F),
                                p_FC_expected, 10 * kEps));
  }
}

// This confirms that in adding a new polygon to existing mesh data:
//
//   1. Only a single new vertex is added.
//   2. That new vertex is at the centroid of the polygon.
//   3. For N-sided polygon, N faces are added to the set of faces.
//   4. Each of the triangles have winding that produce a normal in the same
//      direction as the input polygons.
TEST_F(ContactSurfaceUtilityTest, AddPolygonToMeshData) {
  using V = SurfaceVertex<double>;
  using VIndex = SurfaceVertexIndex;

  // Vertices sufficient to support a well-defined quad.
  //
  //             y
  //
  //             │   o
  //             o    v2
  //           v3│
  //             │
  //             │
  // ────────────┼────o───────── x
  //             │     v1
  //       o     │
  //       v0    │
  //             │
  const vector<V> vertices_source{
      V{Vector3d{-1.5, -0.25, 0}}, V{Vector3d{1, 0, 0}},
      V{Vector3d{0.75, 1.25, 0}}, V{Vector3d{0, 1, 0}}};

  vector<SurfaceFace> faces;
  vector<V> vertices_M(vertices_source);
  const vector<VIndex> quad{VIndex{0}, VIndex{1}, VIndex{2}, VIndex{3}};
  const Vector3d nhat_M{0, 0, 1};
  AddPolygonToMeshData(quad, nhat_M, &faces, &vertices_M);

  auto triangle_normal = [](VIndex v0, VIndex v1, VIndex v2,
                            const vector<V>& vertices_F) -> Vector3d {
    const Vector3d phat_V0V1_F =
        (vertices_F[v1].r_MV() - vertices_F[v0].r_MV()).normalized();
    const Vector3d phat_V0V2_F =
        (vertices_F[v2].r_MV() - vertices_F[v0].r_MV()).normalized();
    return phat_V0V1_F.cross(phat_V0V2_F).normalized();
  };

  // By construction, the source polygon
  const Vector3d poly_normal_M{0, 0, 1};

  ASSERT_EQ(vertices_M.size(), vertices_source.size() + 1);
  ASSERT_TRUE(
      CompareMatrices(vertices_M.back().r_MV(),
                      CalcPolygonCentroid(quad, nhat_M, vertices_M)));
  const VIndex centroid_index(vertices_M.size() - 1);

  ASSERT_EQ(faces.size(), quad.size());
  std::set<VIndex> quad_verts(quad.begin(), quad.end());
  for (const auto& face : faces) {
    const std::set<VIndex> verts{face.vertex(0), face.vertex(1),
                                 face.vertex(2)};
    EXPECT_EQ(verts.size(), 3);  // No duplicate vertex indices.
    EXPECT_NE(verts.find(centroid_index), verts.end());  // Includes centroid.
    const Vector3d tri_normal_M = triangle_normal(
        face.vertex(0), face.vertex(1), face.vertex(2), vertices_M);
    EXPECT_NEAR(tri_normal_M.dot(poly_normal_M), 1,
                std::numeric_limits<double>::epsilon());
    for (VIndex i(0); i < 3; ++i) {
      if (i == centroid_index) continue;
      // The other two vertices come from the quad.
      EXPECT_NE(quad_verts.find(i), quad_verts.end());
    }
  }
  // Note: technically, if the quad were: (v0, v1, v2, v3),
  // AddPolgyonToMeshData() could return (v0, v1, v4) four times and this test
  // would pass. We can consider making the test more robust to exclude this
  // possibility but are currently implicitly relying on the idea that such an
  // egregious error would be obvious during visualization.
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
