#include "drake/geometry/proximity/contact_surface_utility.h"

#include <algorithm>
#include <limits>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff.h"
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
  static Vector3d AverageVertex(const vector<int>& polygon,
                                const vector<Vector3d>& vertices_F) {
    Vector3d v = Vector3d::Zero();
    for (const auto& i : polygon) {
      v += vertices_F[i];
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
  const double kEps = std::numeric_limits<double>::epsilon();

  // Create a number of vertices in mesh frame M. All vertices lie on the Mz = 0
  // plane (such that Mz is a good normal direction). We'll construct polygons
  // on these vertices.
  const vector<Vector3d> vertices_M{
      Vector3d{0, 0, 0},      Vector3d{1, 0, 0},     Vector3d{0, 1, 0},
      Vector3d{1, 1, 0},      Vector3d{1.5, 1.5, 0}, Vector3d{kEps, 1, 0},
      Vector3d{-1, -0.25, 0}, Vector3d{-2, 0, 0}};

  for (const auto& X_FM : X_FMs_) {
    // Now put the vertices in an arbitrary frame F.
    vector<Vector3d> vertices_F;
    std::transform(
        vertices_M.begin(), vertices_M.end(), std::back_inserter(vertices_F),
        [&X_FM](const Vector3d& v_M) -> Vector3d { return X_FM * v_M; });
    const Vector3d& nhat_F = X_FM.rotation().matrix().col(2);

    {
      // Well-formed triangle; it's simply the average vertex location.
      const vector<int> triangle{6, 1, 2};
      const Vector3d p_FC = CalcPolygonCentroid(triangle, nhat_F, vertices_F);
      const Vector3d p_FC_expected = AverageVertex(triangle, vertices_F);
      EXPECT_TRUE(CompareMatrices(p_FC, p_FC_expected));
    }
    {
      // Radially symmetric quad; the centroid is simply the average vertex
      // position.
      const vector<int> quad{0, 1, 3, 2};
      const Vector3d p_FC = CalcPolygonCentroid(quad, nhat_F, vertices_F);
      const Vector3d p_FC_expected = AverageVertex(quad, vertices_F);
      EXPECT_TRUE(CompareMatrices(p_FC, p_FC_expected, 2 * kEps));
    }
    {
      // Quad with a duplicate vertex; Centroid is the same as for the
      // corresponding triangle.
      const vector<int> quad{0, 1, 1, 2};
      const vector<int> triangle{0, 1, 2};
      const Vector3d p_FC = CalcPolygonCentroid(quad, nhat_F, vertices_F);
      const Vector3d p_FC_expected = AverageVertex(triangle, vertices_F);
      EXPECT_TRUE(CompareMatrices(p_FC, p_FC_expected));
    }
    {
      // Quad with a negligible edge; Centroid is _almost_ the same as for the
      // corresponding triangle.
      const vector<int> quad{6, 1, 5, 2};
      const vector<int> triangle{6, 1, 2};
      const Vector3d p_FC = CalcPolygonCentroid(quad, nhat_F, vertices_F);
      const Vector3d p_FC_expected = AverageVertex(triangle, vertices_F);
      EXPECT_TRUE(CompareMatrices(p_FC, p_FC_expected, kEps));
    }
    {
      // Make an asymmetric quad from v7, v1, v3, v2. This can be decomposed
      // into a triangle and a square with equal areas. So, we can compute each
      // centroid by averaging their vertex positions and simply average them to
      // get the expected centroid.
      const vector<int> quad{7, 1, 3, 2};
      const vector<int> triangle{7, 0, 2};
      const vector<int> square{0, 1, 3, 2};

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
  constexpr double kEps = 4 * std::numeric_limits<double>::epsilon();

  // We'll use two primitives from two points:
  // Triangle: (a, a, b)
  // Quad: (a, a, b, b)
  const vector<Vector3d> vertices{Vector3d{0.25, 1.5, -3},
                                  Vector3d{-0.75, 0.25, 1.25}};
  const int a{0};
  const int b{1};

  const Vector3d p_AB = vertices[b] - vertices[a];
  const Vector3d normal = Vector3d{0, p_AB(2), -p_AB(1)}.normalized();

  {
    const vector<int> triangle{a, a, b};
    const Vector3d centroid = CalcPolygonCentroid(triangle, normal, vertices);
    const Vector3d expected_centroid = (2 * vertices[a] + vertices[b]) / 3.0;
    EXPECT_TRUE(CompareMatrices(centroid, expected_centroid, kEps));
  }

  {
    const vector<int> quad{a, a, b, b};
    const Vector3d centroid = CalcPolygonCentroid(quad, normal, vertices);
    const Vector3d expected_centroid = (2 * vertices[a] + 2 * vertices[b]) / 4;
    EXPECT_TRUE(CompareMatrices(centroid, expected_centroid, kEps));
  }
}

// This confirms that with assertions armed, non-planar polygons are detected.
TEST_F(ContactSurfaceUtilityTest, NonPlanarPolygon) {
  constexpr double kEps = 11 * std::numeric_limits<double>::epsilon();
  if (kDrakeAssertIsArmed) {
    const Vector3d Mz = Vector3d::UnitZ();
    const vector<Vector3d> vertices_M{
        Vector3d{-1.5, -0.25, kEps}, Vector3d{1, 0, -kEps},
        Vector3d{0.75, 1.25, -kEps}, Vector3d{0, 1, -kEps}};
    const vector<int> quad{0, 1, 2, 3};
    DRAKE_EXPECT_THROWS_MESSAGE(
        CalcPolygonCentroid(quad, Mz, vertices_M), std::runtime_error,
        "CalcPolygonCentroid: input polygon is not planar");
  }
}

// This confirms that with assertions armed, normal perpendicularity is ignored
// for degenerate polygons (lines and points).
TEST_F(ContactSurfaceUtilityTest, EvalNormalWithDegeneratePolygon) {
  if (kDrakeAssertIsArmed) {
    const vector<Vector3d> vertices_M{
        Vector3d{-1.5, -0.25, 0}, Vector3d{1, 0, 0},
        Vector3d{0.75, 1.25, 0}, Vector3d{0, 1, 0}};
    const Vector3d My = Vector3d::UnitY();

    {
      // Base case: polygon is _not_ degenerate, but the normal is parallel
      // with the polygon's plane; it throws.
      const vector<int> quad{0, 1, 2, 3};
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
      const vector<int> quad{0, 1, 1, 0};
      EXPECT_NO_THROW(CalcPolygonCentroid(quad, My, vertices_M));
    }

    {
      // Case: same normal, subset of the vertices span a point.
      const vector<int> quad{0, 0, 0, 0};
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
  const double kEps = std::numeric_limits<double>::epsilon();

  // Vertices sufficient to support a well-defined quad.
  const vector<Vector3d> vertices_M{Vector3d{-1.5, -0.25, 0}, Vector3d{1, 0, 0},
                                    Vector3d{0.75, 1.25, 0}, Vector3d{0, 1, 0}};
  // A quad formed with a duplicate index; this forces the function to perform
  // full area calculations, but it should still have the expected value of
  // the corresponding triangle.
  const vector<int> pseudo_triangle{0, 1, 1, 2};
  const Vector3d p_MC_expected =
      AverageVertex(vector<int>{0, 1, 2}, vertices_M);

  {
    // A literal zero normal vector.
    const vector<int> quad{0, 1, 2, 3};
    Vector3d kZeroVec = Vector3d::Zero();
    if (kDrakeAssertIsArmed) {
      // With assertions armed; we throw.
      DRAKE_EXPECT_THROWS_MESSAGE(
          CalcPolygonCentroid(quad, kZeroVec, vertices_M), std::runtime_error,
          "CalcPolygonCentroid: given normal is too small; .*");
    } else {
      // Without assertions, we compute the average vertex position.
      const Vector3d expected_centroid =
          (vertices_M[quad[0]] + vertices_M[quad[1]] + vertices_M[quad[2]] +
           vertices_M[quad[3]]) /
          4;
      EXPECT_TRUE(CompareMatrices(
          CalcPolygonCentroid(quad, kZeroVec, vertices_M), expected_centroid));
    }
  }

  {
    // A close-to-zero normal vector.
    const vector<int> quad{0, 1, 2, 3};
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
          (vertices_M[pseudo_triangle[0]] + vertices_M[pseudo_triangle[1]] +
           vertices_M[pseudo_triangle[2]] + vertices_M[pseudo_triangle[3]]) /
          4;
      EXPECT_TRUE(
          CompareMatrices(CalcPolygonCentroid(pseudo_triangle, Fy, vertices_M),
                          expected_centroid));
    }
  }

  for (const auto& X_FM : X_FMs_) {
    vector<Vector3d> vertices_F;
    std::transform(
        vertices_M.begin(), vertices_M.end(), std::back_inserter(vertices_F),
        [&X_FM](const Vector3d& v_M) -> Vector3d { return X_FM * v_M; });
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
  const vector<Vector3d> vertices_source{
      Vector3d{-1.5, -0.25, 0}, Vector3d{1, 0, 0},
      Vector3d{0.75, 1.25, 0}, Vector3d{0, 1, 0}};

  vector<SurfaceFace> faces;
  vector<Vector3d> vertices_M(vertices_source);
  const vector<int> quad{0, 1, 2, 3};
  const Vector3d nhat_M{0, 0, 1};
  AddPolygonToMeshData(quad, nhat_M, &faces, &vertices_M);

  auto triangle_normal = [](int v0, int v1, int v2,
                            const vector<Vector3d>& vertices_F) -> Vector3d {
    const Vector3d phat_V0V1_F = (vertices_F[v1] - vertices_F[v0]).normalized();
    const Vector3d phat_V0V2_F = (vertices_F[v2] - vertices_F[v0]).normalized();
    return phat_V0V1_F.cross(phat_V0V2_F).normalized();
  };

  // By construction, the source polygon
  const Vector3d poly_normal_M{0, 0, 1};

  ASSERT_EQ(vertices_M.size(), vertices_source.size() + 1);
  ASSERT_TRUE(
      CompareMatrices(vertices_M.back(),
                      CalcPolygonCentroid(quad, nhat_M, vertices_M)));
  const int centroid_index = static_cast<int>(vertices_M.size()) - 1;

  ASSERT_EQ(faces.size(), quad.size());
  std::set<int> quad_verts(quad.begin(), quad.end());
  for (const auto& face : faces) {
    const std::set<int> verts{face.vertex(0), face.vertex(1), face.vertex(2)};
    EXPECT_EQ(verts.size(), 3);  // No duplicate vertex indices.
    EXPECT_NE(verts.find(centroid_index), verts.end());  // Includes centroid.
    const Vector3d tri_normal_M = triangle_normal(
        face.vertex(0), face.vertex(1), face.vertex(2), vertices_M);
    EXPECT_NEAR(tri_normal_M.dot(poly_normal_M), 1,
                std::numeric_limits<double>::epsilon());
    for (int i = 0; i < 3; ++i) {
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

// Verifies that a representative triangle of a polygon satisfies these
// properties:
//
//   1. The triangle has the same area as the polygon.
//   2. The triangle has the same centroid as the polygon.
//   3. The triangle has winding that produces the same normal as the polygon.
//
// @param[in] triangle_index
//     Index into `faces` of the representative triangle.
// @param[in] faces
//     Set of faces that contain the representative triangle.
// @param[in] vertices_F
//     Set of vertices of the `faces`, whose positional vectors are expressed
//     in frame F.
// @param[in] polygon_F
//     The polygon is represented by position vectors of its vertices,
//     measured and expressed in frame F.
// @param[in] nhat_F
//     The unit normal vector of the polygon, expressed in frame F. We assume
//     the winding of the polygon vertices is consistent with this normal
//     vector.
void VerifyRepresentativeTriangle(int triangle_index,
                                  const vector<SurfaceFace>& faces,
                                  const vector<Vector3d>& vertices_F,
                                  const vector<Vector3d>& polygon_F,
                                  const Vector3d& nhat_F) {
  const SurfaceFace& face = faces.at(triangle_index);
  const Vector3d& p_FV0 = vertices_F.at(face.vertex(0));
  const Vector3d& p_FV1 = vertices_F.at(face.vertex(1));
  const Vector3d& p_FV2 = vertices_F.at(face.vertex(2));
  const vector<Vector3d> triangle_F{p_FV0, p_FV1, p_FV2};

  const double triangle_area = CalcPolygonArea(triangle_F, nhat_F);
  const double polygon_area = CalcPolygonArea(polygon_F, nhat_F);
  const double kEps = std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(triangle_area, polygon_area,
              8. * kEps * std::max(polygon_area, 1.));

  const Vector3d triangle_normal_F =
      (p_FV1 - p_FV0).cross(p_FV2 - p_FV0).normalized();
  EXPECT_TRUE(CompareMatrices(triangle_normal_F, nhat_F, 16. * kEps));

  // Triangle's centroid Cₜ.
  const Vector3d p_FCt = (p_FV0 + p_FV1 + p_FV2) / 3.;
  // Polygon's centroid Cₚ.
  const Vector3d p_FCp = CalcPolygonCentroid(polygon_F, nhat_F);
  EXPECT_TRUE(
      CompareMatrices(p_FCt, p_FCp, 4. * kEps * std::max(p_FCp.norm(), 1.)));
}

// Tests AddPolygonToMeshDataAsOneTriangle() on a sequence of polygons. We
// will use VerifyRepresentativeTriangle() to verify the properties of each
// added representative triangle.
GTEST_TEST(ContactSurfaceUtility, AddPolygonToMeshDataAsOneTriangle) {
  // The robust test will be done in a general frame F, but the set up will be
  // in a convenient frame M.
  const RigidTransformd X_FM{
      AngleAxisd{-9 * M_PI / 7, Vector3d{1, 2, 3}.normalized()},
      Vector3d{0.5, 0.75, -1.25}};

  vector<Vector3d> polygons_F[2];
  Vector3d nhats_F[2];
  {
    const vector<Vector3d> polygon_M[2]{
        // A triangle is a special case of a polygon.
        {Vector3d::Zero(), 2. * Vector3d::UnitX(), 3. * Vector3d::UnitY()},

        // A general convex quadrilateral.
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
        {Vector3d{-1.5, -0.25, 0}, Vector3d{1, 0, 0}, Vector3d{0.75, 1.25, 0},
         Vector3d{0, 1, 0}}};
    for (int i = 0; i < 2; ++i) {
      nhats_F[i] = X_FM.rotation().col(2);
      std::transform(
          polygon_M[i].begin(), polygon_M[i].end(),
          std::back_inserter(polygons_F[i]),
          [&X_FM](const Vector3d& p_MV) -> Vector3d { return X_FM * p_MV; });
    }
  }

  vector<SurfaceFace> faces;
  vector<Vector3d> vertices_F;
  for (int i = 0; i < 2; ++i) {
    AddPolygonToMeshDataAsOneTriangle<double>(polygons_F[i], nhats_F[i], &faces,
                                              &vertices_F);
    EXPECT_EQ(faces.size(), i + 1);
    EXPECT_EQ(vertices_F.size(), 3 * (i + 1));
    VerifyRepresentativeTriangle(i, faces, vertices_F, polygons_F[i],
                                 nhats_F[i]);
  }
}

// Tests AddPolygonToMeshDataAsOneTriangle() for the special case of a
// polygon that is already a triangle, i.e. a 3-gon. It should add that
// triangle exactly without calculating a different representative triangle.
GTEST_TEST(ContactSurfaceUtility, AddPolygonToMeshDataAsOneTriangle_3Gon) {
  // A triangle is a special case of a polygon. Values of the coordinates
  // are less relevant to this test.
  const vector<Vector3d> polygon_F{Vector3d::Zero(), 2 * Vector3d::UnitX(),
                                   3 * Vector3d::UnitY()};
  const Vector3d nhat_F = Vector3d::UnitZ();

  vector<SurfaceFace> faces;
  vector<Vector3d> vertices_F;
  AddPolygonToMeshDataAsOneTriangle<double>(polygon_F, nhat_F, &faces,
                                            &vertices_F);
  ASSERT_EQ(faces.size(), 1);
  EXPECT_EQ(faces[0].vertex(0), 0);
  EXPECT_EQ(faces[0].vertex(1), 1);
  EXPECT_EQ(faces[0].vertex(2), 2);

  ASSERT_EQ(vertices_F.size(), 3);
  EXPECT_EQ(vertices_F[0], polygon_F[0]);
  EXPECT_EQ(vertices_F[1], polygon_F[1]);
  EXPECT_EQ(vertices_F[2], polygon_F[2]);
}

// Tests AddPolygonToMeshDataAsOneTriangle() with AutoDiffXd. It's only a
// smoke test. We only check that it compiles, and derivatives propagate to a
// coordinate of a vertex.
GTEST_TEST(ContactSurfaceUtility, AddPolygonToMeshDataAsOneTriangle_AutoDiff) {
  using Vector3ad = Vector3<AutoDiffXd>;
  const vector<Vector3ad> polygon_F{math::InitializeAutoDiff(Vector3d::Zero()),
                                    Vector3ad::UnitX(), Vector3ad{1, 1, 0},
                                    Vector3ad::UnitY()};
  const Vector3ad nhat_F = Vector3ad::UnitZ();

  vector<SurfaceFace> faces;
  vector<Vector3<AutoDiffXd>> vertices_F;
  AddPolygonToMeshDataAsOneTriangle<AutoDiffXd>(polygon_F, nhat_F, &faces,
                                            &vertices_F);
  EXPECT_EQ(vertices_F[0][0].derivatives().size(), 3);
}

// Tests AddPolygonToMeshDataAsOneTriangle() for a special case of an input
// polygon with near-zero area. It should skip such a polygon.
GTEST_TEST(ContactSurfaceUtility, AddPolygonToMeshDataAsOneTriangle_ZeroArea) {
  // A near-zero-area polygon is skipped.
  {
    const double l = 0.1 * sqrt(kMinimumPolygonArea);
    const vector<Vector3d> polygon_F{Vector3d::Zero(), l * Vector3d::UnitX(),
                                     l * Vector3d{1, 1, 0},
                                     l * Vector3d::UnitY()};
    const Vector3d nhat_F = Vector3d::UnitZ();

    vector<SurfaceFace> faces;
    vector<Vector3d> vertices_F;
    AddPolygonToMeshDataAsOneTriangle<double>(polygon_F, nhat_F, &faces,
                                              &vertices_F);
    EXPECT_EQ(faces.size(), 0);
    EXPECT_EQ(vertices_F.size(), 0);
  }
  // This polygon is above the threshold, so it's not skipped.
  {
    const double l = 10 * sqrt(kMinimumPolygonArea);
    const vector<Vector3d> polygon_F{Vector3d::Zero(), l * Vector3d::UnitX(),
                                     l * Vector3d{1, 1, 0},
                                     l * Vector3d::UnitY()};
    const Vector3d nhat_F = Vector3d::UnitZ();

    vector<SurfaceFace> faces;
    vector<Vector3d> vertices_F;
    AddPolygonToMeshDataAsOneTriangle<double>(polygon_F, nhat_F, &faces,
                                              &vertices_F);
    // We only check that its representative triangle was added, but we do
    // not verify the triangle itself. Checking correctness of representative
    // triangles was done in another test.
    EXPECT_EQ(faces.size(), 1);
    EXPECT_EQ(vertices_F.size(), 3);
  }
}

// Tests the overload of CalcPolygonCentroid() that takes a polygon represented
// as an ordered list of positional vectors. We only perform a simple test here
// and assume its correctness follows from the extensive tests of the other
// version.
GTEST_TEST(ContactSurfaceUtility, CalcPolygonCentroidFromVertexPositions) {
  const vector<Vector3d> p_FVs{Vector3d::UnitX(), Vector3d::UnitY(),
                               Vector3d::UnitZ()};
  const Vector3d n_F = Vector3d(1., 1., 1.);
  const Vector3d p_FC = CalcPolygonCentroid<double>(p_FVs, n_F);
  const Vector3d p_FC_expected(1. / 3., 1. / 3., 1. / 3.);
  const double kEps = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(p_FC, p_FC_expected, 10. * kEps));
}

// Tests calculation of area of a polygon. We use a polygon with some
// collinear vertices to test for robustness.
GTEST_TEST(ContactSurfaceUtility, CalcPolygonArea) {
  // We will use a general frame F for robustness, but the set up will be in
  // frame M for convenience.
  const RigidTransformd X_FM{
      AngleAxisd{-9 * M_PI / 7, Vector3d{1, 2, 3}.normalized()},
      Vector3d{0.5, 0.75, -1.25}};

  vector<Vector3d> p_FVs;
  Vector3d nhat_F;
  {
    // This pentagon V₀V₁V₂V₃V₄ has three collinear vertices. Its sub-triangle
    // V₀V₁V₂ has zero area.
    //
    //       My
    //       |                    ● V₃(3,2,0)
    //       │
    //       │
    //    V₄ ●
    //       |
    //       │
    //    V₀ ●──────●──────●────── Mx
    //              V₁     V₂
    const vector<Vector3d> p_MVs{Vector3d::Zero(), Vector3d::UnitX(),
                                 2. * Vector3d::UnitX(), Vector3d{3., 2., 0.},
                                 Vector3d::UnitY()};

    nhat_F = X_FM.rotation().col(2);
    std::transform(
        p_MVs.begin(), p_MVs.end(), std::back_inserter(p_FVs),
        [&X_FM](const Vector3d& p_MV) -> Vector3d { return X_FM * p_MV; });
  }

  const double area = CalcPolygonArea(p_FVs, nhat_F);
  // The pentagon V₀V₁V₂V₃V₄ has the same area as the quadrilateral V₀V₂V₃V₄.
  const double kExpectedArea = 3.5;
  const double kEps = std::numeric_limits<double>::epsilon();
  const double kTolerance = 4. * kEps * kExpectedArea;
  EXPECT_NEAR(area, kExpectedArea, kTolerance);
}

// Tests a corner case for calculating area of a polygon with AutoDiffXd when
// the polygon has zero area.
GTEST_TEST(ContactSurfaceUtility, CalcPolygonArea_AutoDiffWithZeroArea) {
  using Vector3ad = Vector3<AutoDiffXd>;
  //
  //    Fy
  //    ^
  //    |        The face normal nhat_F is perpendicular to the screen
  //    |        in Fz direction.
  //    |
  //    +---●---●---●---> Fx
  //        V₀  V₁  V₂
  //
  // The triangle has zero area because its three vertices are collinear. We
  // will use AutoDiff to calculate the area derivative with respect to
  // the position of V₀.
  //
  // Notice that we InitializeAutoDiff() for V₀ only, so we will expect
  // derivatives.size() == 3 for the three coordinates of V₀.
  const vector<Vector3ad> p_FVs{math::InitializeAutoDiff(Vector3d::UnitX()),
                                Vector3ad::UnitX() * 2, Vector3ad::UnitX() * 3};
  const Vector3ad nhat_F = Vector3ad::UnitZ();

  const AutoDiffXd area = CalcPolygonArea(p_FVs, nhat_F);
  EXPECT_EQ(area.value(), 0.);

  ASSERT_EQ(area.derivatives().size(), 3u);
  // Confirms that none of the three derivatives are NaN. In fact, the following
  // expected derivatives confirm that moving V₀ in Fy direction will increase
  // area the most. This is because the triangle is flat along Fx, and we fix
  // the unit face normal nhat_F in Fz direction. Moving V₀ along Fx direction
  // will keep the triangle flat. Moving V₀ along Fz direction will increase
  // the triangle's area, but its projection along nhat_F is still flat.
  const Vector3d kExpectDerivatives{0, 0.5, 0};
  EXPECT_TRUE(CompareMatrices(area.derivatives(), kExpectDerivatives));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
