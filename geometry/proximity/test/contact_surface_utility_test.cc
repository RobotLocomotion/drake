#include "drake/geometry/proximity/contact_surface_utility.h"

#include <algorithm>
#include <limits>
#include <set>
#include <utility>
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
using math::RigidTransformd;
using math::RotationMatrixd;
using std::move;
using std::unique_ptr;
using std::vector;

// Tests the accumulation API (AddVertex() and AddPolygon()) and the
// construction API (MakeMeshAndField()).
//
// The following is explicitly tested.
//
//   a) The added vertex is stored and its corresponding field value likewise.
//      Note: this isn't validated *during* aggregation, but merely confirmed by
//      looking at the final, constructed mesh and field.
//   b) AddPolygon() for vertex indices does the "right" work.
//     - We call the appropriate "add polygon to mesh" function.
//       - in this case, for the defined polygon, we should get *five* new
//         vertices and *four* new triangles.
//     - The centroid has the correct pressure value stored.
//     - It *accumulates* face and vertex data across multiple calls.
//   c) MakeMeshAndField() produces the expected mesh and field.
//
//
// This behavior is orthogonal to scalar type, so, we'll do all of the tests
// with T = double.
//
// We'll add a polygon with four vertices lying on the Ny = 0 plane with a
// normal pointing in the -Ny direction. Three of the vertices (a, c, & d) are
// co-linear. Geometrically, the polygon is a triangle and its centroid is
// simply the average position of a, b, & c. By giving it *four* vertices, we
// force the centroid computation to perform area-weighted computation. This
// confirms that the normal is correctly used.
//
//           Nz
//            ┆
//            ┆ c
//            ┆  ●
//            ┆  │╲
//            ┆ d●  ╲
//            ┆  │    ╲
//            ┆  ●─────●
//            ┆  a      b
//         ┄┄┄●┄┄┄┄┄┄┄┄┄┄┄┄┄┄ Nx
//            ┆
GTEST_TEST(TriMeshBuilderTest, AddingFeatures) {
  // An arbitrary transform so that we're not using 0's and 1's.
  const RigidTransformd X_MN(
      AngleAxisd{M_PI / 7, Vector3d{1, 2, 3}.normalized()},
      Vector3d{-0.5, 1.25, 0.75});
  // The triangle defined in frame N so it's easy to reason about.
  const std::vector<Vector3d> polygon_N{
      {0.25, 0, 0.25}, {0.5, 0, 0.25}, {0.25, 0, 0.5}, {0.25, 0, 0.35}};
  const std::vector<Vector3d> polygon_M{X_MN * polygon_N[0],
                                        X_MN * polygon_N[1],
                                        X_MN * polygon_N[2],
                                        X_MN * polygon_N[3]};
  const Vector3d p_MC_expected =
      (polygon_M[0] + polygon_M[1] + polygon_M[2]) / 3;
  const Vector3d nhat_M = X_MN.rotation() * Vector3d(0, -1, 0);

  // To build the mesh, we just need to be able to evaluate *some* pressure
  // value over the polygon, and know the gradient of the function.
  const Vector3d grad_p_M{10, 11, 12};
  auto calc_pressure_in_M = [&grad_p_M](const Vector3d& p_MV) {
    return grad_p_M.dot(p_MV);
  };

  TriMeshBuilder<double> builder_M;

  // Confirm that in debug build, bad vertex indices throw.
#ifdef DRAKE_ASSERT_IS_ARMED
  EXPECT_THROW(
      builder_M.AddPolygon({0, 1, 2}, Vector3d::Zero(), Vector3d::Zero()),
      std::exception);
#endif

  // First register the vertices; we won't directly test the result here, but
  // examine the result in the final mesh and field below.
  std::vector<int> polygon_indices;
  std::vector<double> expected_pressures;
  for (int v = 0; v < static_cast<int>(polygon_M.size()); ++v) {
    const Vector3d& p_MV = polygon_M[v];
    expected_pressures.push_back(calc_pressure_in_M(p_MV));
    polygon_indices.push_back(
        builder_M.AddVertex(p_MV, expected_pressures.back()));
  }
  builder_M.AddPolygon(polygon_indices, nhat_M, grad_p_M);
  // Extra vertex introduces extra pressure value.
  expected_pressures.push_back(calc_pressure_in_M(p_MC_expected));

  // Confirm vertices and faces have accumulated.
  const int expected_face_count = 4;
  const int expected_vertex_count = 5;
  EXPECT_EQ(builder_M.num_vertices(), expected_vertex_count);
  EXPECT_EQ(builder_M.num_faces(), expected_face_count);

  // Adding with the same data should double vertex and face counts.
  polygon_indices.clear();
  for (int v = 0; v < static_cast<int>(polygon_M.size()); ++v) {
    const Vector3d& p_MV = polygon_M[v];
    expected_pressures.push_back(calc_pressure_in_M(p_MV));
    polygon_indices.push_back(
        builder_M.AddVertex(p_MV, expected_pressures.back()));
  }
  builder_M.AddPolygon(polygon_indices, nhat_M, grad_p_M);
  // Extra vertex introduces extra pressure value.
  expected_pressures.push_back(calc_pressure_in_M(p_MC_expected));

  EXPECT_EQ(builder_M.num_vertices(), 2 * expected_vertex_count);
  EXPECT_EQ(builder_M.num_faces(), 2 * expected_face_count);

  auto [mesh_M, surf_field_M] = builder_M.MakeMeshAndField();

  // The last vertex should be the centroid of the polygon.
  EXPECT_TRUE(CompareMatrices(mesh_M->vertices().back(), p_MC_expected, 1e-15));

  // Now we want to confirm that the pressure field has appropriate pressure
  // values -- including the centroid.
  ASSERT_EQ(mesh_M->num_vertices(),
            static_cast<int>(expected_pressures.size()));
  for (int v = 0; v < mesh_M->num_vertices(); ++v) {
    ASSERT_NEAR(surf_field_M->EvaluateAtVertex(v), expected_pressures[v],
                10 * std::numeric_limits<double>::epsilon());
    // NOTE: The TriMeshBuilder doesn't currently build a field with gradients,
    // so we won't test them.
  }

  // Confirm that the field is built on the mesh.
  EXPECT_EQ(mesh_M.get(), &surf_field_M->mesh());
}

// Same test as for TriMeshBuilderTest -- except no centroid is added. So, the
// testing logic directly related to the centroid is absent here.
GTEST_TEST(PolyMeshBuilderTest, AddingFeatures) {
  // An arbitrary transform so that we're not using 0's and 1's.
  const RigidTransformd X_MN(
      AngleAxisd{M_PI / 7, Vector3d{1, 2, 3}.normalized()},
      Vector3d{-0.5, 1.25, 0.75});
  // The polygon's defined in frame N so it's easy to reason about.
  const std::vector<Vector3d> polygon_N{
      {0.25, 0, 0.25}, {0.5, 0, 0.25}, {0.25, 0, 0.5}, {0.25, 0, 0.35}};
  const std::vector<Vector3d> polygon_M{X_MN * polygon_N[0],
                                        X_MN * polygon_N[1],
                                        X_MN * polygon_N[2],
                                        X_MN * polygon_N[3]};
  const Vector3d nhat_M = X_MN.rotation() * Vector3d(0, -1, 0);

  // To build the mesh, we just need to be able to evaluate *some* pressure
  // value over the polygon, and know the gradient of the function.
  const Vector3d grad_p_M{10, 11, 12};
  auto calc_pressure_in_M = [&grad_p_M](const Vector3d& p_MV) {
    return grad_p_M.dot(p_MV);
  };

  PolyMeshBuilder<double> builder_M;

  // Confirm that in debug build, bad vertex indices throw.
#ifdef DRAKE_ASSERT_IS_ARMED
  EXPECT_THROW(
      builder_M.AddPolygon({0, 1, 2}, Vector3d::Zero(), Vector3d::Zero()),
      std::exception);
#endif

  // First register the vertices; we won't directly test the result here, but
  // examine the result in the final mesh and field below.
  std::vector<int> polygon_indices;
  std::vector<double> expected_pressures;
  for (int v = 0; v < static_cast<int>(polygon_M.size()); ++v) {
    const Vector3d& p_MV = polygon_M[v];
    expected_pressures.push_back(calc_pressure_in_M(p_MV));
    polygon_indices.push_back(
        builder_M.AddVertex(p_MV, expected_pressures.back()));
  }
  builder_M.AddPolygon(polygon_indices, nhat_M, grad_p_M);

  // Confirm vertices and faces have accumulated.
  const int expected_face_count = 1;
  const int expected_vertex_count = 4;
  EXPECT_EQ(builder_M.num_vertices(), expected_vertex_count);
  EXPECT_EQ(builder_M.num_faces(), expected_face_count);

  // Adding with the same data should double vertex and face counts.
  polygon_indices.clear();
  for (int v = 0; v < static_cast<int>(polygon_M.size()); ++v) {
    const Vector3d& p_MV = polygon_M[v];
    expected_pressures.push_back(calc_pressure_in_M(p_MV));
    polygon_indices.push_back(
        builder_M.AddVertex(p_MV, expected_pressures.back()));
  }
  builder_M.AddPolygon(polygon_indices, nhat_M, grad_p_M);

  EXPECT_EQ(builder_M.num_vertices(), 2 * expected_vertex_count);
  EXPECT_EQ(builder_M.num_faces(), 2 * expected_face_count);

  auto [mesh_M, surf_field_M] = builder_M.MakeMeshAndField();

  // Now we want to confirm that the pressure field has appropriate pressure
  // values -- including the centroid.
  ASSERT_EQ(mesh_M->num_vertices(),
            static_cast<int>(expected_pressures.size()));
  for (int v = 0; v < mesh_M->num_vertices(); ++v) {
    ASSERT_NEAR(surf_field_M->EvaluateAtVertex(v), expected_pressures[v],
                10 * std::numeric_limits<double>::epsilon());
  }
  for (int p = 0; p < mesh_M->num_elements(); ++p) {
    ASSERT_TRUE(CompareMatrices(surf_field_M->EvaluateGradient(p), grad_p_M));
  }

  // Confirm that the field is built on the mesh.
  EXPECT_EQ(mesh_M.get(), &surf_field_M->mesh());
}

// Definition of a simple linear function f(x) = ∇f(x) + d for use in the
// EquivalentForceIntegration test below.
class LinearFunction {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearFunction);

  LinearFunction(const Vector3d& grad_F, double d)
      : grad_F_(grad_F), d_(d) {}

  double operator()(const Vector3d& p_FV) const {
    return grad_F_.dot(p_FV) + d_;
  }
  const Vector3d& gradient() const { return grad_F_; }

 private:
  Vector3d grad_F_;
  double d_{};
};

// Definition of a polygon (the vertices and the pressure field defined on the
// domain of the polygon) for the EquivalentForceIntegration test below.
class Polygon {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Polygon)

  Polygon(vector<Vector3d> vertices_P, const LinearFunction& func_P)
      : vertices_P_(move(vertices_P)), func_P_(func_P) {}

  const vector<Vector3d> vertices() const { return vertices_P_; }
  double EvalPressure(const Vector3d& p_PV) const { return func_P_(p_PV); }
  const Vector3d& pressure_gradient() const { return func_P_.gradient(); }
  Vector3d normal() const {
    const Vector3d p_AB = vertices_P_[1] - vertices_P_[0];
    const Vector3d p_AC = vertices_P_[2] - vertices_P_[0];
    return p_AB.cross(p_AC).normalized();
  }

 private:
  vector<Vector3d> vertices_P_;
  LinearFunction func_P_;
};

// The meshes we build (either polygon or triangle) should produce a mesh that
// integrates to the same result. So, we'll create a couple of arbitrary
// polygons (with equally arbitrary pressure functions) so that we can confirm
// that the resulting meshes and fields integrate to the same normal force.
GTEST_TEST(MeshBuilder, EquivalentForceIntegration) {
  vector<Polygon> polygons_M{
      {{Vector3d{0, 0, 0}, Vector3d{0, 1, 0}, Vector3d{0, 0, 1}},
       LinearFunction{Vector3d{1, 2, 3}, 3}},
      {{Vector3d{1, -1, 0}, Vector3d{1, 1, 0}, Vector3d{-1, 1, 0},
        Vector3d{-1, -1, 0}},
       LinearFunction{Vector3d{-1, 2, -3}, 2}}
       };

  TriMeshBuilder<double> tri_builder_M;
  PolyMeshBuilder<double> poly_builder_M;
  for (const auto& polygon_M : polygons_M) {
    vector<int> tri_indices;
    vector<int> poly_indices;
    for (const auto& p_MV : polygon_M.vertices()) {
      tri_indices.push_back(
          tri_builder_M.AddVertex(p_MV, polygon_M.EvalPressure(p_MV)));
      poly_indices.push_back(
          poly_builder_M.AddVertex(p_MV, polygon_M.EvalPressure(p_MV)));
    }
    const Vector3d nhat_M = polygon_M.normal();
    tri_builder_M.AddPolygon(tri_indices, nhat_M,
                             polygon_M.pressure_gradient());
    poly_builder_M.AddPolygon(poly_indices, nhat_M,
                              polygon_M.pressure_gradient());
  }

  auto [tri_mesh, tri_field] = tri_builder_M.MakeMeshAndField();
  auto [poly_mesh, poly_field] = poly_builder_M.MakeMeshAndField();

  // Integrate the pressure over each mesh. They should integrate to the same
  // force vector.
  auto calc_normal_force = [](const auto& mesh, const auto& field) {
    Vector3d force{0, 0, 0};
    for (int e = 0; e < mesh.num_elements(); ++e) {
      const Vector3d centroid = mesh.element_centroid(e);
      force += mesh.area(e) * field.EvaluateCartesian(e, centroid) *
               mesh.face_normal(e);
    }
    return force;
  };

  const Vector3d tri_force = calc_normal_force(*tri_mesh, *tri_field);
  const Vector3d poly_force = calc_normal_force(*poly_mesh, *poly_field);
  ASSERT_TRUE(CompareMatrices(tri_force, poly_force, 8e-16));
}

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
TEST_F(ContactSurfaceUtilityTest, AddPolygonToTriangleMeshData) {
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

  vector<SurfaceTriangle> faces;
  vector<Vector3d> vertices_M(vertices_source);
  const vector<int> quad{0, 1, 2, 3};
  const Vector3d nhat_M{0, 0, 1};
  AddPolygonToTriangleMeshData(quad, nhat_M, &faces, &vertices_M);

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

// This confirms that in adding a new polygon to existing mesh data:
//
//   1. The polygon is properly encoded
//      - The previous contents of the face data is unchanged.
//      - Four the 4-gon added, the last four entries should be
//          4, v0, v1, v2, v3
TEST_F(ContactSurfaceUtilityTest, AddPolygonToPolygonMeshData) {
  // For this test, we don't actually *need* vertices.

  // Pre-populate the data with recognizable garbage so we can confirm that the
  // new polygon strictly gets appended.
  vector<int> face_data{-1, -2, -3, -4};
  const vector<int> original_face_data(face_data);

  const vector<int> polygon{10, 20, 30, 40};
  AddPolygonToPolygonMeshData(polygon, &face_data);

  ASSERT_EQ(face_data.size(), original_face_data.size() + polygon.size() + 1);
  for (size_t i = 0; i < original_face_data.size(); ++i) {
    ASSERT_EQ(face_data[i], original_face_data[i]);
  }
  ASSERT_EQ(face_data[original_face_data.size()], 4);
  auto iter = face_data.begin() + (original_face_data.size());
  for (size_t i = 0; i < polygon.size(); ++i) {
    ASSERT_EQ(polygon[i], *(++iter));
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
