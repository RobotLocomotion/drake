#include "drake/geometry/render_gl/internal_shape_meshes.h"

#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using geometry::internal::RenderMesh;
using std::vector;

// Computes the normal to the indicated triangle whose magnitude is twice the
// triangle's area. I.e., for triangle (A, B, C), computes: (B - A) X (C - A).
Vector3d CalcTriNormal(const RenderMesh& data, int tri_index) {
  const auto& vertices = data.positions;
  const auto& tris = data.indices;
  const auto& a = vertices.row(tris(tri_index, 0));
  const auto& b = vertices.row(tris(tri_index, 1));
  const auto& c = vertices.row(tris(tri_index, 2));
  return (b - a).cross(c - a);
}

// Computes the area of the indicated triangle.
double CalcTriArea(const RenderMesh& data, int tri_index) {
  return CalcTriNormal(data, tri_index).norm() * 0.5;
}

// Computes the total area of the given triangles.
double CalcTotalArea(const RenderMesh& data) {
  double total_area = 0;
  for (int t = 0; t < data.indices.rows(); ++t) {
    const auto a = CalcTriArea(data, t);
    total_area += a;
  }
  return total_area;
}

// Computes the normal for the triangle (implied by its winding).
Vector3d CalcTriUnitNormal(const RenderMesh& data, int tri_index) {
  return CalcTriNormal(data, tri_index).normalized();
}

// Computes the centroid of the indicated triangle.
Vector3d CalcTriCentroid(const RenderMesh& data, int tri_index) {
  const auto& vertices = data.positions;
  const auto& tris = data.indices;
  return (vertices.row(tris(tri_index, 0)) + vertices.row(tris(tri_index, 1)) +
          vertices.row(tris(tri_index, 2))) /
         3.f;
}

// Simply confirm that the utility functions above report good values.
GTEST_TEST(PrimitiveMeshTests, ConfirmUtilities) {
  RenderMesh data;
  data.positions.resize(3, 3);
  /*
            │╲ (0, 1.5, 0)        Right isosceles triangle with legs of length
            │ ╲                   1.5, lying on the z = 0 plane.
            │  ╲                     area = 1.5 * 1.5 / 2 = 2.25 / 2 = 1.125
            │   ╲                    unit normal = <0, 0, 1>
            │    ╲                   normal = <0, 0, 2.25>
   (0, 0, 0)└─────  (1.5, 0, 0)      centroid = (0.5, 0.5, 0)
  */
  data.positions.row(0) << 0, 0, 0;
  data.positions.row(1) << 1.5, 0, 0;
  data.positions.row(2) << 0, 1.5, 0;
  data.indices.resize(1, 3);
  data.indices.row(0) << 0, 1, 2;
  EXPECT_EQ(CalcTriNormal(data, 0), Vector3d(0, 0, 2.25));
  EXPECT_EQ(CalcTriUnitNormal(data, 0), Vector3d(0, 0, 1));
  EXPECT_EQ(CalcTriArea(data, 0), 1.125);
  EXPECT_EQ(CalcTriCentroid(data, 0), Vector3d(0.5f, 0.5f, 0));
}

/* Defines a normal cone that spans the vertex normals of the triangle indicated
 by index `t`. The cone is defined by a unit-vector whose tail is at the apex
 of the cone, and points down the center line of the cone. The angle between the
 center line and the boundary of the cone is theta; we store cos_theta.

 This is used to validate the generated normals on a primitive. We have the
 expectation that a triangle normal should always lie within the minimal normal
 cone which cover that triangle's vertex normals.

      n0      nf    n1
         \    |    /
          \   |   /
           \__|__/

 In 2d, the triangles face normal is nf and its vertex normals are n0 and n1.
 There is a cone where n0 and n1 lie on the boundary of the cone. nf lies inside
 that cone. This will provide the definition of the face normal being
 "well aligned" with the vertex normals.

 Note: This is *not* used in testing the mesh loading; an OBJ can have arbitrary
 vertex normals and nothing can reasonably be asserted about them other than
 we read what was there.  */
class NormalCone {
 public:
  NormalCone(const RenderMesh& mesh_data, int t) {
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    const Vector3d n_0 =
        mesh_data.normals.row(mesh_data.indices(t, 0)).normalized();
    const Vector3d n_1 =
        mesh_data.normals.row(mesh_data.indices(t, 1)).normalized();
    const Vector3d n_2 =
        mesh_data.normals.row(mesh_data.indices(t, 2)).normalized();
    const Vector3d p_01 = n_1 - n_0;
    const Vector3d p_02 = n_2 - n_0;
    const Vector3d p_12 = n_2 - n_1;
    const double dist_01 = p_01.norm();
    const double dist_02 = p_02.norm();
    const double dist_12 = p_12.norm();
    if (dist_01 < kEps && dist_02 < kEps && dist_12 < kEps) {
      // The normals are all in the same direction. So, the cone's "ray" is any
      // of the normals.
      ray_ = n_0;
      cos_theta_ = 1.0 - kEps;
    } else if (dist_01 < kEps) {
      // Two normals have the identical direction; so we need a cone that simply
      // includes two unique vectors.
      ray_ = (n_0 + n_2).normalized();
      cos_theta_ = ray_.dot(n_0);
    } else if (dist_02 < kEps) {
      ray_ = (n_0 + n_1).normalized();
      cos_theta_ = ray_.dot(n_0);
    } else if (dist_12 < kEps) {
      ray_ = (n_1 + n_0).normalized();
      cos_theta_ = ray_.dot(n_1);
    } else {
      // Three "unique" normals. Ray is perpendicular to the plane spanned by
      // the end points of the vectors.
      ray_ = p_01.cross(p_02).normalized();
      cos_theta_ = ray_.dot(n_0);
      if (cos_theta_ < 0) {
        // We don't know the correct ordering; should it be p_01 x p_02 or
        // p_02 x p_01? Assuming all normals lie in the same hemisphere, the
        // dot product of ray_ with any of the vectors can't be negative. If it
        // is, we picked the wrong cross product and simply need to negate
        // everything.
        ray_ = -ray_;
        cos_theta_ = -cos_theta_;
      }
    }
  }

  /* Reports true if the given direction vector lies within `this` normal cone.
   */
  bool Contains(const Vector3d& dir) const {
    return ray_.dot(dir.normalized()) >= cos_theta_;
  }

  const Vector3d& ray() const { return ray_; }
  double cos_theta() const { return cos_theta_; }

 private:
  Vector3d ray_;
  double cos_theta_{};
};

// Confirms correctness of the normal cone.
GTEST_TEST(PrimitiveMeshTests, NormalCone) {
  constexpr double kEps = std::numeric_limits<double>::epsilon();

  // Note: due to floating point precision issues, normalizing the vector
  // <1, 2, 3> and <1, 2, 3> / sqrt(14) do *not* produce the same result.
  // So, we normalize <1, 2, 3> twice thus guaranteeing that when the
  // NormalCone normalizes again, it should be idempotent w.r.t. the normal
  // values.
  const Vector3d expected_ray = Vector3d(1, 2, 3).normalized().normalized();

  RenderMesh mesh_data;
  mesh_data.indices.resize(1, 3);
  mesh_data.indices << 0, 1, 2;
  mesh_data.normals.resize(3, 3);
  // Each case must set the normals explicitly.

  {
    // Case: all three normals point in the same direction.
    for (int i = 0; i < 3; ++i) {
      mesh_data.normals.row(i) = expected_ray;
    }
    const NormalCone cone(mesh_data, 0);
    // The ray should be an exact match and cos_theta should be epsilon less
    // than one.
    EXPECT_TRUE(CompareMatrices(cone.ray(), expected_ray));
    EXPECT_EQ(cone.cos_theta(), 1 - kEps);
    EXPECT_TRUE(cone.Contains(expected_ray));
    // A small perturbation is no longer inside.
    EXPECT_FALSE(cone.Contains(Vector3d(1.01, 1.99, 2.99).normalized()));
  }

  // We want to rotate a ray we know to lie on the boundary of the code a
  // "small" amount and have it lie *outside* the cone. Empirically, we find
  // that the value of 0.065 radians is the smallest value such that all of the
  // tests recognize the change from inside to outside. This is largely
  // attributable to the precision of 32-bit floats but hasn't been analyzed
  // carefully.
  const double kThetaEpsilon = 0.065f;

  {
    // Case: Two normals are identical; the third points in a different
    // direction. We'll create the two normals by rotating the expected ray
    // +/- theta around a vector perpendicular to that ray. This guarantees that
    // the minimal cone has angle theta.
    const double theta = M_PI / 7;
    const double cos_theta_expected = std::cos(theta);
    const AngleAxisd R(theta, Vector3d(-2, 1, 0).normalized());
    const Vector3d n0 = R * expected_ray;
    const Vector3d n1 = R.inverse() * expected_ray;
    ASSERT_NEAR(expected_ray.dot(n0), expected_ray.dot(n1), kEps);
    ASSERT_NEAR(n0.dot(n1), std::cos(2 * theta), kEps);
    // Exercise all three permutations for which normal is unique.
    for (int axis = 0; axis < 3; ++axis) {
      for (int i = 0; i < 3; ++i) {
        if (i == axis) {
          mesh_data.normals.row(i) = n1;
        } else {
          mesh_data.normals.row(i) = n0;
        }
      }
      const NormalCone cone(mesh_data, 0);
      // The ray and cos_theta should match within epsilon.
      EXPECT_TRUE(CompareMatrices(cone.ray(), expected_ray, kEps));
      EXPECT_NEAR(cone.cos_theta(), cos_theta_expected, kEps);
      // The two normals should lie on the boundary of the cone.
      EXPECT_NEAR(cone.ray().dot(n0), cone.cos_theta(), kEps);
      EXPECT_NEAR(cone.ray().dot(n1), cone.cos_theta(), kEps);
      const AngleAxisd R_outside(theta + kThetaEpsilon,
                                 Vector3d(-2, -1, 0).normalized());
      EXPECT_FALSE(cone.Contains(R_outside * expected_ray));
    }
  }

  {
    // Case: The normals are all different such that their end points span a
    // plane. In order to get the expected_ray back out of the cone, I need
    // to make sure that the three normals are defined such that their end
    // points are evenly spaced on a circle.
    const double theta = M_PI / 7;
    const double cos_theta_expected = std::cos(theta);
    const Vector3d n_base =
        AngleAxisd(theta, Vector3d(-2, 1, 0).normalized()) * expected_ray;
    for (int i = 0; i < 3; ++i) {
      const AngleAxisd R(2 * M_PI / 3 * i, expected_ray);
      mesh_data.normals.row(i) = R * n_base;
    }
    const NormalCone cone(mesh_data, 0);
    // The ray and cos_theta should match within epsilon.
    EXPECT_TRUE(CompareMatrices(cone.ray(), expected_ray, kEps));
    EXPECT_NEAR(cone.cos_theta(), cos_theta_expected, kEps);
    // The three normals should lie on the boundary of the cone.
    for (int i = 0; i < 3; ++i) {
      const Vector3d n = mesh_data.normals.row(i);
      EXPECT_NEAR(cone.ray().dot(n), cone.cos_theta(), 2 * kEps);
    }
    const AngleAxisd R_outside(theta + kThetaEpsilon,
                               Vector3d(-2, -1, 0).normalized());
    EXPECT_FALSE(cone.Contains(R_outside * expected_ray));
  }
}

/* Testing utility to test various characteristics of the primitive shapes. As
 they are all convex and defined centered on their local origin, there are
 general tests that we can assert for all of them (sharing the logic), relying
 on the shape-specific tests to evaluate shape-specified invariants. This tests
 the following invariants:

   - Confirm that the number of normals, positions, and uvs all match.
   - The winding of the faces always point *away* from the origin.
   - The face normals lie within the corresponding face's normal cones.
   - All normals have unit length.
   - Confirm all uvs lie in the range [0, 1].

 @param mesh              The mesh data to test.
 @param origin_is_inside  Reports if we have the expectation that the origin is
                          "inside" the mesh.
 */
void TestGenericPrimitiveTraits(const RenderMesh& mesh,
                                bool origin_is_inside = true) {
  const double kEps = std::numeric_limits<double>::epsilon();

  ASSERT_EQ(mesh.positions.rows(), mesh.normals.rows());
  ASSERT_EQ(mesh.positions.rows(), mesh.uvs.rows());

  // All normals have unit length.
  for (int n_i = 0; n_i < mesh.normals.rows(); ++n_i) {
    const Vector3d n = mesh.normals.row(n_i);
    ASSERT_NEAR(n.norm(), 1, kEps);
  }

  // Face normals point outward, within the face's normal cone.
  for (int t = 0; t < mesh.indices.rows(); ++t) {
    Vector3d n_face = CalcTriNormal(mesh, t);
    if (origin_is_inside) {
      Vector3d c = CalcTriCentroid(mesh, t);
      // If the winding were backwards, this dot product would be negative.
      ASSERT_GT(n_face.dot(c), 0) << "for triangle " << t;
    }
    const NormalCone cone(mesh, t);
    ASSERT_TRUE(cone.Contains(n_face)) << fmt::format(
        "for triangle {}\n  face normal: {}", t, fmt_eigen(n_face.transpose()));
  }

  // UVs lie in the expected range.
  ASSERT_EQ(mesh.uvs.rows(), mesh.positions.rows());
  for (int i = 0; i < mesh.uvs.rows(); ++i) {
    const double u = mesh.uvs(i, 0);
    const double v = mesh.uvs(i, 1);
    ASSERT_TRUE((0 <= u) && (u <= 1));
    ASSERT_TRUE((0 <= v) && (v <= 1));
  }
}

/* The tests for these tessellated primitives are merely suggestive; they don't
 guarantee correct meshes. We *might* consider confirming there are no holes in
 the mesh. In practice that may not be necessary; those kinds of artifacts would
 be immediately apparent in rendered images. Defer those heavyweight types of
 tests until there's a proven bug. */

GTEST_TEST(PrimitiveMeshTests, MakeLongLatUnitSphere) {
  const double kEps = std::numeric_limits<double>::epsilon();

  // Closed form solution for surface area of unit sphere.
  const double kIdealArea = 4 * M_PI;  // 4πR², R = 1.

  double prev_area = 0;
  for (int resolution : {3, 10, 20, 40}) {
    SCOPED_TRACE(fmt::format("Sphere with resolution {}", resolution));
    const RenderMesh mesh_data = MakeLongLatUnitSphere(resolution, resolution);

    // Confirm area converges towards (but not above) ideal area.
    double curr_area = CalcTotalArea(mesh_data);
    EXPECT_GT(curr_area, prev_area);
    EXPECT_LE(curr_area, kIdealArea);
    prev_area = curr_area;

    // All vertices lie on the unit sphere and that position is equal to the
    // reported normal (and its magnitude is 1).
    for (int v = 0; v < mesh_data.positions.rows(); ++v) {
      const Vector3d v_i = mesh_data.positions.row(v);
      EXPECT_NEAR(v_i.norm(), 1.f, kEps) << "for vertex " << v;
      const Vector3d n_i = mesh_data.normals.row(v);
      EXPECT_TRUE(CompareMatrices(v_i, n_i, kEps));
      EXPECT_NEAR(n_i.norm(), 1, kEps);
    }

    TestGenericPrimitiveTraits(mesh_data);

    // TestGenericPrimitiveTraits has confirmed all UVs coordinates lie in the
    // range [0, 1]. Writing a test to validate the texture coordinates
    // exhaustively would require essentially duplicating the uv-generation
    // code. Instead, we'll rely on a coarse sampling and users complaining
    // about bad looking renderings to detect more subtle bugs in the
    // texture coordinates.
    // We'll confirm the north and south poles (first and last vertices) are
    // where we expect them.
    EXPECT_TRUE(
        CompareMatrices(mesh_data.uvs.row(0), Vector2d(0, 1).transpose()));
    const int row_count = mesh_data.uvs.rows();
    EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(row_count - 1),
                                Vector2d(0, 0).transpose()));
  }
}

GTEST_TEST(PrimitiveMeshTests, MakeUnitCylinder) {
  const double kEps = std::numeric_limits<double>::epsilon();

  // Closed form solution for surface area of unit cylinder: H = 1, R = 1.
  //  Total cap area: 2 * πR² = 2π
  //  Barrel area: 2πRH = 2π
  //  Total area = 2π + 2π = 4π
  const double kIdealArea = 4 * M_PI;

  double prev_area = 0;
  for (int resolution : {3, 10, 20, 40}) {
    SCOPED_TRACE(fmt::format("Cylinder with resolution {}", resolution));
    const RenderMesh mesh_data = MakeUnitCylinder(resolution, resolution);

    // Confirm area converges towards (but not above) ideal area.
    double curr_area = CalcTotalArea(mesh_data);
    EXPECT_GT(curr_area, prev_area);
    EXPECT_LE(curr_area, kIdealArea);
    prev_area = curr_area;

    // All vertices (except first and last) are 1 unit away from z-axis.
    for (int v_i = 1; v_i < mesh_data.positions.rows() - 1; ++v_i) {
      Vector3d v = mesh_data.positions.row(v_i);
      v(2) = 0;
      EXPECT_NEAR(v.norm(), 1.f, kEps) << "for vertex " << v_i;
    }
    // First and last vertices are *on* the z axis.
    for (int v_i : {0, static_cast<int>(mesh_data.positions.rows() - 1)}) {
      Vector3d v = mesh_data.positions.row(v_i);
      v(2) = 0;
      EXPECT_NEAR(v.norm(), 0.f, kEps) << "for vertex " << v_i;
    }

    // The first resolution + 1 vertices should be at z = 0.5. The last
    // resolution + 1 mesh_data.positions should be at z = -0.5.
    for (int v_i = 0; v_i < resolution + 1; ++v_i) {
      EXPECT_EQ(mesh_data.positions(v_i, 2), 0.5);
    }
    for (int v_i = mesh_data.positions.rows() - resolution - 1;
         v_i < mesh_data.positions.rows(); ++v_i) {
      EXPECT_EQ(mesh_data.positions(v_i, 2), -0.5);
    }

    TestGenericPrimitiveTraits(mesh_data);

    // See note in MakeLongLatUnitSphere about testing texture coordinates.
    EXPECT_TRUE(
        CompareMatrices(mesh_data.uvs.row(0), Vector2d(0, 1).transpose()));
    const int row_count = mesh_data.uvs.rows();
    EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(row_count - 1),
                                Vector2d(0, 0).transpose()));
  }
}

GTEST_TEST(PrimitiveMeshTests, MakeSquarePatch) {
  const double kEps = std::numeric_limits<double>::epsilon();
  const double kMax = std::numeric_limits<double>::max();
  const double kMeasure = 25;
  const double kArea = kMeasure * kMeasure;

  for (int resolution : {1, 4, 15}) {
    SCOPED_TRACE(fmt::format("Square patch with resolution {}", resolution));
    const RenderMesh mesh_data = MakeSquarePatch(kMeasure, resolution);

    TestGenericPrimitiveTraits(mesh_data, false /* origin_is_inside */);

    // The tolerance we select is going to be a scaled machine epsilon. The
    // first scale factor is the actual expected area; this makes our tolerance
    // a _relative_ tolerance. There is a second contribution to error:
    // the sum of many small real values each contribute round-off error. The
    // more we sum, the more round off error we introduce. We scale by the
    // number of triangles (representative of the number of small additions).
    const double area_epsilon = kEps * kArea * resolution * resolution;
    EXPECT_NEAR(CalcTotalArea(mesh_data), kArea, area_epsilon);
    EXPECT_EQ(mesh_data.positions.rows(), (resolution + 1) * (resolution + 1));
    EXPECT_EQ(mesh_data.indices.rows(), 2 * resolution * resolution);
    for (int t = 0; t < mesh_data.indices.rows(); ++t) {
      EXPECT_TRUE(CompareMatrices(CalcTriUnitNormal(mesh_data, t),
                                  Vector3d(0, 0, 1), kEps));
    }

    // Confirm the bounding box is what we would expected.
    Vector3d min_pt{kMax, kMax, kMax};
    Vector3d max_pt = -min_pt;
    for (int v = 0; v < mesh_data.positions.rows(); ++v) {
      Vector3d vertex = mesh_data.positions.row(v);
      min_pt = min_pt.cwiseMin(vertex);
      max_pt = max_pt.cwiseMax(vertex);
    }
    const Vector3d corner = Vector3d(0.5f, 0.5f, 0.0) * kMeasure;
    EXPECT_TRUE(CompareMatrices(min_pt, -corner, kEps));
    EXPECT_TRUE(CompareMatrices(max_pt, corner, 8 * kEps));

    const auto expected_n = Vector3d::UnitZ().transpose();
    for (int v = 0; v < mesh_data.positions.rows(); ++v) {
      ASSERT_TRUE(CompareMatrices(mesh_data.normals.row(v), expected_n));
    }

    // Coarse sampling of UVs. The first vertex should be at (0, 0) the last
    // at (1, 1). See note on texture coordinate testing in
    // MakeLongLatUnitSphere.
    EXPECT_TRUE(
        CompareMatrices(mesh_data.uvs.row(0), Vector2d(0, 0).transpose()));
    const int row_count = mesh_data.uvs.rows();
    EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(row_count - 1),
                                Vector2d(1, 1).transpose()));
  }
}

GTEST_TEST(PrimitiveMeshTests, MakeUnitBox) {
  const double kEps = std::numeric_limits<double>::epsilon();
  const double kMax = std::numeric_limits<double>::max();

  SCOPED_TRACE("Box");
  const RenderMesh mesh_data = MakeUnitBox();

  // In computing total area, we scale epsilon by the expected area and also
  // the number of triangles, as per-triangle area gets magnified.
  EXPECT_NEAR(CalcTotalArea(mesh_data), 6.f, kEps);
  EXPECT_EQ(mesh_data.positions.rows(), 24);
  EXPECT_EQ(mesh_data.normals.rows(), 24);
  EXPECT_EQ(mesh_data.indices.rows(), 12);

  // Confirm the bounding box is what we would expected.
  Vector3d min_pt{kMax, kMax, kMax};
  Vector3d max_pt = -min_pt;
  for (int v = 0; v < mesh_data.positions.rows(); ++v) {
    Vector3d vertex = mesh_data.positions.row(v);
    min_pt = min_pt.cwiseMin(vertex);
    max_pt = max_pt.cwiseMax(vertex);
  }
  const Vector3d corner{0.5f, 0.5f, 0.5};
  EXPECT_TRUE(CompareMatrices(min_pt, -corner, kEps));
  EXPECT_TRUE(CompareMatrices(max_pt, corner, kEps));

  TestGenericPrimitiveTraits(mesh_data);

  // We expect the UVs to have the pattern (0, 0) -> (1, 0) -> (1, 1) -> (1, 0)
  // once for each face. We'll test that explicitly.
  for (int row = 0; row < mesh_data.uvs.rows(); row += 4) {
    ASSERT_TRUE(
        CompareMatrices(mesh_data.uvs.row(row), Vector2d(0, 0).transpose()));
    ASSERT_TRUE(CompareMatrices(mesh_data.uvs.row(row + 1),
                                Vector2d(1, 0).transpose()));
    ASSERT_TRUE(CompareMatrices(mesh_data.uvs.row(row + 2),
                                Vector2d(1, 1).transpose()));
    ASSERT_TRUE(CompareMatrices(mesh_data.uvs.row(row + 3),
                                Vector2d(0, 1).transpose()));
  }
}

GTEST_TEST(PrimitiveMeshTests, MakeCapsule) {
  const double kEps = std::numeric_limits<double>::epsilon();
  constexpr double kRadius = 0.75;
  constexpr double kLength = 2.5;
  for (int resolution : {3, 10, 25}) {
    SCOPED_TRACE(fmt::format("Capsule with resolution {}", resolution));
    const RenderMesh mesh_data = MakeCapsule(resolution, kRadius, kLength);

    // The first half of vertices and normals belong to a hemisphere with center
    // at (0, 0, kLength / 2). Confirm that the vertex positions lie on the
    // sphere and the normals lie in the same direction as from sphere center
    // to vertex. Repeat with the second half, with a sphere center at
    // (0, 0, -kLength / 2).
    const int num_vertices = mesh_data.positions.rows();
    {
      const Vector3d p_MC(0, 0, kLength / 2);
      for (int v = 0; v < num_vertices / 2; ++v) {
        const Vector3d p_MV = mesh_data.positions.row(v);
        const Vector3d p_CV = p_MV - p_MC;
        ASSERT_NEAR(p_CV.norm(), kRadius, kEps);
        const Vector3d nhat_V_expected = p_CV.normalized();
        const Vector3d nhat_V = mesh_data.normals.row(v);
        ASSERT_TRUE(CompareMatrices(nhat_V, nhat_V_expected, kEps));
      }
    }
    {
      const Vector3d p_MC(0, 0, -kLength / 2);
      for (int v = num_vertices / 2; v < num_vertices; ++v) {
        const Vector3d p_MV = mesh_data.positions.row(v);
        const Vector3d p_CV = p_MV - p_MC;
        ASSERT_NEAR(p_CV.norm(), kRadius, kEps);
        const Vector3d nhat_V_expected = p_CV.normalized();
        const Vector3d nhat_V = mesh_data.normals.row(v);
        ASSERT_TRUE(CompareMatrices(nhat_V, nhat_V_expected, kEps));
      }
    }

    TestGenericPrimitiveTraits(mesh_data);

    // See note in MakeLongLatUnitSphere about testing texture coordinates.
    EXPECT_TRUE(
        CompareMatrices(mesh_data.uvs.row(0), Vector2d(0, 1).transpose()));
    const int row_count = mesh_data.uvs.rows();
    EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(row_count - 1),
                                Vector2d(0, 0).transpose()));
    // TODO(SeanCurtis-TRI) Consider testing the v-values of the two equators
    //  to make sure the distribution from 0-1 is arc-length parameterized.
  }
}

}  // namespace
}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
