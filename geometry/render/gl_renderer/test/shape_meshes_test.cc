#include "drake/geometry/render/gl_renderer/shape_meshes.h"

#include <limits>
#include <sstream>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {
namespace {

using Eigen::AngleAxisf;
using Vector2f = Vector2<GLfloat>;
using Vector3f = Vector3<GLfloat>;
using std::vector;

GTEST_TEST(LoadMeshFromObjTest, ErrorModes) {
  {
    // Case: Vertices only reports no faces found.
    std::stringstream in_stream("v 1 2 3");
    DRAKE_EXPECT_THROWS_MESSAGE(LoadMeshFromObj(&in_stream), std::runtime_error,
                                "The OBJ data appears to have no faces.*");
  }
  {
    // Case: Not an obj in any way reports as no faces.
    std::stringstream in_stream("Not an obj\njust some\nmeaningles text.\n");
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadMeshFromObj(&in_stream), std::runtime_error,
        "The OBJ data appears to have no faces.* might not be an OBJ file.+");
  }
  {
    // Case: The obj has no normals. Note: the face specification is otherwise
    // invalid in that it references vertex positions that don't exist.
    std::stringstream in_stream("v 1 2 3\nf 1 2 3\n");
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadMeshFromObj(&in_stream), std::runtime_error,
        "OBJ has no normals; RenderEngineGl requires OBJs with normals.+");
  }
  {
    // Case: not all faces reference normals. Note: the face specification is
    // otherwise invalid in that it references vertex positions that don't
    // exist.
    std::stringstream in_stream(R"""(
v 1 2 3
vn 0 0 1
vt 0 1
f 1 2 3
)""");
    DRAKE_EXPECT_THROWS_MESSAGE(LoadMeshFromObj(&in_stream), std::runtime_error,
                                "Not all faces reference normals.+");
  }
  {
    // Case: not all faces reference uvs. Note: the face specification is
    // otherwise invalid in that it references vertex positions that don't
    // exist.
    std::stringstream in_stream(R"""()
v 1 2 3
vn 0 0 1
vt 0 0
f 1//0 2//0 3//0
)""");
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadMeshFromObj(&in_stream), std::runtime_error,
        "tinyobj::LoadObj failed to load file.+");
  }
}

GTEST_TEST(LoadMeshFromObjTest, NoMeshUvs) {
  // Update: This OBJ has no UVs, but we have now updated this to accept it.
  std::stringstream in_stream(R"""(
v 0.1 0.2 0.3
v 0.4 0.5 0.6
v -0.1 -0.2 -0.3
vn 0 0 1
f 1//1 2//1 3//1
)""");
  EXPECT_NO_THROW(LoadMeshFromObj(&in_stream));
}

// Simply confirms that the filename variant of LoadMeshFromObj successfully
// dispatches files or errors, as appropriate. The actual parsing functionality
// is parsed via the stream interface.
GTEST_TEST(LoadMeshFromObjTest, ReadingFile) {
  const std::string filename =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj");

  MeshData mesh_data = LoadMeshFromObj(filename);
  EXPECT_EQ(mesh_data.positions.rows(), 24);
  EXPECT_EQ(mesh_data.normals.rows(), 24);
  EXPECT_EQ(mesh_data.uvs.rows(), 24);
  EXPECT_EQ(mesh_data.indices.rows(), 12);

  DRAKE_EXPECT_THROWS_MESSAGE(LoadMeshFromObj("Bad file name"),
                              std::runtime_error,
                              "Cannot load the obj file 'Bad file name'");
}

// Helpful struct for specifying multiple cases in the TriangluatePolygons test.
struct TriangluateTestParmas {
  std::string name;
  std::string obj_spec;
  int position_count;
  // The expected normal and texture coordinate for the vertices in face 0 (f0).
  Vector3f n0;
  Vector2f uv0;
  // The expected normal and texture coordinate for the vertices in face 1 (f1).
  Vector3f n1;
  Vector2f uv1;
};

// Confirms that non-triangular faces get triangulated. This also confirms that
// duplication occurs due to associations of vertex positions with multiple
// normals or texture coordinates.
GTEST_TEST(LoadMeshFromObjTest, TriangulatePolygons) {
  /*
             o 4
            ╱  ╲            A five-sided polygon and a four-sided polygon
           ╱    ╲           should be triangulated into three and two
          ╱      ╲          triangles respectively. But with the same
         ╱        ╲         vertices.
        ╱    f0    ╲
     5 o            o 3     The upper and lower polygons have *different*
        ╲          ╱        normals. So, vertices 1 & 2 will be copied.
         ╲        ╱         All vertices have the same texture coordinate, so
       1  o──────o 2        its value does not trigger duplication.
          │  f1  │
          o──────o
          6      7
  */
  constexpr char positions[] = R"""(
  v -1 -1 0
  v 1 -1 0
  v 2 1 0
  v 0 2 0
  v -2 1 0
  v -1 -2 0
  v 1 -2 0)""";

  const Vector3f unit3_z = Vector3f::UnitZ();
  const Vector3f unit3_y = Vector3f::UnitY();
  const Vector2f zero2 = Vector2f::Zero();
  const Vector2f ones2(1, 1);

  vector<TriangluateTestParmas> test_params{
      // Every vertex is associated with a single normal and texture coordinate.
      {"Unique vertices", R"""(
      vn 0 0 1
      vt 0 0
      f 1/1/1 2/1/1 3/1/1 4/1/1 5/1/1
      f 6/1/1 7/1/1 2/1/1 1/1/1)""",
       7, unit3_z, zero2, unit3_z, zero2},
      // Upper and lower faces have different normals; vertices 1 & 2 will be
      // duplicated.
      {"Vertices with multiple normals", R"""(
      vn 0 0 1
      vn 0 1 0
      vt 0 0
      f 1/1/1 2/1/1 3/1/1 4/1/1 5/1/1
      f 6/1/2 7/1/2 2/1/2 1/1/2)""",
       9, unit3_z, zero2, unit3_y, zero2},
      // Upper and lower faces have different uvs; vertices 1 & 2 will be
      // duplicated.
      {"Vertices with multiple uvs", R"""(
      vn 0 0 1
      vt 0 0
      vt 1 1
      f 1/1/1 2/1/1 3/1/1 4/1/1 5/1/1
      f 6/2/1 7/2/1 2/2/1 1/2/1)""",
       9, unit3_z, zero2, unit3_z, ones2},
  };

  for (const auto& params : test_params) {
    SCOPED_TRACE(params.name);
    std::stringstream in_stream;
    in_stream << positions;
    in_stream << params.obj_spec;
    MeshData mesh_data = LoadMeshFromObj(&in_stream);
    EXPECT_EQ(mesh_data.positions.rows(), params.position_count);
    EXPECT_EQ(mesh_data.normals.rows(), params.position_count);
    EXPECT_EQ(mesh_data.uvs.rows(), params.position_count);
    // The two faces will always be triangulated into five triangles.
    EXPECT_EQ(mesh_data.indices.rows(), 5);

    // The first three triangles come from f0, and all have the expected normal
    // and texture coordinate.
    for (int t = 0; t < 3; ++t) {
      for (int i = 0; i < 3; ++i) {
        const int index = mesh_data.indices(t, i);
        EXPECT_TRUE(CompareMatrices(mesh_data.normals.row(index),
                                    params.n0.transpose()));
        EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(index),
                                    params.uv0.transpose()));
      }
    }
    // The last two triangles come from f1 and all have the expected normal and
    // texture coordinate.
    for (int t = 3; t < 5; ++t) {
      for (int i = 0; i < 3; ++i) {
        const int index = mesh_data.indices(t, i);
        EXPECT_TRUE(CompareMatrices(mesh_data.normals.row(index),
                                    params.n1.transpose()));
        EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(index),
                                    params.uv1.transpose()));
      }
    }
  }
}

// Geometry already triangulated gets preserved. This doesn't do variations on
// whether vertex positions need copying due to associations with multiple
// normals/uvs. It relies on `TriangulatePolygons` to handle that.
GTEST_TEST(LoadMeshFromObjTest, PreserveTriangulation) {
  /*
             o 4
            ╱  ╲
           ╱    ╲
          ╱      ╲
         ╱        ╲
        ╱          ╲       Note: Ascii art makes drawing the following edges
     5 o────────────o 3    nearly impossible.
        ╲          ╱
         ╲        ╱        Connect 2 to 5 to form two triangles.
       1  o──────o 2
          │      │         Connect 1 to 7 to form two triangles.
          o──────o
          6      7
  */
  std::stringstream in_stream(R"""(
  v -1 -1 0
  v 1 -1 0
  v 2 1 0
  v 0 2 0
  v -2 1 0
  v -1 -2 0
  v 1 -2 0
  vn 0 0 1
  vt 0 0
  f 1/1/1 2/1/1 5/1/1
  f 2/1/1 3/1/1 5/1/1
  f 3/1/1 4/1/1 5/1/1
  f 1/1/1 6/1/1 7/1/1
  f 1/1/1 7/1/1 2/1/1
  )""");
  MeshData mesh_data = LoadMeshFromObj(&in_stream);
  EXPECT_EQ(mesh_data.positions.rows(), 7);
  EXPECT_EQ(mesh_data.normals.rows(), 7);
  EXPECT_EQ(mesh_data.uvs.rows(), 7);
  EXPECT_EQ(mesh_data.indices.rows(), 5);
}

// The MeshData produces *new* vertices from what was in the OBJ based on what
// vertex gets referenced by which faces. A vertex that doesn't get referenced
// gets omitted.
GTEST_TEST(LoadMeshFromObjTest, RemoveUnreferencedVertices) {
  /*

        4 o───────o 3
          │     ╱ │
          │   ╱   │   o 5
          │ ╱     │
    1, 6  o───────o 2

  */
  std::stringstream in_stream(R"""(
  v -1 -1 0  # First four corners form a box around the origin.
  v 1 -1 0
  v 1 1 0
  v -1 1 0
  v 3 0 0    # Unreferenced vertex to the right of the box (omitted).
  v -1 -1 0  # Duplicate of vertex 1 but propagated through.
  vn 0 0 1
  vt 0 0
  f 1/1/1 2/1/1 3/1/1
  f 6/1/1 3/1/1 4/1/1
  )""");
  MeshData mesh_data = LoadMeshFromObj(&in_stream);
  EXPECT_EQ(mesh_data.positions.rows(), 5);
  EXPECT_EQ(mesh_data.normals.rows(), 5);
  EXPECT_EQ(mesh_data.uvs.rows(), 5);
  EXPECT_EQ(mesh_data.indices.rows(), 2);
}

// Computes the normal to the indicated triangle whose magnitude is twice the
// triangle's area. I.e., for triangle (A, B, C), computes: (B - A) X (C - A).
Vector3f CalcTriNormal(const MeshData& data, int tri_index) {
  const auto& vertices = data.positions;
  const auto& tris = data.indices;
  const auto& a = vertices.row(tris(tri_index, 0));
  const auto& b = vertices.row(tris(tri_index, 1));
  const auto& c = vertices.row(tris(tri_index, 2));
  return (b - a).cross(c - a);
}

// Computes the area of the indicated triangle.
GLfloat CalcTriArea(const MeshData& data, int tri_index) {
  return CalcTriNormal(data, tri_index).norm() * 0.5;
}

// Computes the total area of the given triangles.
GLfloat CalcTotalArea(const MeshData& data) {
  float total_area = 0;
  for (int t = 0; t < data.indices.rows(); ++t) {
    const auto a = CalcTriArea(data, t);
    total_area += a;
  }
  return total_area;
}

// Computes the normal for the triangle (implied by its winding).
Vector3f CalcTriUnitNormal(const MeshData& data, int tri_index) {
  return CalcTriNormal(data, tri_index).normalized();
}

// Computes the centroid of the indicated triangle.
Vector3f CalcTriCentroid(const MeshData& data, int tri_index) {
  const auto& vertices = data.positions;
  const auto& tris = data.indices;
  return (vertices.row(tris(tri_index, 0)) + vertices.row(tris(tri_index, 1)) +
          vertices.row(tris(tri_index, 2))) /
         3.f;
}

// Simply confirm that the utility functions above report good values.
GTEST_TEST(PrimitiveMeshTests, ConfirmUtilities) {
  MeshData data;
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
  EXPECT_EQ(CalcTriNormal(data, 0), Vector3f(0, 0, 2.25));
  EXPECT_EQ(CalcTriUnitNormal(data, 0), Vector3f(0, 0, 1));
  EXPECT_EQ(CalcTriArea(data, 0), 1.125);
  EXPECT_EQ(CalcTriCentroid(data, 0), Vector3f(0.5f, 0.5f, 0));
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
  NormalCone(const MeshData& mesh_data, int t) {
    constexpr float kEps = std::numeric_limits<float>::epsilon();
    const Vector3f n_0 =
        mesh_data.normals.row(mesh_data.indices(t, 0)).normalized();
    const Vector3f n_1 =
        mesh_data.normals.row(mesh_data.indices(t, 1)).normalized();
    const Vector3f n_2 =
        mesh_data.normals.row(mesh_data.indices(t, 2)).normalized();
    const Vector3f p_01 = n_1 - n_0;
    const Vector3f p_02 = n_2 - n_0;
    const Vector3f p_12 = n_2 - n_1;
    const float dist_01 = p_01.norm();
    const float dist_02 = p_02.norm();
    const float dist_12 = p_12.norm();
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
  bool Contains(const Vector3f& dir) const {
    return ray_.dot(dir.normalized()) >= cos_theta_;
  }

  const Vector3f& ray() const { return ray_; }
  float cos_theta() const { return cos_theta_; }

 private:
  Vector3f ray_;
  float cos_theta_{};
};

// Confirms correctness of the normal cone.
GTEST_TEST(PrimitiveMeshTests, NormalCone) {
  constexpr float kEps = std::numeric_limits<float>::epsilon();

  // Note: due to floating point precision issues, normalizing the vector
  // <1, 2, 3> and <1, 2, 3> / sqrt(14) do *not* produce the same result.
  // So, we normalize <1, 2, 3> twice thus guaranteeing that when the
  // NormalCone normalizes again, it should be idempotent w.r.t. the normal
  // values.
  const Vector3f expected_ray = Vector3f(1, 2, 3).normalized().normalized();

  MeshData mesh_data;
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
    EXPECT_FALSE(cone.Contains(Vector3f(1.01, 1.99, 2.99).normalized()));
  }

  // We want to rotate a ray we know to lie on the boundary of the code a
  // "small" amount and have it lie *outside* the cone. Empirically, we find
  // that the value of 0.065 radians is the smallest value such that all of the
  // tests recognize the change from inside to outside. This is largely
  // attributable to the precision of 32-bit floats but hasn't been analyzed
  // carefully.
  const float kThetaEpsilon = 0.065f;

  {
    // Case: Two normals are identical; the third points in a different
    // direction. We'll create the two normals by rotating the expected ray
    // +/- theta around a vector perpendicular to that ray. This guarantees that
    // the minimal cone has angle theta.
    const float theta = M_PI / 7;
    const float cos_theta_expected = std::cos(theta);
    // Note: We're not using RotationMatrix because it cannot support `float`;
    // RotationMatrix::ThrowIfInvalid calls ExtractDoubleOrThrow which throws
    // for T = float. It's not worthing supporting float just to support this
    // bizarre corner of computation.
    const AngleAxisf R(theta, Vector3f(-2, 1, 0).normalized());
    const Vector3f n0 = R * expected_ray;
    const Vector3f n1 = R.inverse() * expected_ray;
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
      const AngleAxisf R_outside(theta + kThetaEpsilon,
                                 Vector3f(-2, -1, 0).normalized());
      EXPECT_FALSE(cone.Contains(R_outside * expected_ray));
    }
  }

  {
    // Case: The normals are all different such that their end points span a
    // plane. In order to get the expected_ray back out of the cone, I need
    // to make sure that the three normals are defined such that their end
    // points are evenly spaced on a circle.
    const float theta = M_PI / 7;
    const float cos_theta_expected = std::cos(theta);
    const Vector3f n_base =
        AngleAxisf(theta, Vector3f(-2, 1, 0).normalized()) * expected_ray;
    for (int i = 0; i < 3; ++i) {
      const AngleAxisf R(2 * M_PI / 3 * i, expected_ray);
      mesh_data.normals.row(i) = R * n_base;
    }
    const NormalCone cone(mesh_data, 0);
    // The ray and cos_theta should match within epsilon.
    EXPECT_TRUE(CompareMatrices(cone.ray(), expected_ray, kEps));
    EXPECT_NEAR(cone.cos_theta(), cos_theta_expected, kEps);
    // The three normals should lie on the boundary of the cone.
    for (int i = 0; i < 3; ++i) {
      const Vector3f n = mesh_data.normals.row(i);
      EXPECT_NEAR(cone.ray().dot(n), cone.cos_theta(), 2 * kEps);
    }
    const AngleAxisf R_outside(theta + kThetaEpsilon,
                               Vector3f(-2, -1, 0).normalized());
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
void TestGenericPrimitiveTraits(const MeshData& mesh,
                                bool origin_is_inside = true) {
  const GLfloat kEps = std::numeric_limits<GLfloat>::epsilon();

  ASSERT_EQ(mesh.positions.rows(), mesh.normals.rows());
  ASSERT_EQ(mesh.positions.rows(), mesh.uvs.rows());

  // All normals have unit length.
  for (int n_i = 0; n_i < mesh.normals.rows(); ++n_i) {
    const Vector3f n = mesh.normals.row(n_i);
    ASSERT_NEAR(n.norm(), 1, kEps);
  }

  // Face normals point outward, within the face's normal cone.
  for (int t = 0; t < mesh.indices.rows(); ++t) {
    Vector3f n_face = CalcTriNormal(mesh, t);
    if (origin_is_inside) {
      Vector3f c = CalcTriCentroid(mesh, t);
      // If the winding were backwards, this dot product would be negative.
      ASSERT_GT(n_face.dot(c), 0) << "for triangle " << t;
    }
    const NormalCone cone(mesh, t);
    ASSERT_TRUE(cone.Contains(n_face))
        << "for triangle " << t << "\n  face normal: " << n_face.transpose();
  }

  // UVs lie in the expected range.
  ASSERT_EQ(mesh.uvs.rows(), mesh.positions.rows());
  for (int i = 0; i < mesh.uvs.rows(); ++i) {
    const GLfloat u = mesh.uvs(i, 0);
    const GLfloat v = mesh.uvs(i, 1);
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
  const GLfloat kEps = std::numeric_limits<GLfloat>::epsilon();

  // Closed form solution for surface area of unit sphere.
  const GLfloat kIdealArea = static_cast<GLfloat>(4 * M_PI);  // 4πR², R = 1.

  float prev_area = 0;
  for (int resolution : {3, 10, 20, 40}) {
    SCOPED_TRACE(fmt::format("Sphere with resolution {}", resolution));
    const MeshData mesh_data = MakeLongLatUnitSphere(resolution, resolution);

    // Confirm area converges towards (but not above) ideal area.
    float curr_area = CalcTotalArea(mesh_data);
    EXPECT_GT(curr_area, prev_area);
    EXPECT_LE(curr_area, kIdealArea);
    prev_area = curr_area;

    // All vertices lie on the unit sphere and that position is equal to the
    // reported normal (and its magnitude is 1).
    for (int v = 0; v < mesh_data.positions.rows(); ++v) {
      const Vector3f v_i = mesh_data.positions.row(v);
      EXPECT_NEAR(v_i.norm(), 1.f, kEps) << "for vertex " << v;
      const Vector3f n_i = mesh_data.normals.row(v);
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
    EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(0),
                                Vector2f(0, 1).transpose()));
    const int row_count = mesh_data.uvs.rows();
    EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(row_count - 1),
                                Vector2f(0, 0).transpose()));
  }
}

GTEST_TEST(PrimitiveMeshTests, MakeUnitCylinder) {
  const GLfloat kEps = std::numeric_limits<GLfloat>::epsilon();

  // Closed form solution for surface area of unit cylinder: H = 1, R = 1.
  //  Total cap area: 2 * πR² = 2π
  //  Barrel area: 2πRH = 2π
  //  Total area = 2π + 2π = 4π
  const GLfloat kIdealArea = static_cast<GLfloat>(4 * M_PI);

  float prev_area = 0;
  for (int resolution : {3, 10, 20, 40}) {
    SCOPED_TRACE(fmt::format("Cylinder with resolution {}", resolution));
    const MeshData mesh_data = MakeUnitCylinder(resolution, resolution);

    // Confirm area converges towards (but not above) ideal area.
    float curr_area = CalcTotalArea(mesh_data);
    EXPECT_GT(curr_area, prev_area);
    EXPECT_LE(curr_area, kIdealArea);
    prev_area = curr_area;

    // All vertices (except first and last) are 1 unit away from z-axis.
    for (int v_i = 1; v_i < mesh_data.positions.rows() - 1; ++v_i) {
      Vector3f v = mesh_data.positions.row(v_i);
      v(2) = 0;
      EXPECT_NEAR(v.norm(), 1.f, kEps) << "for vertex " << v_i;
    }
    // First and last vertices are *on* the z axis.
    for (int v_i : {0, static_cast<int>(mesh_data.positions.rows() - 1)}) {
      Vector3f v = mesh_data.positions.row(v_i);
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
    EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(0),
                                Vector2f(0, 1).transpose()));
    const int row_count = mesh_data.uvs.rows();
    EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(row_count - 1),
                                Vector2f(0, 0).transpose()));
  }
}

GTEST_TEST(PrimitiveMeshTests, MakeSquarePatch) {
  const GLfloat kEps = std::numeric_limits<GLfloat>::epsilon();
  const GLfloat kMax = std::numeric_limits<GLfloat>::max();
  const GLfloat kMeasure = 25;
  const GLfloat kArea = kMeasure * kMeasure;

  for (int resolution : {1, 4, 15}) {
    SCOPED_TRACE(fmt::format("Square patch with resolution {}", resolution));
    const MeshData mesh_data = MakeSquarePatch(kMeasure, resolution);

    TestGenericPrimitiveTraits(mesh_data, false /* origin_is_inside */);

    // The tolerance we select is going to be a scaled machine epsilon. The
    // first scale factor is the actual expected area; this makes our tolerance
    // a _relative_ tolerance. There is a second contribution to error:
    // the sum of many small real values each contribute round-off error. The
    // more we sum, the more round off error we introduce. We scale by the
    // number of triangles (representative of the number of small additions).
    const GLfloat area_epsilon = kEps * kArea * resolution * resolution;
    EXPECT_NEAR(CalcTotalArea(mesh_data), kArea, area_epsilon);
    EXPECT_EQ(mesh_data.positions.rows(), (resolution + 1) * (resolution + 1));
    EXPECT_EQ(mesh_data.indices.rows(), 2 * resolution * resolution);
    for (int t = 0; t < mesh_data.indices.rows(); ++t) {
      EXPECT_TRUE(CompareMatrices(CalcTriUnitNormal(mesh_data, t),
                                  Vector3f(0, 0, 1), kEps));
    }

    // Confirm the bounding box is what we would expected.
    Vector3f min_pt{kMax, kMax, kMax};
    Vector3f max_pt = -min_pt;
    for (int v = 0; v < mesh_data.positions.rows(); ++v) {
      Vector3f vertex = mesh_data.positions.row(v);
      min_pt = min_pt.cwiseMin(vertex);
      max_pt = max_pt.cwiseMax(vertex);
    }
    const Vector3f corner = Vector3f(0.5f, 0.5f, 0.0) * kMeasure;
    EXPECT_TRUE(CompareMatrices(min_pt, -corner, kEps));
    EXPECT_TRUE(CompareMatrices(max_pt, corner, kEps));

    const auto expected_n = Vector3f::UnitZ().transpose();
    for (int v = 0; v < mesh_data.positions.rows(); ++v) {
      ASSERT_TRUE(CompareMatrices(mesh_data.normals.row(v), expected_n));
    }

    // Coarse sampling of UVs. The first vertex should be at (0, 0) the last
    // at (1, 1). See note on texture coordinate testing in
    // MakeLongLatUnitSphere.
    EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(0),
                                Vector2f(0, 0).transpose()));
    const int row_count = mesh_data.uvs.rows();
    EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(row_count - 1),
                                Vector2f(1, 1).transpose()));
  }
}

GTEST_TEST(PrimitiveMeshTests, MakeUnitBox) {
  const GLfloat kEps = std::numeric_limits<GLfloat>::epsilon();
  const GLfloat kMax = std::numeric_limits<GLfloat>::max();

  SCOPED_TRACE("Box");
  const MeshData mesh_data = MakeUnitBox();

  // In computing total area, we scale epsilon by the expected area and also
  // the number of triangles, as per-triangle area gets magnified.
  EXPECT_NEAR(CalcTotalArea(mesh_data), 6.f, kEps);
  EXPECT_EQ(mesh_data.positions.rows(), 24);
  EXPECT_EQ(mesh_data.normals.rows(), 24);
  EXPECT_EQ(mesh_data.indices.rows(), 12);

  // Confirm the bounding box is what we would expected.
  Vector3f min_pt{kMax, kMax, kMax};
  Vector3f max_pt = -min_pt;
  for (int v = 0; v < mesh_data.positions.rows(); ++v) {
    Vector3f vertex = mesh_data.positions.row(v);
    min_pt = min_pt.cwiseMin(vertex);
    max_pt = max_pt.cwiseMax(vertex);
  }
  const Vector3f corner{0.5f, 0.5f, 0.5};
  EXPECT_TRUE(CompareMatrices(min_pt, -corner, kEps));
  EXPECT_TRUE(CompareMatrices(max_pt, corner, kEps));

  TestGenericPrimitiveTraits(mesh_data);

  // We expect the UVs to have the pattern (0, 0) -> (1, 0) -> (1, 1) -> (1, 0)
  // once for each face. We'll test that explicitly.
  for (int row = 0; row < mesh_data.uvs.rows(); row += 4) {
    ASSERT_TRUE(CompareMatrices(mesh_data.uvs.row(row),
                                Vector2f(0, 0).transpose()));
    ASSERT_TRUE(CompareMatrices(mesh_data.uvs.row(row + 1),
                                Vector2f(1, 0).transpose()));
    ASSERT_TRUE(CompareMatrices(mesh_data.uvs.row(row + 2),
                                Vector2f(1, 1).transpose()));
    ASSERT_TRUE(CompareMatrices(mesh_data.uvs.row(row + 3),
                                Vector2f(0, 1).transpose()));
  }
}

GTEST_TEST(PrimitiveMeshTests, MakeCapsule) {
  const GLfloat kEps = std::numeric_limits<GLfloat>::epsilon();
  constexpr double kRadius = 0.75;
  constexpr double kLength = 2.5;
  for (int resolution : {3, 10, 25}) {
    SCOPED_TRACE(fmt::format("Capsule with resolution {}", resolution));
    const MeshData mesh_data = MakeCapsule(resolution, kRadius, kLength);

    // The first half of vertices and normals belong to a hemisphere with center
    // at (0, 0, kLength / 2). Confirm that the vertex positions lie on the
    // sphere and the normals lie in the same direction as from sphere center
    // to vertex. Repeat with the second half, with a sphere center at
    // (0, 0, -kLength / 2).
    const int num_vertices = mesh_data.positions.rows();
    {
      const Vector3f p_MC(0, 0, kLength / 2);
      for (int v = 0; v < num_vertices / 2; ++v) {
        const Vector3f p_MV = mesh_data.positions.row(v);
        const Vector3f p_CV = p_MV - p_MC;
        ASSERT_NEAR(p_CV.norm(), kRadius, kEps);
        const Vector3f nhat_V_expected = p_CV.normalized();
        const Vector3f nhat_V = mesh_data.normals.row(v);
        ASSERT_TRUE(CompareMatrices(nhat_V, nhat_V_expected, kEps));
      }
    }
    {
      const Vector3f p_MC(0, 0, -kLength / 2);
      for (int v = num_vertices / 2; v < num_vertices; ++v) {
        const Vector3f p_MV = mesh_data.positions.row(v);
        const Vector3f p_CV = p_MV - p_MC;
        ASSERT_NEAR(p_CV.norm(), kRadius, kEps);
        const Vector3f nhat_V_expected = p_CV.normalized();
        const Vector3f nhat_V = mesh_data.normals.row(v);
        ASSERT_TRUE(CompareMatrices(nhat_V, nhat_V_expected, kEps));
      }
    }

    TestGenericPrimitiveTraits(mesh_data);

    // See note in MakeLongLatUnitSphere about testing texture coordinates.
    EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(0),
                                Vector2f(0, 1).transpose()));
    const int row_count = mesh_data.uvs.rows();
    EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(row_count - 1),
                                Vector2f(0, 0).transpose()));
    // TODO(SeanCurtis-TRI) Consider testing the v-values of the two equators
    //  to make sure the distribution from 0-1 is arc-length parameterized.
  }
}

}  // namespace
}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
