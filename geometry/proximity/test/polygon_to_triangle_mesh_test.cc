#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"

#include <set>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

/* MakeTriangleFromPolygonMesh() doesn't do much. It copies the vertices and
 splits the triangles. Generally, it'll be immediately apparent if the
 triangle mesh is wrong, because we render the triangle mesh in meshcat for
 convex proximity geometries.

 So, this test doesn't attempt to exhaustively confirm every mathematical
 property of the result, but looks for evidence that things are right.

 We create a polygon mesh of a cube with quad faces and confirm the triangle
 mesh satisfies some minimum properties.*/
GTEST_TEST(PolyToTriMeshTest, BasicSmokeTest) {
  // clang-format off
  const std::vector<Vector3d> vertices{
    Vector3d(-1, -1, -1),
    Vector3d(-1, 1, -1),
    Vector3d(1, -1, -1),
    Vector3d(1, 1, -1),
    Vector3d(-1, -1, 1),
    Vector3d(-1, 1, 1),
    Vector3d(1, -1, 1),
    Vector3d(1, 1, 1)
  };
  const PolygonSurfaceMesh<double> poly_cube({
      4, 0, 1, 3, 2,
      4, 0, 2, 6, 4,
      4, 0, 4, 5, 1,
      4, 7, 5, 4, 6,
      4, 7, 6, 2, 3,
      4, 7, 3, 1, 5
  }, vertices);
  // clang-format on

  const TriangleSurfaceMesh<double> tri_cube =
      MakeTriangleFromPolygonMesh(poly_cube);

  // Vertices have been copied verbatim.
  EXPECT_THAT(tri_cube.vertices(), testing::Eq(vertices));

  // For each polygon, we have two triangles.
  ASSERT_EQ(tri_cube.num_elements(), poly_cube.num_elements() * 2);

  int t = 0;
  for (int p = 0; p < poly_cube.num_elements(); ++p) {
    const SurfacePolygon& poly = poly_cube.element(p);

    // Vertices referenced by poly.
    std::set<int> poly_verts;
    for (int vi = 0; vi < poly.num_vertices(); ++vi) {
      poly_verts.insert(poly.vertex(vi));
    }

    // Vertices referenced by pair of triangles and the triangle's normal
    // should match the polygon's normal.
    std::set<int> tri_verts;
    for (int ti = 0; ti < 2; ++ti) {
      const SurfaceTriangle& tri = tri_cube.element(t);
      for (int vi = 0; vi < 3; ++vi) {
        tri_verts.insert(tri.vertex(vi));
      }
      // Normals pointing in the same direction imply same winding.
      EXPECT_TRUE(
          CompareMatrices(poly_cube.face_normal(p), tri_cube.face_normal(t)));
      ++t;
    }

    EXPECT_EQ(tri_verts, poly_verts);

    /* Note: This doesn't confirm that the two triangles properly cover the
     polygon. */
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
