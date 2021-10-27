#include "drake/geometry/proximity/make_ellipsoid_mesh.h"

#include <gtest/gtest.h>

#include "drake/geometry/proximity/tessellation_strategy.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Correctness of MakeEllipsoidVolumeMesh() depends on MakeSphereVolumeMesh().
// This is a smoke test only.
GTEST_TEST(MakeEllipsoidMeshTest, MakeEllipsoidVolumeMesh) {
  // For an ellipsoid with bounding box 10cm x 16cm x 6cm, its semi-axes are
  // 5cm, 8cm, and 3cm long.
  const Ellipsoid ellipsoid(0.05, 0.08, 0.03);

  // Coarsest mesh for resolution_hint equals the length of the major
  // axis 16cm. The coarsest mesh is an octahedron with 7 vertices and 8
  // tetrahedra.
  {
    const auto coarse_mesh = MakeEllipsoidVolumeMesh<double>(
        ellipsoid, 0.16, TessellationStrategy::kDenseInteriorVertices);
    EXPECT_EQ(7, coarse_mesh.num_vertices());
    EXPECT_EQ(8, coarse_mesh.num_elements());
  }
  // Cutting the resolution_hint in half from 16cm down to 8cm increases
  // the number of tetrahedra by 8X.
  {
    const auto medium_mesh = MakeEllipsoidVolumeMesh<double>(
        ellipsoid, 0.08, TessellationStrategy::kDenseInteriorVertices);
    EXPECT_EQ(64, medium_mesh.num_elements());
  }
}

// Correctness of MakeEllipsoidSurfaceMesh() depends on
// MakeEllipsoidVolumeMesh(). This is a smoke test only.
GTEST_TEST(MakeEllipsoidMeshTest, MakeEllipsoidSurfaceMesh) {
  // For an ellipsoid with bounding box 10cm x 16cm x 6cm, its semi-axes are
  // 5cm, 8cm, and 3cm long.
  const Ellipsoid ellipsoid(0.05, 0.08, 0.03);

  // Coarsest mesh for resolution_hint equals the length of the major
  // axis 16cm. The coarsest mesh is an octahedron with 6 vertices and 8
  // triangles.
  {
    const auto coarse_mesh = MakeEllipsoidSurfaceMesh<double>(ellipsoid, 0.16);
    EXPECT_EQ(6, coarse_mesh.num_vertices());
    EXPECT_EQ(8, coarse_mesh.num_triangles());
  }

  // Cutting the resolution_hint in half from 16cm down to 8cm increases
  // the number of triangles by 4X.
  {
    const auto medium_mesh = MakeEllipsoidSurfaceMesh<double>(ellipsoid, 0.08);
    EXPECT_EQ(32, medium_mesh.num_triangles());
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
