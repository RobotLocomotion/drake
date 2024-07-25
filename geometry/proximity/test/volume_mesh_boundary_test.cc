#include "drake/geometry/proximity/volume_mesh_boundary.h"

#include <gtest/gtest.h>

#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

GTEST_TEST(VolumeMeshBoundary, FromVolumeMesh) {
  const VolumeMesh<double> mesh_M({VolumeElement{0, 1, 2, 3}},
                                  {Vector3d::Zero(), Vector3d::UnitX(),
                                   Vector3d::UnitY(), Vector3d::UnitZ()});
  const VolumeMeshBoundary dut(mesh_M);

  // Check that data was populated. The actual value depends on other code.
  EXPECT_EQ(dut.tri_mesh().num_triangles(), 4);
  EXPECT_FALSE(dut.tri_bvh().root_node().is_leaf());
  EXPECT_NE(dut.vertex_edge_normal().vertex_normal(0), Vector3d::Zero());
}

GTEST_TEST(VolumeMeshBoundary, FromSurfaceMesh) {
  //              Mz
  //              ┆
  //           v3 ●
  //              ┆
  //              ┆
  //              ┆
  //              ┆
  //              ┆             v2
  //           v0 ●┄┄┄┄┄┄┄┄┄┄┄┄┄●┄┄┄ My
  //             ╱┆
  //            ╱ ┆
  //           ╱  ┆
  //          ╱   ┆
  //      v1 ●
  //        ╱
  //       Mx
  //
  // It's not const because VolumeMeshBoundary will take ownership of the mesh.
  TriangleSurfaceMesh<double> mesh_M{
      {// The triangle windings give outward normals.
       SurfaceTriangle{0, 2, 1}, SurfaceTriangle{0, 1, 3},
       SurfaceTriangle{0, 3, 2}, SurfaceTriangle{1, 2, 3}},
      {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
       Vector3d::UnitZ()}};

  const VolumeMeshBoundary dut(std::move(mesh_M));

  // Check that data was populated. The actual value depends on other code.
  EXPECT_EQ(dut.tri_mesh().num_triangles(), 4);
  EXPECT_FALSE(dut.tri_bvh().root_node().is_leaf());
  EXPECT_NE(dut.vertex_edge_normal().vertex_normal(0), Vector3d::Zero());
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake