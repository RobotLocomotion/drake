#include "drake/geometry/proximity/mesh_distance_boundary.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

GTEST_TEST(MeshDistanceBoundary, FromVolumeMesh) {
  const VolumeMesh<double> mesh_M({VolumeElement{0, 1, 2, 3}},
                                  {Vector3d::Zero(), Vector3d::UnitX(),
                                   Vector3d::UnitY(), Vector3d::UnitZ()});
  const MeshDistanceBoundary dut(mesh_M);

  // Check that data was populated. The actual value depends on other code.
  EXPECT_EQ(dut.tri_mesh().num_triangles(), 4);
  EXPECT_FALSE(dut.tri_bvh().root_node().is_leaf());
  EXPECT_TRUE(std::holds_alternative<FeatureNormalSet>(dut.feature_normal()));
}

GTEST_TEST(MeshDistanceBoundary, FromSurfaceMesh) {
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
  // It's not const because MeshDistanceBoundary will take ownership of the
  // mesh.
  TriangleSurfaceMesh<double> mesh_M{
      {// The triangle windings give outward normals.
       SurfaceTriangle{0, 2, 1}, SurfaceTriangle{0, 1, 3},
       SurfaceTriangle{0, 3, 2}, SurfaceTriangle{1, 2, 3}},
      {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
       Vector3d::UnitZ()}};

  const MeshDistanceBoundary dut(std::move(mesh_M));

  // Check that data was populated. The actual value depends on other code.
  EXPECT_EQ(dut.tri_mesh().num_triangles(), 4);
  EXPECT_FALSE(dut.tri_bvh().root_node().is_leaf());
  EXPECT_TRUE(std::holds_alternative<FeatureNormalSet>(dut.feature_normal()));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
