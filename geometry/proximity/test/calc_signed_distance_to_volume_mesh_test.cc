#include "drake/geometry/proximity/calc_signed_distance_to_volume_mesh.h"

#include <gtest/gtest.h>

#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using std::vector;

GTEST_TEST(VolumeMeshSignedDistanceTest, CalcSignedDistance) {
  // Simple non-convex mesh of three tetrahedra and five vertices. The vertex
  // v4 is concave.
  //
  //              Mz
  //              ┆
  //           v3 ●
  //              ┆
  //              ┆
  //              ┆
  //              ┆v4
  //              ┆●           v2
  //           v0 ●┄┄┄┄┄┄┄┄┄┄┄┄┄●┄┄┄ My
  //             ╱┆
  //            ╱ ┆
  //           ╱  ┆
  //          ╱   ┆
  //      v1 ●
  //        ╱
  //       Mx
  //
  const VolumeMesh<double> mesh_M{
      {VolumeElement{0, 1, 2, 4}, VolumeElement{0, 2, 3, 4},
       VolumeElement{0, 3, 1, 4}},
      {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
       Vector3d::UnitZ(), Vector3d(0.25, 0.25, 0.25)}};
  const VolumeMeshSignedDistance dut(mesh_M);
  {
    // The query point is inside, nearest to the concave vertex v4. The signed
    // distance is negative. The gradient is in the direction from the query
    // point to the nearest point.
    const Vector3d p_MQ = mesh_M.vertex(4) + Vector3d(-0.005, -0.005, -0.005);
    const auto d = dut.CalcSignedDistance(p_MQ);
    const Vector3d kExpectNearestPoint = mesh_M.vertex(4);
    EXPECT_EQ(d.nearest_point, kExpectNearestPoint);
    EXPECT_EQ(d.signed_distance, -(p_MQ - kExpectNearestPoint).norm());
    EXPECT_EQ(d.gradient, (kExpectNearestPoint - p_MQ).normalized());
  }
  {
    // The query point is outside, nearest to the vertex v1. The signed
    // distance is positive. The gradient is in the direction from the
    // nearest point to the query point.
    const Vector3d p_MQ = mesh_M.vertex(1) + Vector3d(0.005, -0.005, -0.005);
    const auto d = dut.CalcSignedDistance(p_MQ);
    const Vector3d kExpectNearestPoint = mesh_M.vertex(1);
    EXPECT_EQ(d.nearest_point, kExpectNearestPoint);
    EXPECT_EQ(d.signed_distance, (p_MQ - kExpectNearestPoint).norm());
    EXPECT_EQ(d.gradient, (p_MQ - kExpectNearestPoint).normalized());
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
