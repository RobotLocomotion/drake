#include "drake/geometry/proximity/calc_signed_distance_to_surface_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using std::vector;

// This is a simple example that we can use the edge normal for inside-outside
// test but not an arbitrary face normal.
GTEST_TEST(VertexEdgeNormalTest, EdgeNormalSuccess_SomeFaceNormalFail) {
  // A triangular prism
  //
  //              Mz
  //              ┆
  //              ●        ↗ face normal of triangle v0v1v2
  //             /┆  ＼  ↗
  //            / ┆     ＼ v1
  //           /  ●┄┄┄┄┄┄┄●┄┄┄ My
  //       v2 ●  ╱       /
  //          ┆ / ＼    / Q
  //          ┆╱    ＼ /
  //       v3 ●┄┄┄┄┄┄┄● v0
  //         /    ↓ face normal of triangle v0v3v1
  //        /     ↓
  //       Mx
  //
  // For the purpose of this test, we create only the two triangles sharing
  // the edge v0v1, instead of eight triangles of the triangular prism.
  // The face windings of the two triangles give outward normals with respect
  // to the triangular prism.
  const TriangleSurfaceMesh<double> mesh_M{
      {SurfaceTriangle{0, 1, 2}, SurfaceTriangle{0, 3, 1}},
      {Vector3d(2, 1, 0), Vector3d::UnitY(), Vector3d(2, 0, 1),
       Vector3d(2, 0, 0)}};
  const VertexEdgeNormal dut(mesh_M);
  // The query point Q is outside the prism, near the middle of edge v0v1, and
  // slightly above the X-Y plane of frame M of the mesh.
  const Vector3d p_MQ(1, 1.01, 0.01);
  // The closest point N is on the edge v0v1.
  const Vector3d p_MN(1, 1, 0);
  // The edge normal correctly classifies the query point Q as being outside
  // (positive dot product).
  EXPECT_GT(dut.edge_normal({0, 1}).dot(p_MQ - p_MN), 0);
  // The face normal of triangle v0v1v2 can classify correctly, but the face
  // normal of triangle v0v3v1 cannot. These checks illustrate the need for
  // VertexEdgeNormal.
  EXPECT_GT(mesh_M.face_normal(0).dot(p_MQ - p_MN), 0);
  EXPECT_LT(mesh_M.face_normal(1).dot(p_MQ - p_MN), 0);
}

// This is a simple example that we can use the vertex normal for
// inside-outside test but not an arbitrary face normal.
GTEST_TEST(VertexEdgeNormalTest, VertexNormalSucess_SomeFaceNormalFail) {
  // A triangular prism
  //
  //                           Mz
  //                           ┆
  //                           ●        ↗ face normal of triangle v0v1v2
  //                          /┆  ＼  ↗
  //                         / ┆     ＼ v1
  //                        /  ●┄┄┄┄┄┄┄●┄┄┄ My
  //                    v2 ●  ╱       /
  //                     ↙ ┆ / ＼    / Q
  //  face normal of   ↙   ┆╱    ＼ /
  //  triangle v0v2v3   v3 ●┄┄┄┄┄┄┄● v0
  //                      /    ↓     Q
  //                     /     ↓
  //                    Mx    face normal of triangle v0v3v1
  //
  // For the purpose of this test, we create only three triangles sharing
  // the vertex v0, instead of eight triangles of the triangular prism.
  // The face windings of the three triangles give outward normals with
  // respect to the triangular prism.
  const TriangleSurfaceMesh<double> mesh_M{
      {SurfaceTriangle{0, 1, 2}, SurfaceTriangle{0, 3, 1},
       SurfaceTriangle{0, 2, 3}},
      {Vector3d(2, 1, 0), Vector3d::UnitY(), Vector3d(2, 0, 1),
       Vector3d(2, 0, 0)}};
  const VertexEdgeNormal dut(mesh_M);
  // The query point Q is outside the prism, near the vertex v0, and slightly
  // below the plane of triangle v0v1v2.
  const Vector3d p_MQ(2.01, 1.01, -0.02);
  // The closest point N is at the vertex v0.
  const Vector3d p_MN(2, 1, 0);
  // The vertex normal correctly classifies the query point Q as being
  // outside (positive dot product).
  EXPECT_GT(dut.vertex_normal(0).dot(p_MQ - p_MN), 0);
  // The face normal of triangle v0v1v2 incorrectly classifies the query
  // point Q as being inside (negative dot product), although each of the
  // face normals of the other two triangles does it correctly.
  // These checks illustrate the need for VertexEdgeNormal.
  EXPECT_LT(mesh_M.face_normal(0).dot(p_MQ - p_MN), 0);
  EXPECT_GT(mesh_M.face_normal(1).dot(p_MQ - p_MN), 0);
  EXPECT_GT(mesh_M.face_normal(2).dot(p_MQ - p_MN), 0);
}

GTEST_TEST(VertexEdgeNormalTest, EdgeNormalIsAverageFaceNormal) {
  // A tetrahedron
  //
  //              Mz
  //              ┆   ╱
  //           v3 ●  ╱
  //              ┆ ╱
  //           v0 ┆╱    v2
  //     ┄┄┄┄┄┄┄┄┄●┄┄┄┄┄┄┄●┄┄┄ My
  //             ╱┆
  //            ╱ ┆
  //        v1 ●  ┆
  //         ╱    ┆
  //        Mx
  //
  // For the purpose of this test, we create only the two triangles sharing
  // the edge v1v2 of the tetrahedron. The face windings of the triangles
  // give outward normals with respect to the tetrahedron.
  const TriangleSurfaceMesh<double> mesh_M{
      {SurfaceTriangle{0, 2, 1}, SurfaceTriangle{1, 2, 3}},
      {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
       Vector3d::UnitZ()}};
  VertexEdgeNormal dut(mesh_M);

  const double kEps = std::numeric_limits<double>::epsilon();
  // Edge v1v2 is shared by the two faces.
  const Vector3d kAverageNormalAtEdgeV1V2 =
      (mesh_M.face_normal(0) + mesh_M.face_normal(1)).normalized();
  EXPECT_TRUE(
      CompareMatrices(dut.edge_normal({1, 2}), kAverageNormalAtEdgeV1V2, kEps));
}

GTEST_TEST(VertexEdgeNormalTest, VertexNormalIsAngleWeightedAverage) {
  // A tetrahedron
  //
  //              Mz
  //              ┆   ╱
  //           v3 ●  ╱
  //              ┆ ╱
  //           v0 ┆╱    v2
  //     ┄┄┄┄┄┄┄┄┄●┄┄┄┄┄┄┄●┄┄┄ My
  //             ╱┆
  //            ╱ ┆
  //        v1 ●  ┆
  //         ╱    ┆
  //        Mx
  //
  // For the purpose of this test, we create only the three triangles sharing
  // the vertex v1 of the tetrahedron. The face windings of the triangles
  // give outward normals with respect to the tetrahedron.
  const TriangleSurfaceMesh<double> mesh_M{
      {SurfaceTriangle{0, 2, 1}, SurfaceTriangle{0, 1, 3},
       SurfaceTriangle{1, 2, 3}},
      {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
       Vector3d::UnitZ()}};
  VertexEdgeNormal dut(mesh_M);

  const double kEps = std::numeric_limits<double>::epsilon();
  //   face index   vertices of the triangle    angle at v1 in the triangle
  //       0        {v0, v2, v1}                π/4
  //       1        {v0, v1, v3}                π/4
  //       2        {v1, v2, v3}                π/3
  const Vector3d kAngleWeightedNormalAtV1 =
      (M_PI_4 * mesh_M.face_normal(0) + M_PI_4 * mesh_M.face_normal(1) +
       M_PI / 3 * mesh_M.face_normal(2))
          .normalized();
  EXPECT_TRUE(
      CompareMatrices(dut.vertex_normal(1), kAngleWeightedNormalAtV1, kEps));
}

GTEST_TEST(CalcSquaredDistanceToTriangleTest, ClosestToTriangle) {
  // Triangle ABC
  //                   My
  //                   ┆
  //                   ┆              A
  //                   ┆            🮠│
  //                   ┆         🮠   │
  //                   ┆      🮠      │
  //                   ┆   🮠Q        │
  //                   B ┄┄┄┄┄┄┄┄┄┄┄┄┄C┄┄┄┄┄┄┄┄ Mx
  //
  // The projection of Q is in the triangle.
  //
  const Vector3d p_MA(0.1, 0.1, 0);
  const Vector3d p_MB = Vector3d::Zero();
  const Vector3d p_MC(0.1, 0, 0);
  const TriangleSurfaceMesh<double> mesh_M{{SurfaceTriangle{0, 1, 2}},
                                           {p_MA, p_MB, p_MC}};
  const Vector3d p_MQ(0.04, 0.01, 15);
  const SquaredDistanceToTriangle dut =
      CalcSquaredDistanceToTriangle(p_MQ, 0, mesh_M);
  // Expect the closest point N in the triangle.
  const Vector3d expect_p_MN(0.04, 0.01, 0);
  const double kEps = std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(dut.squared_distance, (p_MQ - expect_p_MN).squaredNorm(), kEps);
  EXPECT_TRUE(CompareMatrices(dut.closest_point, expect_p_MN, kEps));
  EXPECT_EQ(dut.location, SquaredDistanceToTriangle::Location::kTriangle);
  EXPECT_EQ(dut.v, 0);
}

GTEST_TEST(CalcSquaredDistanceToTriangleTest, ClosestToEdge) {
  // Triangle ABC
  //                   My
  //                   ┆
  //                   ┆              A
  //                   ┆            🮠│
  //                   ┆         🮠   │
  //                   ┆      🮠      │
  //                   ┆   🮠         │
  //                   B ┄┄┄┄┄┄┄┄┄┄┄┄┄C┄┄┄┄┄┄┄┄ Mx
  //                   ↓       Q      ↓
  //                   ↓              ↓
  //                   ↓              ↓
  //                   ↓              ↓
  //
  // The area between the two rays is the Voronoi region of edge BC (region
  // of query points that are closest to edge BC).
  const Vector3d p_MA(0.1, 0.1, 0);
  const Vector3d p_MB = Vector3d::Zero();
  const Vector3d p_MC(0.1, 0, 0);
  const TriangleSurfaceMesh<double> mesh_M{{SurfaceTriangle{0, 1, 2}},
                                           {p_MA, p_MB, p_MC}};
  const Vector3d p_MQ(0.05, -0.3, 0.001);
  const SquaredDistanceToTriangle dut =
      CalcSquaredDistanceToTriangle(p_MQ, 0, mesh_M);
  // Expect the closest point N at the middle of edge BC.
  const Vector3d expect_p_MN(0.05, 0, 0);
  const double kEps = std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(dut.squared_distance, (p_MQ - expect_p_MN).squaredNorm(), kEps);
  EXPECT_TRUE(CompareMatrices(dut.closest_point, expect_p_MN, kEps));
  EXPECT_EQ(dut.location, SquaredDistanceToTriangle::Location::kEdge);
  EXPECT_EQ(dut.v, 1);
}

GTEST_TEST(CalcSquaredDistanceToTriangleTest, ClosestToVertex) {
  // Triangle ABC
  //                   My
  //                   ┆
  //                   ┆              A
  //      ↖            ┆            🮠│
  //         ↖         ┆         🮠   │
  //            ↖      ┆      🮠      │
  //               ↖   ┆   🮠         │
  //                   B ┄┄┄┄┄┄┄┄┄┄┄┄┄C┄┄┄┄┄┄┄┄ Mx
  //       Q           ↓
  //                   ↓
  //                   ↓
  //                   ↓
  //
  // The area between the two rays is the Voronoi region of vertex B (region
  // of query points that are closest to vertex B from the triangle ABC).
  const Vector3d p_MA(0.1, 0.1, 0);
  const Vector3d p_MB = Vector3d::Zero();
  const Vector3d p_MC(0.1, 0, 0);
  const TriangleSurfaceMesh<double> mesh_M{{SurfaceTriangle{0, 1, 2}},
                                           {p_MA, p_MB, p_MC}};
  // The query point Q is nearest to vertex B.
  const Vector3d p_MQ(-0.1, -0.01, 0.001);
  const SquaredDistanceToTriangle dut =
      CalcSquaredDistanceToTriangle(p_MQ, 0, mesh_M);
  const double kEps = std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(dut.squared_distance, (p_MQ - p_MB).squaredNorm(), kEps);
  EXPECT_TRUE(CompareMatrices(dut.closest_point, p_MB, kEps));
  EXPECT_EQ(dut.location, SquaredDistanceToTriangle::Location::kVertex);
  EXPECT_EQ(dut.v, 1);
}

// TODO(DamrongGuoy) Test with a nonconvex mesh. It will allow an interior
//  query point with its nearest point at a concave vertex or a concave edge.
//  Perhaps an L-shape geometry would be good.

GTEST_TEST(CalcSignedDistanceToSurfaceMeshTest, Test) {
  const TriangleSurfaceMesh<double> mesh_M =
      MakeBoxSurfaceMesh<double>(Box(0.08, 0.04, 0.02), 0.005);
  const Bvh<Obb, TriangleSurfaceMesh<double>> bvh_M(mesh_M);
  const VertexEdgeNormal mesh_normal_M(mesh_M);
  {
    // The query point Q is inside the box. The signed distance is negative.
    const Vector3d p_MQ(0, 0, 0.01 - 0.001);
    const SignedDistanceToSurfaceMesh d =
        CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M, bvh_M, mesh_normal_M);
    Vector3d p_MN(0, 0, 0.01);
    const double kEps = std::numeric_limits<double>::epsilon();
    EXPECT_NEAR(d.signed_distance, -(p_MQ - p_MN).norm(), kEps);
    EXPECT_TRUE(CompareMatrices(d.nearest_point, p_MN, kEps));
    EXPECT_TRUE(CompareMatrices(d.gradient, Vector3d::UnitZ(), kEps));
  }
  {
    // The query point Q is outside the box, nearest to an edge. The signed
    // distance is positive.
    const Vector3d p_MQ(0, 0.02 + 0.001, 0.01 + 0.001);
    const SignedDistanceToSurfaceMesh d =
        CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M, bvh_M, mesh_normal_M);
    // Expect the nearest point at the middle of an edge.
    Vector3d p_MN(0, 0.02, 0.01);
    const double kEps = std::numeric_limits<double>::epsilon();
    EXPECT_NEAR(d.signed_distance, (p_MQ - p_MN).norm(), kEps);
    EXPECT_TRUE(CompareMatrices(d.nearest_point, p_MN, kEps));
    EXPECT_TRUE(CompareMatrices(d.gradient, (p_MQ - p_MN).normalized(), kEps));
  }
  {
    // The query point Q is outside the box, nearest to a vertex. The signed
    // distance is positive.
    const Vector3d p_MQ(0.04 + 0.001, 0.02 + 0.001, 0.01 + 0.001);
    const SignedDistanceToSurfaceMesh d =
        CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M, bvh_M, mesh_normal_M);
    Vector3d p_MN(0.04, 0.02, 0.01);
    const double kEps = std::numeric_limits<double>::epsilon();
    EXPECT_NEAR(d.signed_distance, (p_MQ - p_MN).norm(), kEps);
    EXPECT_TRUE(CompareMatrices(d.nearest_point, p_MN, kEps));
    EXPECT_TRUE(CompareMatrices(d.gradient, (p_MQ - p_MN).normalized(), kEps));
  }
  {
    // The query point is at a vertex of the box. The nearest point is the
    // query point itself. The signed distance is zero. The gradient is
    // selected arbitrarily.
    const Vector3d p_MQ(0.04, 0.02, 0.01);
    const SignedDistanceToSurfaceMesh d =
        CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M, bvh_M, mesh_normal_M);
    const double kEps = std::numeric_limits<double>::epsilon();
    EXPECT_NEAR(d.signed_distance, 0, kEps);
    EXPECT_TRUE(CompareMatrices(d.nearest_point, p_MQ, kEps));
    EXPECT_TRUE(CompareMatrices(d.gradient, Vector3d::UnitX(), kEps) ||
                CompareMatrices(d.gradient, Vector3d::UnitY(), kEps) ||
                CompareMatrices(d.gradient, Vector3d::UnitZ(), kEps));
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
