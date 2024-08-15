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
GTEST_TEST(FeatureNormalSet, EdgeNormalSuccess_SomeFaceNormalFail) {
  // A triangular prism
  //
  //              Mz
  //              ┆
  //              ●        ↗ face normal of triangle v0v1v2
  //             /┆  ＼  ↗
  //            / ┆     ＼ v1
  //           /  ●┄┄┄┄┄┄┄●┄┄┄ My
  //       v2 ●  ╱       /
  //          ┆ / ＼    /      Q
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
  const FeatureNormalSet dut(mesh_M);
  // The query point Q is well outside the prism. Its nearest point is at the
  // middle of edge v0v1. Q is slightly above the X-Y plane of the mesh's
  // frame M, so the face normal of v0v3v1 will give the incorrect
  // inside-outside test, while the edge normal of v0v1 (and also the face
  // normal of v0v1v2) will give the correct inside-outside test.
  const Vector3d p_MQ(1, 2, 0.1);
  // The closest point N is at the middle of the edge v0v1.
  const Vector3d p_MN(1, 1, 0);
  // The edge normal correctly classifies the query point Q as being outside.
  // The dot product is well positive.
  EXPECT_GT(dut.edge_normal({0, 1}).dot(p_MQ - p_MN), 0.8);
  // The face normal of triangle v0v1v2 can classify correctly, but the face
  // normal of triangle v0v3v1 cannot. These checks illustrate the need for
  // FeatureNormalSet.
  EXPECT_GT(mesh_M.face_normal(0).dot(p_MQ - p_MN), 0);
  EXPECT_LT(mesh_M.face_normal(1).dot(p_MQ - p_MN), 0);
}

// This is a simple example that we can use the vertex normal for
// inside-outside test but not an arbitrary face normal.
GTEST_TEST(FeatureNormalSet, VertexNormalSucess_SomeFaceNormalFail) {
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
  const FeatureNormalSet dut(mesh_M);
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
  // These checks illustrate the need for FeatureNormalSet.
  EXPECT_LT(mesh_M.face_normal(0).dot(p_MQ - p_MN), 0);
  EXPECT_GT(mesh_M.face_normal(1).dot(p_MQ - p_MN), 0);
  EXPECT_GT(mesh_M.face_normal(2).dot(p_MQ - p_MN), 0);
}

class FeatureNormalSetTest : public ::testing::Test {
 public:
  FeatureNormalSetTest()
      :  // Surface mesh of a tetrahedron, expressed in the mesh's frame M.
         //
         //              Mz
         //              ┆
         //           v3 ●
         //              ┆
         //              ┆      v2
         //           v0 ●┄┄┄┄┄┄┄●┄┄┄ My
         //             ╱
         //            ╱
         //        v1 ●
         //         ╱
         //        Mx
         //
        mesh_M_(
            // The triangles have outward face winding.
            {SurfaceTriangle{0, 2, 1}, SurfaceTriangle{0, 1, 3},
             SurfaceTriangle{0, 3, 2}, SurfaceTriangle{1, 2, 3}},
            {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
             Vector3d::UnitZ()}) {}

  void SetUp() override {
    // Sanity check that we define the face normals consistent with the mesh.
    ASSERT_EQ(kFaceNormalV0V2V1_, mesh_M_.face_normal(0));
    ASSERT_EQ(kFaceNormalV0V1V3_, mesh_M_.face_normal(1));
    ASSERT_EQ(kFaceNormalV1V2V3_, mesh_M_.face_normal(3));
  }

 protected:
  const TriangleSurfaceMesh<double> mesh_M_;
  const Vector3d kFaceNormalV0V2V1_{-Vector3d::UnitZ()};
  const Vector3d kFaceNormalV0V1V3_{-Vector3d::UnitY()};
  const Vector3d kFaceNormalV1V2V3_{Vector3d(1, 1, 1).normalized()};
  const double kEps_{std::numeric_limits<double>::epsilon()};
};

TEST_F(FeatureNormalSetTest, EdgeNormalIsAverageFaceNormal) {
  const FeatureNormalSet dut(mesh_M_);

  // Edge v1v2 is shared by triangles v0v2v1 and v1v2v3. We expect
  // edge_normal() of v1v2 to be the equal-weight average of the two face
  // normals independent of their face areas. (Here the triangle v0v2v1 has
  // less area than the triangle v1v2v3.)
  const Vector3d kAverageNormalAtEdgeV1V2 =
      (kFaceNormalV0V2V1_ + kFaceNormalV1V2V3_).normalized();

  EXPECT_TRUE(CompareMatrices(dut.edge_normal({1, 2}), kAverageNormalAtEdgeV1V2,
                              kEps_));
}

TEST_F(FeatureNormalSetTest, VertexNormalIsAngleWeightedAverage) {
  const FeatureNormalSet dut(mesh_M_);

  // Vertex v1 is shared by triangles v0v2v1, v0v1v3, and v1v2v3. We expect
  // vertex_normal() of v1 to be the angle-weighted average normal.
  //
  //     triangle        angle at v1 in the triangle
  //   {v0, v2, v1}                π/4
  //   {v0, v1, v3}                π/4
  //   {v1, v2, v3}                π/3
  const Vector3d kAngleWeightedNormalAtV1 =
      (M_PI_4 * kFaceNormalV0V2V1_ + M_PI_4 * kFaceNormalV0V1V3_ +
       M_PI / 3 * kFaceNormalV1V2V3_)
          .normalized();

  EXPECT_TRUE(
      CompareMatrices(dut.vertex_normal(1), kAngleWeightedNormalAtV1, kEps_));
}

class CalcSquaredDistanceToTriangleTest : public ::testing::Test {
 protected:
  //                   My
  //                   ┆
  //                   0.1           v0
  //                   ┆            🮠│
  //                   ┆         🮠   │
  //                   ┆      🮠      │
  //                   ┆   🮠         │
  //                  v1 ┄┄┄┄┄┄┄┄┄┄┄┄0.1┄┄┄┄┄┄┄┄ Mx
  //                                 v2
  const TriangleSurfaceMesh<double> mesh_M_{
      {SurfaceTriangle{0, 1, 2}},
      {Vector3d(0.1, 0.1, 0), Vector3d::Zero(), Vector3d(0.1, 0, 0)}};
  const double kEps_{std::numeric_limits<double>::epsilon()};
};

TEST_F(CalcSquaredDistanceToTriangleTest, Inside) {
  // The query point Q is 15 meters above the triangle, and its projection
  // Q' is inside the triangle.
  //
  //                   My
  //                   ┆
  //                   0.1
  //                   ┆            🮠│
  //                   ┆         🮠   │
  //                   ┆      🮠      │
  //                   ┆   🮠 Q'      │
  //                   0 ┄┄┄┄┄┄┄┄┄┄┄┄0.1┄┄┄┄┄┄┄┄ Mx
  //
  const Vector3d p_MQ(0.04, 0.02, 15);
  // Q' is the closest point.
  const Vector3d kExpectClosestPoint(0.04, 0.02, 0);
  const double kExpectSquaredDistance =
      (p_MQ - kExpectClosestPoint).squaredNorm();

  const SquaredDistanceToTriangle dut =
      CalcSquaredDistanceToTriangle(p_MQ, 0, mesh_M_);

  EXPECT_NEAR(dut.squared_distance, kExpectSquaredDistance, kEps_);
  EXPECT_TRUE(CompareMatrices(dut.closest_point, kExpectClosestPoint, kEps_));
  EXPECT_EQ(dut.location, SquaredDistanceToTriangle::Projection::kInside);
  EXPECT_EQ(dut.v, 0);
}

TEST_F(CalcSquaredDistanceToTriangleTest, OutsideNearEdge) {
  // The query point Q is 10 meters below the triangle, and its projection
  // Q' is outside the triangle nearest to edge v1v2.
  //
  //                   My
  //                   ┆
  //                   0.1           v0
  //                   ┆            🮠│
  //                   ┆         🮠   │
  //                   ┆      🮠      │
  //                   ┆   🮠         │
  //                  v1 ┄┄┄┄┄┄┄┄┄┄┄┄┄v2┄┄┄┄┄┄┄┄ Mx
  //                   ↓      Q'      ↓
  //                   ↓              ↓
  //                   ↓              ↓
  //
  const Vector3d p_MQ(0.04, -0.02, -10);
  // Q' is (0.04, -0.02, 0), and its projection onto the edge v1v2 is the
  // closest point (0.04, 0, 0).
  const Vector3d kExpectClosestPoint(0.04, 0, 0);
  const double kExpectSquaredDistance =
      (p_MQ - kExpectClosestPoint).squaredNorm();

  const SquaredDistanceToTriangle dut =
      CalcSquaredDistanceToTriangle(p_MQ, 0, mesh_M_);

  EXPECT_NEAR(dut.squared_distance, kExpectSquaredDistance, kEps_);
  EXPECT_TRUE(CompareMatrices(dut.closest_point, kExpectClosestPoint, kEps_));
  EXPECT_EQ(dut.location,
            SquaredDistanceToTriangle::Projection::kOutsideNearEdge);
  // dut.v = 1 represents the edge v1v2.
  EXPECT_EQ(dut.v, 1);
}

TEST_F(CalcSquaredDistanceToTriangleTest, OutsideNearVertex) {
  // The query point Q is 5 meters above the triangle, and its projection
  // Q' is outside the triangle nearest to vertex v1.
  //                   My
  //                   ┆
  //                   0.1           v0
  //      ↖            ┆            🮠│
  //         ↖         ┆         🮠   │
  //            ↖      ┆      🮠      │
  //               ↖   ┆   🮠         │
  //                  v1 ┄┄┄┄┄┄┄┄┄┄┄┄┄v2┄┄┄┄┄┄┄┄ Mx
  //       Q'          ↓
  //                   ↓
  //                   ↓
  //                   ↓
  //
  // The area between the two rays is the Voronoi region of vertex v1 (region
  // of points that are closest to vertex v1).
  const Vector3d p_MQ(-0.08, -0.02, 5);
  // The closest point is at the vertex v1.
  const Vector3d kExpectClosestPoint(0, 0, 0);
  const double kExpectSquaredDistance =
      (p_MQ - kExpectClosestPoint).squaredNorm();

  const SquaredDistanceToTriangle dut =
      CalcSquaredDistanceToTriangle(p_MQ, 0, mesh_M_);

  EXPECT_NEAR(dut.squared_distance, kExpectSquaredDistance, kEps_);
  EXPECT_TRUE(CompareMatrices(dut.closest_point, kExpectClosestPoint, kEps_));
  EXPECT_EQ(dut.location,
            SquaredDistanceToTriangle::Projection::kOutsideNearVertex);
  // dut.v = 1 represents the vertex v1.
  EXPECT_EQ(dut.v, 1);
}

// This test fixture uses a convex mesh to cover these cases:
//   - negative signed distance with nearest point in a triangle,
//   - positive signed distance with nearest point in a triangle,
//   - positive signed distance with nearest point at a vertex,
//   - positive signed distance with nearest point in an edge.
// For the cases of negative signed distances with nearest points at vertices
// or in edges, we will use a non-convex mesh in the next test fixture.
class CalcSignedDistanceToSurfaceMesh_ConvexTest : public ::testing::Test {
 public:
  CalcSignedDistanceToSurfaceMesh_ConvexTest()
      :  // This convex surface mesh is the boundary of a tetrahedron.
         //
         //              Mz
         //              ┆
         //           v3 ●
         //              ┆
         //              ┆
         //              ┆
         //              ┆
         //              ┆            v2
         //           v0 ●┄┄┄┄┄┄┄┄┄┄┄┄┄●┄┄┄ My
         //             ╱
         //            ╱
         //           ╱
         //          ╱
         //      v1 ●
         //        ╱
         //       Mx
         //
        mesh_M_{{
                    // The triangle windings give outward normals.
                    SurfaceTriangle{0, 2, 1},
                    SurfaceTriangle{0, 1, 3},
                    SurfaceTriangle{0, 3, 2},
                    SurfaceTriangle{1, 2, 3},
                },
                {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
                 Vector3d::UnitZ()}},
        bvh_M_(mesh_M_),
        mesh_normal_M_(mesh_M_) {}

 protected:
  const TriangleSurfaceMesh<double> mesh_M_;
  const Bvh<Obb, TriangleSurfaceMesh<double>> bvh_M_;
  const FeatureNormalSet mesh_normal_M_;
  const double kEps_{std::numeric_limits<double>::epsilon()};
};

TEST_F(CalcSignedDistanceToSurfaceMesh_ConvexTest, NegativeNearTriangle) {
  // The query point Q is slightly above the triangle v0v1v2 on the X-Y plane.
  const Vector3d p_MQ(0.3, 0.3, 0.1);
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);

  // Expect the nearest point at the projection of Q onto the X-Y plane,
  // which is the plane of the triangle v0v1v2.
  const Vector3d kExpectNearestPoint(0.3, 0.3, 0);
  EXPECT_TRUE(CompareMatrices(d.nearest_point, kExpectNearestPoint, kEps_));
  // Q is inside at distance 0.1 meter from the boundary.
  EXPECT_NEAR(d.signed_distance, -0.1, kEps_);
  EXPECT_TRUE(CompareMatrices(d.gradient, -Vector3d::UnitZ(), kEps_));
}

TEST_F(CalcSignedDistanceToSurfaceMesh_ConvexTest, PositiveNearTriangle) {
  const Vector3d p_MQ(0.1, 0.2, -0.3);
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);

  // Expect the nearest point at the projection of Q onto the X-Y plane,
  // which is the plane of the triangle v0v1v2.
  const Vector3d kExpectNearestPoint(0.1, 0.2, 0);
  EXPECT_TRUE(CompareMatrices(d.nearest_point, kExpectNearestPoint, kEps_));
  // Q is outside at distance 0.3 meter from the boundary.
  EXPECT_NEAR(d.signed_distance, 0.3, kEps_);
  EXPECT_TRUE(CompareMatrices(d.gradient, -Vector3d::UnitZ(), kEps_));
}

TEST_F(CalcSignedDistanceToSurfaceMesh_ConvexTest, PositiveNearVertex) {
  const Vector3d p_MQ = mesh_M_.vertex(0) + Vector3d(-0.005, -0.005, -0.005);
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);

  const Vector3d kExpectNearestPoint = mesh_M_.vertex(0);
  EXPECT_EQ(d.nearest_point, kExpectNearestPoint);
  EXPECT_EQ(d.signed_distance, (p_MQ - kExpectNearestPoint).norm());
  EXPECT_EQ(d.gradient, (p_MQ - kExpectNearestPoint).normalized());
}

TEST_F(CalcSignedDistanceToSurfaceMesh_ConvexTest, PositiveNearEdge) {
  const Vector3d kMidPointEdgeV0V1 =
      (mesh_M_.vertex(0) + mesh_M_.vertex(1)) / 2;
  // Set up the query point Q by a small translation from the midpoint of
  // edge v0v1 in an outward direction perpendicular to the edge.
  const Vector3d p_MQ = kMidPointEdgeV0V1 + Vector3d(0, -0.01, -0.01);
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);

  EXPECT_EQ(d.nearest_point, kMidPointEdgeV0V1);
  EXPECT_EQ(d.signed_distance, (p_MQ - d.nearest_point).norm());
  EXPECT_EQ(d.gradient, (p_MQ - d.nearest_point).normalized());
}

TEST_F(CalcSignedDistanceToSurfaceMesh_ConvexTest, ZeroNearVertex) {
  // The query point is at a vertex. The nearest point is the
  // query point itself. The signed distance is zero. The gradient is
  // selected somewhat arbitrarily.
  const Vector3d p_MQ = mesh_M_.vertex(0);
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);

  EXPECT_NEAR(d.signed_distance, 0, kEps_);
  EXPECT_TRUE(CompareMatrices(d.nearest_point, p_MQ, kEps_));
  // TODO(DamrongGuoy): Check the conical-hull condition in a more robust way.
  //  This check relies on the specific implementation that we have.
  EXPECT_TRUE(CompareMatrices(d.gradient, -Vector3d::UnitX(), kEps_) ||
              CompareMatrices(d.gradient, -Vector3d::UnitY(), kEps_) ||
              CompareMatrices(d.gradient, -Vector3d::UnitZ(), kEps_));
}

// TODO(DamrongGuoy): ZeroNearTriangle, ZeroNearEdge

// This test fixture uses a non-convex mesh. It helps us cover the remaining
// cases from the previous fixture:
//   - negative signed distance with nearest point at a vertex,
//   - negative signed distance with nearest point at an edge.
class CalcSignedDistanceToSurfaceMesh_NonConvexTest : public ::testing::Test {
 public:
  CalcSignedDistanceToSurfaceMesh_NonConvexTest()
      :  // This non-convex surface mesh has one concave vertex v0 at the
         // origin and three concave edges v0v1, v0v2, and v0v3. The vertex
         // v4 is convex.
         //
         //              Mz
         //              ┆
         //           v3 ●
         //              ┆
         //              ┆
         //              ┆
         //              ┆
         //              ┆            v2
         //           v0 ●┄┄┄┄┄┄┄┄┄┄┄┄┄●┄┄┄ My
         //     ●       ╱
         //    v4      ╱
         //           ╱
         //          ╱
         //      v1 ●
         //        ╱
         //       Mx
         //
        mesh_M_{{
                    // The first three triangles share the vertex v0. Their
                    // windings give outward normals.
                    SurfaceTriangle{0, 1, 2},
                    SurfaceTriangle{0, 3, 1},
                    SurfaceTriangle{0, 2, 3},
                    // The last three triangles share the vertex v4. Their
                    // windings give outward normals.
                    SurfaceTriangle{4, 2, 1},
                    SurfaceTriangle{4, 1, 3},
                    SurfaceTriangle{4, 3, 2},
                },
                {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
                 Vector3d::UnitZ(), Vector3d(-1, -1, -1)}},
        bvh_M_(mesh_M_),
        mesh_normal_M_(mesh_M_) {}

 protected:
  const TriangleSurfaceMesh<double> mesh_M_;
  const Bvh<Obb, TriangleSurfaceMesh<double>> bvh_M_;
  const FeatureNormalSet mesh_normal_M_;
};

TEST_F(CalcSignedDistanceToSurfaceMesh_NonConvexTest, NegativeNearVertex) {
  const Vector3d p_MQ = mesh_M_.vertex(0) + Vector3d(-0.005, -0.005, -0.005);
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);

  const Vector3d kExpectNearestPoint = mesh_M_.vertex(0);
  EXPECT_EQ(d.nearest_point, kExpectNearestPoint);
  EXPECT_EQ(d.signed_distance, -(p_MQ - kExpectNearestPoint).norm());
  EXPECT_EQ(d.gradient, (kExpectNearestPoint - p_MQ).normalized());
}

TEST_F(CalcSignedDistanceToSurfaceMesh_NonConvexTest, NegativeNearEdge) {
  const Vector3d kMidPointEdgeV0V1 =
      (mesh_M_.vertex(0) + mesh_M_.vertex(1)) / 2;
  // Set up the query point Q by a small translation from the midpoint of
  // edge v0v1 in an inward direction perpendicular to the concave edge.
  const Vector3d p_MQ = kMidPointEdgeV0V1 + Vector3d(0, -0.01, -0.01);
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);

  EXPECT_EQ(d.nearest_point, kMidPointEdgeV0V1);
  EXPECT_EQ(d.signed_distance, -(p_MQ - d.nearest_point).norm());
  EXPECT_EQ(d.gradient, (d.nearest_point - p_MQ).normalized());
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
