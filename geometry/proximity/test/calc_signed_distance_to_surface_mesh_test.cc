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
  // The query point Q projects inside the triangle.
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
  // Q is 15 meters above the triangle.
  const Vector3d p_MQ(0.04, 0.02, 15);
  // The projection is the closest point.
  const Vector3d kClosest(0.04, 0.02, 0);

  const SquaredDistanceToTriangle dut =
      CalcSquaredDistanceToTriangle(p_MQ, 0, mesh_M_);

  EXPECT_NEAR(dut.squared_distance, (p_MQ - kClosest).squaredNorm(), kEps_);
  EXPECT_TRUE(CompareMatrices(dut.closest_point, kClosest, kEps_));
  EXPECT_EQ(dut.location, SquaredDistanceToTriangle::Projection::kInside);
  EXPECT_EQ(dut.v, 0);
}

TEST_F(CalcSquaredDistanceToTriangleTest, OutsideNearEdge) {
  // The query point Q projects outside the triangle nearest to edge v1v2.
  // The area between the two rays is the region of such projections.
  //
  //                   My
  //                   ┆
  //                   0.1           v0
  //                   ┆            🮠│
  //                   ┆         🮠   │
  //                   ┆      🮠      │
  //                   ┆   🮠         │
  //                  v1 ┄┄┄┄┄┄┄┄┄┄┄┄┄v2┄┄┄┄┄┄┄┄ Mx
  //                   ↓     ● Q'     ↓
  //                   ↓              ↓
  //                   ↓              ↓
  //
  // Q is 10 meters below the triangle.
  const Vector3d p_MQ(0.04, -0.02, -10);
  // Q' is the projection of Q onto the plane of the triangle at
  // (0.04, -0.02, 0). Q' is outside the triangle and projects onto the
  // edge v1v2 at (0.04, 0, 0), which is the closest point.
  const Vector3d kClosest(0.04, 0, 0);

  const SquaredDistanceToTriangle dut =
      CalcSquaredDistanceToTriangle(p_MQ, 0, mesh_M_);

  EXPECT_NEAR(dut.squared_distance, (p_MQ - kClosest).squaredNorm(), kEps_);
  EXPECT_TRUE(CompareMatrices(dut.closest_point, kClosest, kEps_));
  EXPECT_EQ(dut.location,
            SquaredDistanceToTriangle::Projection::kOutsideNearEdge);
  // dut.v = 1 represents the edge v1v2.
  EXPECT_EQ(dut.v, 1);
}

// Test a special case that the query point Q is in an edge.
TEST_F(CalcSquaredDistanceToTriangleTest, SpecialCasePointInEdge) {
  //
  //                   My
  //                   ┆
  //                   0.1           v0
  //                   ┆            🮠│
  //                   ┆         🮠   │
  //                   ┆      🮠      │
  //                   ┆   🮠         │
  //                  v1 ┄┄┄┄Q┄┄┄┄┄┄┄┄v2┄┄┄┄┄┄┄┄ Mx
  //
  // Q is in the edge v1v2.
  const Vector3d p_MQ(0.04, 0, 0);
  const SquaredDistanceToTriangle dut =
      CalcSquaredDistanceToTriangle(p_MQ, 0, mesh_M_);

  EXPECT_NEAR(dut.squared_distance, 0, kEps_);
  // Q is its own closest point.
  EXPECT_TRUE(CompareMatrices(dut.closest_point, p_MQ, kEps_));
  EXPECT_EQ(dut.location,
            SquaredDistanceToTriangle::Projection::kOutsideNearEdge);
  // dut.v = 1 represents the edge v1v2.
  EXPECT_EQ(dut.v, 1);
}

TEST_F(CalcSquaredDistanceToTriangleTest, OutsideNearVertex) {
  // Q' is the projection of the query point Q onto the plane of the
  // triangle.  Q' is outside the triangle nearest to vertex v1.
  // The area between the two rays is the region of such projections.
  //
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
  // Q is 5 meters above the triangle.
  const Vector3d p_MQ(-0.08, -0.02, 5);
  const SquaredDistanceToTriangle dut =
      CalcSquaredDistanceToTriangle(p_MQ, 0, mesh_M_);

  EXPECT_NEAR(dut.squared_distance, (p_MQ - mesh_M_.vertex(1)).squaredNorm(),
              kEps_);
  EXPECT_TRUE(CompareMatrices(dut.closest_point, mesh_M_.vertex(1), kEps_));
  EXPECT_EQ(dut.location,
            SquaredDistanceToTriangle::Projection::kOutsideNearVertex);
  // dut.v = 1 represents the vertex v1.
  EXPECT_EQ(dut.v, 1);
}

// Test a special case that the query point Q is at a vertex.
TEST_F(CalcSquaredDistanceToTriangleTest, SpecialCasePointAtVertex) {
  //
  //                   My
  //                   ┆
  //                   0.1           v0
  //                   ┆            🮠│
  //                   ┆         🮠   │
  //                   ┆      🮠      │
  //                   ┆   🮠         │
  //                  v1 ┄┄┄┄┄┄┄┄┄┄┄┄┄●┄┄┄┄┄┄┄┄ Mx
  //                                Q = v2
  // Q is at v2.
  const Vector3d p_MQ = mesh_M_.vertex(2);
  const SquaredDistanceToTriangle dut =
      CalcSquaredDistanceToTriangle(p_MQ, 0, mesh_M_);

  EXPECT_NEAR(dut.squared_distance, 0, kEps_);
  // Q is its own closest point.
  EXPECT_TRUE(CompareMatrices(dut.closest_point, p_MQ, kEps_));
  EXPECT_EQ(dut.location,
            SquaredDistanceToTriangle::Projection::kOutsideNearVertex);
  // dut.v = 2 represents the vertex v2.
  EXPECT_EQ(dut.v, 2);
}

// This test fixture covers cases of positive signed distances with
// nearest points in a triangle, in an edge, or at a vertex. The next test
// fixture covers cases of negative signed distances.
class CalcSignedDistanceToSurfaceMesh_Positive : public ::testing::Test {
 public:
  CalcSignedDistanceToSurfaceMesh_Positive()
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

  void SetUp() override {
    // Sanity check that we define the face normals consistent with the mesh.
    ASSERT_EQ(kFaceNormalV0V2V1_, mesh_M_.face_normal(0));
    ASSERT_EQ(kFaceNormalV0V1V3_, mesh_M_.face_normal(1));
    ASSERT_EQ(kFaceNormalV0V3V2_, mesh_M_.face_normal(2));
    ASSERT_EQ(kFaceNormalV1V2V3_, mesh_M_.face_normal(3));
  }

 protected:
  const TriangleSurfaceMesh<double> mesh_M_;
  const Bvh<Obb, TriangleSurfaceMesh<double>> bvh_M_;
  const FeatureNormalSet mesh_normal_M_;
  const double kEps_{std::numeric_limits<double>::epsilon()};
  const Vector3d kFaceNormalV0V2V1_{-Vector3d::UnitZ()};
  const Vector3d kFaceNormalV0V1V3_{-Vector3d::UnitY()};
  const Vector3d kFaceNormalV0V3V2_{-Vector3d::UnitX()};
  const Vector3d kFaceNormalV1V2V3_{Vector3d::Ones().normalized()};
};

TEST_F(CalcSignedDistanceToSurfaceMesh_Positive, Triangle) {
  // Q is 0.3 meters below the bottom face v0v2v1.
  const Vector3d p_MQ(0.1, 0.2, -0.3);
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);

  // The nearest point is the projection of Q onto the bottom face in the
  // X-Y plane.
  EXPECT_TRUE(CompareMatrices(d.nearest_point, Vector3d(0.1, 0.2, 0), kEps_));
  EXPECT_NEAR(d.signed_distance, 0.3, kEps_);
  EXPECT_TRUE(CompareMatrices(d.gradient, -Vector3d::UnitZ(), kEps_));
}

// This test also illustrates why it is a good idea to use the average
// normal when the nearest point is in an edge.
TEST_F(CalcSignedDistanceToSurfaceMesh_Positive, Edge) {
  const Vector3d kMidV1V2 = (mesh_M_.vertex(1) + mesh_M_.vertex(2)) / 2;
  // Set up the query point Q by a translation from the midpoint of
  // edge v0v1 in an outward direction perpendicular to the edge.
  const Vector3d p_MQ = kMidV1V2 + Vector3d(0.1, 0.1, -2);
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);

  EXPECT_EQ(d.nearest_point, kMidV1V2);
  EXPECT_EQ(d.signed_distance, (p_MQ - kMidV1V2).norm());
  EXPECT_EQ(d.gradient, (p_MQ - kMidV1V2).normalized());

  // It is a good idea to use the average face normal because
  // the two face normals do not agree on the sign of the dot products.
  // Here we can use a large tolerance 0.1 because we are interested mostly in
  // the sign of the dot product to assign the sign of the distance.
  EXPECT_NEAR((p_MQ - kMidV1V2).dot(kFaceNormalV0V2V1_), 2, 0.1);
  EXPECT_NEAR((p_MQ - kMidV1V2).dot(kFaceNormalV1V2V3_), -1, 0.1);
}

// This test also illustrates why it is a good idea to use the average normal
// when the nearest point is at a vertex.
TEST_F(CalcSignedDistanceToSurfaceMesh_Positive, Vertex) {
  const Vector3d p_MQ = mesh_M_.vertex(3) + Vector3d(0.1, 0.1, 0.1);
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);

  const Vector3d kNearest = mesh_M_.vertex(3);
  EXPECT_EQ(d.nearest_point, kNearest);
  EXPECT_EQ(d.signed_distance, (p_MQ - kNearest).norm());
  EXPECT_EQ(d.gradient, (p_MQ - kNearest).normalized());

  // It is a good idea to use the average face normal because
  // the three face normals do not agree on the sign of the dot products.
  EXPECT_NEAR((p_MQ - kNearest).dot(kFaceNormalV1V2V3_), 0.173, 0.01);
  EXPECT_NEAR((p_MQ - kNearest).dot(kFaceNormalV0V1V3_), -0.1, 0.01);
  EXPECT_NEAR((p_MQ - kNearest).dot(kFaceNormalV0V3V2_), -0.1, 0.01);
}

// This test fixture covers cases of negative signed distances with nearest
// points in triangles, edges, or vertices.  We need a non-convex
// mesh for negative signed distances with nearest points in edges or vertices.
class CalcSignedDistanceToSurfaceMesh_Negative : public ::testing::Test {
 public:
  CalcSignedDistanceToSurfaceMesh_Negative()
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

// TODO(DamrongGuoy): Triangle case

TEST_F(CalcSignedDistanceToSurfaceMesh_Negative, Edge) {
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

TEST_F(CalcSignedDistanceToSurfaceMesh_Negative, Vertex) {
  const Vector3d p_MQ = mesh_M_.vertex(0) + Vector3d(-0.005, -0.005, -0.005);
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);

  const Vector3d kExpectNearestPoint = mesh_M_.vertex(0);
  EXPECT_EQ(d.nearest_point, kExpectNearestPoint);
  EXPECT_EQ(d.signed_distance, -(p_MQ - kExpectNearestPoint).norm());
  EXPECT_EQ(d.gradient, (kExpectNearestPoint - p_MQ).normalized());
}

// TODO(DamrongGuoy): Special cases of Zero{Triangle, Edge, Vertex}
/*

TEST_F(CalcSignedDistanceToSurfaceMesh_Positive, ZeroNearEdge) {
  // The query point is at the middle of edge v1v2.
  const Vector3d p_MQ = (mesh_M_.vertex(1) + mesh_M_.vertex(2)) / 2;

  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);

  EXPECT_EQ(d.nearest_point, p_MQ);
  EXPECT_EQ(d.signed_distance, 0);
  EXPECT_EQ(d.gradient, mesh_normal_M_.edge_normal({1, 2}));
}

TEST_F(CalcSignedDistanceToSurfaceMesh_Positive, ZeroNearVertex) {
  // The query point is at a vertex. The nearest point is the
  // query point itself. The signed distance is zero. The gradient is
  // selected somewhat arbitrarily.
  const Vector3d p_MQ = mesh_M_.vertex(0);
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);

  EXPECT_NEAR(d.signed_distance, 0, kEps_);
  EXPECT_TRUE(CompareMatrices(d.nearest_point, p_MQ, kEps_));
  EXPECT_TRUE(
      CompareMatrices(d.gradient, Vector3d(-1, -1, -1).normalized(), kEps_));
}
*/

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
