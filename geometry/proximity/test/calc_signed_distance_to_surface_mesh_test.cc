#include "drake/geometry/proximity/calc_signed_distance_to_surface_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

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

// The document of CalcSignedDistanceToSurfaceMesh() shows a table of many
// possible cases; however, we will test only a representative subset as
// shown in the following table. We pick them for code coverage. If the
// implementation changes, we might change the tests accordingly.
// We also test a few special cases when there are multiple nearest points.
//
//  |   sign   |     unique    |  location of  |      gradient      |
//  |          | nearest point | nearest point |                    |
//  | :------: | :-----------: | :-----------: | :----------------: |
//  | positive |      yes      |   triangle    | (Q-N).normalized() |
//  | negative |      yes      |     edge      | (N-Q).normalized() |
//  |   zero   |      yes      |    vertex     |    vertex normal   |
//  | :------: | :-----------: | :-----------: | :----------------: |
//  | positive |      no       |   triangle    | (Q-N).normalized() |
//  | positive |      no       |    vertex     | (Q-N).normalized() |
class CalcSignedDistanceToSurfaceMeshTest : public ::testing::Test {
 public:
  CalcSignedDistanceToSurfaceMeshTest()
      :  // First we use two tetrahedra v0v1v3v4 and v0v3v2v4 to create a
         // non-convex polytope with the concave edge v0v3. Then we convert to a
         // triangle
         // surface mesh for
         // testing. Defining two tetrahedra is easier than defining six
         // triangles.
         //
         //                       Mz
         //                       ┆     -Mx
         //            v4      v3 ●     ╱
         //              ●┄┄┄┄┄┄┄┄┆┄┄┄┄+
         //             ╱         ┆   ╱
         //            ╱          ┆  ╱
         //           ╱           ┆ ╱
         //          ╱         v0 ┆╱            v2
         //  -My ┄┄┄+┄┄┄┄┄┄┄┄┄┄┄┄┄●┄┄┄┄┄┄┄┄┄┄┄┄┄●┄┄┄ My
         //                      ╱
         //                     ╱
         //                    ╱
         //                   ╱
         //               v1 ●
         //                 ╱
         //                Mx
         //
        p_MV0_{0, 0, 0},
        p_MV1_{1, 0, 0},
        p_MV2_{0, 1, 0},
        p_MV3_{0, 0, 1},
        p_MV4_{-1, -1, 0},
        two_tetrahedra_M_{
            {VolumeElement{0, 1, 3, 4}, VolumeElement{0, 3, 2, 4}},
            {p_MV0_, p_MV1_, p_MV2_, p_MV3_, p_MV4_}},
        mesh_M_(ConvertVolumeToSurfaceMesh(two_tetrahedra_M_)),
        bvh_M_(mesh_M_),
        mesh_normal_M_(mesh_M_) {}

 protected:
  const Vector3d p_MV0_;
  const Vector3d p_MV1_;
  const Vector3d p_MV2_;
  const Vector3d p_MV3_;
  const Vector3d p_MV4_;
  const VolumeMesh<double> two_tetrahedra_M_;
  const TriangleSurfaceMesh<double> mesh_M_;
  const Bvh<Obb, TriangleSurfaceMesh<double>> bvh_M_;
  const FeatureNormalSet mesh_normal_M_;
  const double kEps_{std::numeric_limits<double>::epsilon() * (1 << 4)};
};

// Positive signed distance with nearest point in a triangle.
TEST_F(CalcSignedDistanceToSurfaceMeshTest, PositiveTriangle) {
  const Vector3d kCentroidTriangleV1V3V4 = (p_MV1_ + p_MV3_ + p_MV4_) / 3;
  // Outward normal of the triangle
  const Vector3d nhat_M = (p_MV3_ - p_MV1_).cross(p_MV4_ - p_MV1_).normalized();
  // Set up the query point Q by translation from the centroid along the
  // outward normal for an arbitrary distance.
  const Vector3d p_MQ = kCentroidTriangleV1V3V4 + 0.25 * nhat_M;
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);
  EXPECT_TRUE(CompareMatrices(d.nearest_point, kCentroidTriangleV1V3V4, kEps_));
  EXPECT_NEAR(d.signed_distance, (d.nearest_point - p_MQ).norm(), kEps_);
  EXPECT_TRUE(CompareMatrices(d.gradient, (p_MQ - d.nearest_point).normalized(),
                              kEps_));
}

// Negative signed distance with nearest point in an edge. This case needs
// the concave edge v0v3.
TEST_F(CalcSignedDistanceToSurfaceMeshTest, NegativeEdge) {
  const Vector3d kMidPointEdgeV0V3 = (p_MV0_ + p_MV3_) / 2;
  // Set up the query point Q by a small translation from the midpoint of
  // the concave edge v0v3 in an inward direction perpendicular to the edge.
  // Any small translation with negative X, negative Y, and zero Z
  // would work.
  const Vector3d p_MQ = kMidPointEdgeV0V3 + Vector3d(-0.01, -0.02, 0);
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);
  EXPECT_TRUE(CompareMatrices(d.nearest_point, kMidPointEdgeV0V3, kEps_));
  EXPECT_NEAR(d.signed_distance, -(d.nearest_point - p_MQ).norm(), kEps_);
  EXPECT_TRUE(CompareMatrices(d.gradient, (d.nearest_point - p_MQ).normalized(),
                              kEps_));
}

// Zero signed distance with nearest point at a vertex.
TEST_F(CalcSignedDistanceToSurfaceMeshTest, ZeroVertex) {
  // Test the non-convex vertex v3.
  const Vector3d p_MQ = p_MV3_;
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);
  EXPECT_TRUE(CompareMatrices(d.nearest_point, p_MV3_, kEps_));
  EXPECT_NEAR(d.signed_distance, 0, kEps_);
  // Sanity check that vertex 3 in the mesh is indeed at p_MV3_, so we can
  // query the mesh_normal_M_. We are assuming that when we called
  // ConvertVolumeToSurfaceMesh() in the constructor, it preserved the vertices.
  ASSERT_EQ(mesh_M_.vertex(3), p_MV3_);
  EXPECT_TRUE(
      CompareMatrices(d.gradient, mesh_normal_M_.vertex_normal(3), kEps_));
}

// Nearest points in multiple triangles.
TEST_F(CalcSignedDistanceToSurfaceMeshTest, MultipleTriangles) {
  // Q is equally far from two faces v0v3v1 and v0v2v3 on XZ-plane and
  // YZ-plane respectively.
  const Vector3d p_MQ(0.1, 0.1, 0.1);
  // The nearest point N1 is in triangle v0v3v1 in XZ-plane.
  const Vector3d p_MN1(0.1, 0, 0.1);
  // The nearest point N2 is in triangle v0v2v3 in YZ-plane.
  const Vector3d p_MN2(0, 0.1, 0.1);
  // Both N1 and N2 are 0.1 meters from Q.
  const double kExpectDistance = 0.1;
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);
  EXPECT_NEAR(d.signed_distance, kExpectDistance, kEps_);
  EXPECT_TRUE(CompareMatrices(d.nearest_point, p_MN1, kEps_) ||
              CompareMatrices(d.nearest_point, p_MN2, kEps_));
  EXPECT_TRUE(CompareMatrices(d.gradient, (p_MQ - d.nearest_point).normalized(),
                              kEps_));
}

// Nearest points at multiple vertices. This case needs a non-convex mesh.
TEST_F(CalcSignedDistanceToSurfaceMeshTest, MultipleVertices) {
  // Q is equally far from two vertices v1 and v2 on X-axis and Y-axis
  // respectively.
  const Vector3d p_MQ(1.5, 1.5, 0);
  const double dist1 = (p_MQ - p_MV1_).norm();
  const double dist2 = (p_MQ - p_MV2_).norm();
  // Sanity check that v1 and v2 are at the same distance from Q.
  ASSERT_EQ(dist1, dist2);
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);
  EXPECT_TRUE(CompareMatrices(d.nearest_point, p_MV1_, kEps_) ||
              CompareMatrices(d.nearest_point, p_MV2_, kEps_));
  EXPECT_NEAR(d.signed_distance, dist1, kEps_);
  EXPECT_TRUE(CompareMatrices(d.gradient, (p_MQ - d.nearest_point).normalized(),
                              kEps_));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
