#include "drake/geometry/proximity/calc_signed_distance_to_surface_mesh.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;

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

 protected:
  const TriangleSurfaceMesh<double> mesh_M_;
  const double kEps{std::numeric_limits<double>::epsilon()};
};

TEST_F(FeatureNormalSetTest, EdgeNormalIsAverageFaceNormal) {
  const auto dut =
      std::get<FeatureNormalSet>(FeatureNormalSet::MaybeCreate(mesh_M_));
  // Edge v1v2 is shared by face 0 (v0,v2,v1) and face 3 (v1, v2,v3).
  const Vector3d kExpectEdgeNormal =
      (mesh_M_.face_normal(0) + mesh_M_.face_normal(3)).normalized();
  EXPECT_TRUE(
      CompareMatrices(dut.edge_normal({1, 2}), kExpectEdgeNormal, kEps));
}

TEST_F(FeatureNormalSetTest, VertexNormalIsAngleWeightedAverage) {
  const auto dut =
      std::get<FeatureNormalSet>(FeatureNormalSet::MaybeCreate(mesh_M_));
  // Vertex v1 is shared by face 0 (v0,v2,v1), face 1 (v0,v1,v3), and
  // face 3 (v1,v2,v3).  We expect vertex_normal() of v1 to be their
  // angle-weighted average normal.
  //
  //   face      triangle        angle at v1 in the triangle
  //    0      {v0, v2, v1}                π/4
  //    1      {v0, v1, v3}                π/4
  //    3      {v1, v2, v3}                π/3
  const Vector3d kExpectVertexNormal =
      (M_PI_4 * mesh_M_.face_normal(0) + M_PI_4 * mesh_M_.face_normal(1) +
       (M_PI / 3) * mesh_M_.face_normal(3))
          .normalized();
  EXPECT_TRUE(CompareMatrices(dut.vertex_normal(1), kExpectVertexNormal, kEps));
}

// Test the error message when the mesh has two triangles that make a "sharp
// knife" with a very small dihedral angle.
GTEST_TEST(FeatureNormalSet, ErrorSharpKnife) {
  // This is a "flatten" tetrahedron with all four vertices on the same plane.
  // Its surface mesh has zero dihedral angle.
  //
  //              Mz
  //              ┆
  //              ┆       v2
  //           v0 ●┄┄┄┄┄┄┄●┄┄┄ My
  //             ╱       ╱
  //            ╱       ╱
  //        v1 ●┄┄┄┄┄┄┄● v3
  //         ╱
  //        Mx
  //
  const VolumeMesh<double> one_flat_tetrahedron_M(
      {VolumeElement(0, 1, 2, 3)}, {Vector3d::Zero(), Vector3d::UnitX(),
                                    Vector3d::UnitY(), Vector3d(1, 1, 0)});
  const TriangleSurfaceMesh<double> mesh_M =
      ConvertVolumeToSurfaceMesh(one_flat_tetrahedron_M);

  auto dut = FeatureNormalSet::MaybeCreate(mesh_M);
  ASSERT_TRUE(std::holds_alternative<std::string>(dut));
  EXPECT_EQ(std::get<std::string>(dut),
            "FeatureNormalSet: Cannot compute an edge normal because "
            "the two triangles sharing the edge make a very sharp edge.");
}

// Test the error message when the mesh has a "pointy, needle-like" vertex.
GTEST_TEST(FeatureNormalSet, ErrorPointyNeedleVertex) {
  // This scaling factor will creat a "needle" with aspect ratio 1:100.
  const double kSmallBase = 1e-2;
  // The apex vertex v3 becomes very pointy as the base triangle v0v1v2
  // shrinks.
  //
  //              Mz
  //              ┆
  //           v3 ●
  //              ┆
  //              ┆
  //              ┆
  //              ┆
  //              ┆   v2
  //           v0 ●┄┄┄●┄┄┄ My
  //             ╱
  //         v1 ●
  //          ╱
  //        Mx
  //
  const VolumeMesh<double> needle_tetrahedron_M(
      {VolumeElement(0, 1, 2, 3)},
      {Vector3d::Zero(), kSmallBase * Vector3d::UnitX(),
       kSmallBase * Vector3d::UnitY(), Vector3d::UnitZ()});
  const TriangleSurfaceMesh<double> mesh_M =
      ConvertVolumeToSurfaceMesh(needle_tetrahedron_M);

  auto dut = FeatureNormalSet::MaybeCreate(mesh_M);
  ASSERT_TRUE(std::holds_alternative<std::string>(dut));
  EXPECT_EQ(std::get<std::string>(dut),
            "FeatureNormalSet: Cannot compute a vertex normal because "
            "the triangles sharing the vertex form a very pointy needle.");
}

class CalcSquaredDistanceToTriangleTest : public ::testing::Test {
 public:
  CalcSquaredDistanceToTriangleTest()
      :  // We will test the code in a general frame M of the mesh, but first
         // we define the vertices in a convenient frame F like this:
         //
         //              Fz
         //              ┆
         //           v3 ● 0.1
         //              ┆
         //              ┆
         //           v1 ●┄┄┄┄┄┄┄0.1┄┄┄┄ Fy
         //             ╱
         //            ╱
         //        v2 ●┄┄┄┄┄┄┄● v0
         //         ╱
         //        Fx
         //
        p_FV0_(0.1, 0.1, 0),
        p_FV1_(0, 0, 0),
        p_FV2_(0.1, 0, 0),
        p_FV3_(0, 0, 0.1),
        // A transform that expresses the convenient frame F in a general
        // frame M of the mesh.
        X_MF_(RollPitchYawd(M_PI, M_PI_2, M_PI_4), Vector3d(0.5, 1, 2)),
        mesh_M_(
            std::vector<SurfaceTriangle>{
                {0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}},
            {X_MF_ * p_FV0_, X_MF_ * p_FV1_, X_MF_ * p_FV2_, X_MF_ * p_FV3_}),
        normal_set_M_{std::get<FeatureNormalSet>(
            FeatureNormalSet::MaybeCreate(mesh_M_))} {}

 protected:
  const Vector3d p_FV0_;
  const Vector3d p_FV1_;
  const Vector3d p_FV2_;
  const Vector3d p_FV3_;
  const RigidTransformd X_MF_;
  const TriangleSurfaceMesh<double> mesh_M_;
  const FeatureNormalSet normal_set_M_;
  const double kEps{std::numeric_limits<double>::epsilon() * (1 << 4)};
};

TEST_F(CalcSquaredDistanceToTriangleTest, Inside) {
  // The query point Q projects to N inside the triangle.
  //
  //                   Fy
  //                   ┆
  //                   0.1          v0
  //                   ┆           ╱│
  //                   ┆        ╱   │
  //                   ┆     ╱      │
  //                   ┆  ╱  ● N    │
  //                  v1┄┄┄┄┄┄┄┄┄┄┄┄0.1┄┄┄┄┄┄┄┄ Fx
  //                                v2
  //
  // Q is 15 meters above the triangle in frame F.
  const Vector3d p_MQ = X_MF_ * Vector3d(0.04, 0.02, 15);
  // The projection N is the closest point.
  const Vector3d p_MN = X_MF_ * Vector3d(0.04, 0.02, 0);
  // The face normal in frame F.
  const Vector3d nhat_F = -Vector3d::UnitZ();
  const Vector3d nhat_M = X_MF_.rotation() * nhat_F;

  const SquaredDistanceToTriangle dut =
      CalcSquaredDistanceToTriangle(p_MQ, 0, mesh_M_, normal_set_M_);
  EXPECT_NEAR(dut.squared_distance, (p_MQ - p_MN).squaredNorm(), kEps);
  EXPECT_TRUE(CompareMatrices(dut.closest_point, p_MN, kEps));
  EXPECT_TRUE(CompareMatrices(dut.feature_normal, nhat_M, kEps));
}

TEST_F(CalcSquaredDistanceToTriangleTest, OutsideNearEdge) {
  // The query point Q projects to Q' outside the triangle nearest to
  // the edge v1v2.  N is the closest point on the edge v1v2.
  // The area between the two rays is the region of such projections.
  //
  //                   Fy
  //                   ┆
  //                   0.1          v0
  //                   ┆           ╱│
  //                   ┆        ╱   │
  //                   ┆     ╱      │
  //                   ┆  ╱  N      │
  //                  v1┄┄┄┄●┄┄┄┄┄┄┄v2┄┄┄┄┄┄┄┄ Fx
  //                   ↓    ● Q'    ↓
  //                   ↓            ↓
  //                   ↓            ↓
  //
  // Q is 10 meters below the triangle in frame F.
  const Vector3d p_MQ = X_MF_ * Vector3d(0.04, -0.02, -10);
  // Q projects onto the plane of the triangle at Q'(0.04, -0.02, 0) in frame
  // F. The projection Q' is outside the triangle and projects onto the
  // edge v1v2 at the closest point N(0.04, 0, 0) in frame F.
  const Vector3d p_MN = X_MF_ * Vector3d(0.04, 0, 0);

  const SquaredDistanceToTriangle dut =
      CalcSquaredDistanceToTriangle(p_MQ, 0, mesh_M_, normal_set_M_);
  EXPECT_NEAR(dut.squared_distance, (p_MQ - p_MN).squaredNorm(), kEps);
  EXPECT_TRUE(CompareMatrices(dut.closest_point, p_MN, kEps));
  EXPECT_TRUE(CompareMatrices(dut.feature_normal,
                              normal_set_M_.edge_normal({1, 2}), kEps));
}

TEST_F(CalcSquaredDistanceToTriangleTest, OutsideNearVertex) {
  // Q' is the projection of the query point Q onto the plane of the
  // triangle.  Q' is outside the triangle nearest to vertex v1.
  // The area between the two rays is the region of such projections.
  //
  //                   Fy
  //                   ┆
  //                   0.1          v0
  //      ↖            ┆           ╱│
  //         ↖         ┆        ╱   │
  //            ↖      ┆     ╱      │
  //               ↖   ┆  ╱         │
  //                  v1 ┄┄┄┄┄┄┄┄┄┄┄v2┄┄┄┄┄┄┄┄ Fx
  //       Q'          ↓
  //                   ↓
  //                   ↓
  //                   ↓
  //
  // Q is 5 meters above the triangle in frame F.
  const Vector3d p_MQ = X_MF_ * Vector3d(-0.08, -0.02, 5);
  const SquaredDistanceToTriangle dut =
      CalcSquaredDistanceToTriangle(p_MQ, 0, mesh_M_, normal_set_M_);

  EXPECT_NEAR(dut.squared_distance, (p_MQ - mesh_M_.vertex(1)).squaredNorm(),
              kEps);
  EXPECT_TRUE(CompareMatrices(dut.closest_point, mesh_M_.vertex(1), kEps));
  EXPECT_TRUE(CompareMatrices(dut.feature_normal,
                              normal_set_M_.vertex_normal(1), kEps));
}

// The document of CalcSignedDistanceToSurfaceMesh() shows a table of many
// possible cases; however, we will test only a representative subset as
// shown in the following table. We pick them for code coverage. If the
// implementation changes, we might change the tests accordingly.
// We also test a few special cases when there are multiple nearest points.
//
//  |   sign   | unique nearest|  location of  |      gradient      |
//  |          | point N       | nearest point |                    |
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
      :  // These coordinates are simply ones and zeros, but they define
         // complicated normals at vertices, edges, and faces of the mesh.
         // The main numerics in the code involves dot products with
         // the normals.
        p_MV0_{0, 0, 0},
        p_MV1_{1, 0, 0},
        p_MV2_{0, 1, 0},
        p_MV3_{0, 0, 1},
        p_MV4_{-1, -1, 0},
        // We use two tetrahedra v0v1v3v4 and v0v3v2v4 to create a
        // non-convex polytope and convert it to a triangle
        // surface mesh for testing. Defining two tetrahedra is more
        // convenient than defining six triangles.
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
        mesh_M_(ConvertVolumeToSurfaceMesh(VolumeMesh<double>(
            {VolumeElement{0, 1, 3, 4}, VolumeElement{0, 3, 2, 4}},
            {p_MV0_, p_MV1_, p_MV2_, p_MV3_, p_MV4_}))),
        bvh_M_(mesh_M_),
        mesh_normal_M_(std::get<FeatureNormalSet>(
            FeatureNormalSet::MaybeCreate(mesh_M_))) {}

 protected:
  const Vector3d p_MV0_;
  const Vector3d p_MV1_;
  const Vector3d p_MV2_;
  const Vector3d p_MV3_;
  const Vector3d p_MV4_;
  const TriangleSurfaceMesh<double> mesh_M_;
  const Bvh<Obb, TriangleSurfaceMesh<double>> bvh_M_;
  const FeatureNormalSet mesh_normal_M_;
  const double kEps{std::numeric_limits<double>::epsilon() * (1 << 4)};
};

// Positive signed distance with nearest point in a triangle.
TEST_F(CalcSignedDistanceToSurfaceMeshTest, PositiveTriangle) {
  const Vector3d p_MC = mesh_M_.element_centroid(0);
  const Vector3d n_M = mesh_M_.face_normal(0);
  // We don't know which one is face 0, but we know that a small
  // translation from its centroid along its outward face normal keeps
  // the nearest point at its centroid. All triangles, except the
  // triangles v0v3v1 and v0v2v3, have no limit on such translation distance.
  // Each of the two triangles limits the translation distance to 1/3 before
  // the nearest point switches, so we can use 1/4 safely.
  const Vector3d p_MQ = p_MC + 0.25 * n_M;
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);
  EXPECT_TRUE(CompareMatrices(d.nearest_point, p_MC, kEps));
  EXPECT_NEAR(d.signed_distance, 0.25, kEps);
  EXPECT_TRUE(CompareMatrices(d.gradient, n_M, kEps));
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
  EXPECT_TRUE(CompareMatrices(d.nearest_point, kMidPointEdgeV0V3, kEps));
  EXPECT_NEAR(d.signed_distance, -(d.nearest_point - p_MQ).norm(), kEps);
  EXPECT_TRUE(
      CompareMatrices(d.gradient, (d.nearest_point - p_MQ).normalized(), kEps));
}

// Zero signed distance with nearest point at a vertex.
TEST_F(CalcSignedDistanceToSurfaceMeshTest, ZeroVertex) {
  // Test the non-convex vertex v3.
  const Vector3d p_MQ = p_MV3_;
  const SignedDistanceToSurfaceMesh d =
      CalcSignedDistanceToSurfaceMesh(p_MQ, mesh_M_, bvh_M_, mesh_normal_M_);
  EXPECT_TRUE(CompareMatrices(d.nearest_point, p_MV3_, kEps));
  EXPECT_NEAR(d.signed_distance, 0, kEps);
  // Sanity check that vertex 3 in the mesh is indeed at p_MV3_, so we can
  // query the mesh_normal_M_. We are assuming that when we called
  // ConvertVolumeToSurfaceMesh() in the constructor, it preserved the vertices.
  ASSERT_EQ(mesh_M_.vertex(3), p_MV3_);
  EXPECT_TRUE(
      CompareMatrices(d.gradient, mesh_normal_M_.vertex_normal(3), kEps));
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
  EXPECT_NEAR(d.signed_distance, kExpectDistance, kEps);
  EXPECT_TRUE(CompareMatrices(d.nearest_point, p_MN1, kEps) ||
              CompareMatrices(d.nearest_point, p_MN2, kEps));
  EXPECT_TRUE(
      CompareMatrices(d.gradient, (p_MQ - d.nearest_point).normalized(), kEps));
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
  EXPECT_TRUE(CompareMatrices(d.nearest_point, p_MV1_, kEps) ||
              CompareMatrices(d.nearest_point, p_MV2_, kEps));
  EXPECT_NEAR(d.signed_distance, dist1, kEps);
  EXPECT_TRUE(
      CompareMatrices(d.gradient, (p_MQ - d.nearest_point).normalized(), kEps));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
