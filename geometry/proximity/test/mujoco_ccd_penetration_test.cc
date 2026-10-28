#include "drake/geometry/proximity/mujoco_ccd_penetration.h"

#include <algorithm>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/mujoco_ccd_mesh_data.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

/* Makes an axis-aligned box [-h, h]³ (scaled per axis) as a
 PolygonSurfaceMesh with six quadrilateral faces, wound counterclockwise when
 viewed from outside — the same convention MakeConvexHull() produces. */
PolygonSurfaceMesh<double> MakeBoxMesh(const Vector3d& h) {
  std::vector<Vector3d> vertices = {
      {-h.x(), -h.y(), -h.z()}, {h.x(), -h.y(), -h.z()}, {h.x(), h.y(), -h.z()},
      {-h.x(), h.y(), -h.z()},  {-h.x(), -h.y(), h.z()}, {h.x(), -h.y(), h.z()},
      {h.x(), h.y(), h.z()},    {-h.x(), h.y(), h.z()}};
  // clang-format off
  std::vector<int> face_data = {
      4, 4, 5, 6, 7,   // +z
      4, 0, 3, 2, 1,   // -z
      4, 1, 2, 6, 5,   // +x
      4, 0, 4, 7, 3,   // -x
      4, 2, 3, 7, 6,   // +y
      4, 0, 1, 5, 4};  // -y
  // clang-format on
  return PolygonSurfaceMesh<double>(std::move(face_data), std::move(vertices));
}

/* The same box, but with each quad split into two triangles; exercises the
 quantized-normal merge in MakeMujocoCcdMeshData(). */
PolygonSurfaceMesh<double> MakeTriangulatedBoxMesh(const Vector3d& h) {
  const PolygonSurfaceMesh<double> quads = MakeBoxMesh(h);
  std::vector<Vector3d> vertices;
  for (int v = 0; v < quads.num_vertices(); ++v) {
    vertices.push_back(quads.vertex(v));
  }
  std::vector<int> face_data;
  for (int f = 0; f < quads.num_faces(); ++f) {
    const SurfacePolygon face = quads.element(f);
    const int a = face.vertex(0), b = face.vertex(1), c = face.vertex(2),
              d = face.vertex(3);
    face_data.insert(face_data.end(), {3, a, b, c});
    face_data.insert(face_data.end(), {3, a, c, d});
  }
  return PolygonSurfaceMesh<double>(std::move(face_data), std::move(vertices));
}

GTEST_TEST(MujocoCcdMeshDataTest, CubeTables) {
  const MujocoCcdMeshData data =
      MakeMujocoCcdMeshData(MakeBoxMesh(Vector3d::Ones()));
  EXPECT_EQ(data.nvert, 8);
  EXPECT_EQ(data.polynum, 6);
  EXPECT_EQ(data.npolygonmax, 4);
  // Every cube vertex borders exactly three faces.
  EXPECT_EQ(data.nmeshdegmax, 3);
  for (int v = 0; v < data.nvert; ++v) {
    EXPECT_EQ(data.polymapnum[v], 3);
  }
  EXPECT_TRUE(CompareMatrices(data.centroid, Vector3d::Zero(), 1e-14));
  // Each polygon's stored winding must produce the stored outward normal
  // (right-hand rule), and the normal must have unit length.
  for (int p = 0; p < data.polynum; ++p) {
    const Vector3d n(data.polynormal[3 * p], data.polynormal[3 * p + 1],
                     data.polynormal[3 * p + 2]);
    EXPECT_NEAR(n.norm(), 1.0, 1e-12);
    const int adr = data.polyvertadr[p];
    auto vert = [&](int i) {
      const int vi = data.polyvert[adr + i];
      return Vector3d(data.vert[3 * vi], data.vert[3 * vi + 1],
                      data.vert[3 * vi + 2]);
    };
    const Vector3d winding_normal =
        (vert(1) - vert(0)).cross(vert(2) - vert(1)).normalized();
    // The stored normal is computed from the polygon's own vertices (as
    // MuJoCo's MakePolygonNormals does), so it matches the winding exactly.
    EXPECT_GT(winding_normal.dot(n), 1.0 - 1e-12);
  }
}

GTEST_TEST(MujocoCcdMeshDataTest, TriangulatedCubeMerges) {
  const MujocoCcdMeshData data =
      MakeMujocoCcdMeshData(MakeTriangulatedBoxMesh(Vector3d::Ones()));
  // The twelve coplanar triangles merge back into six quadrilaterals.
  EXPECT_EQ(data.polynum, 6);
  EXPECT_EQ(data.npolygonmax, 4);
  EXPECT_EQ(data.nmeshdegmax, 3);
}

class MujocoCcdPenetrationTest : public ::testing::Test {
 protected:
  std::vector<PenetrationAsPointPair<double>> Compute(
      const MujocoCcdMeshData& mesh_A, const RigidTransformd& X_WA,
      const MujocoCcdMeshData& mesh_B, const RigidTransformd& X_WB) {
    std::vector<PenetrationAsPointPair<double>> pairs;
    resolved_ = ComputeMujocoMultipointPenetration(
        mesh_A, X_WA, mesh_B, X_WB, id_A_, id_B_, &scratch_, &pairs);
    return pairs;
  }

  static void CheckInvariants(
      const std::vector<PenetrationAsPointPair<double>>& pairs) {
    for (const auto& pair : pairs) {
      EXPECT_GT(pair.depth, 0.0);
      EXPECT_NEAR(pair.nhat_BA_W.norm(), 1.0, 1e-12);
      EXPECT_NEAR((pair.p_WCb - pair.p_WCa).dot(pair.nhat_BA_W), pair.depth,
                  1e-9);
    }
  }

  const GeometryId id_A_{GeometryId::get_new_id()};
  const GeometryId id_B_{GeometryId::get_new_id()};
  std::vector<double> scratch_;
  // The return value of the most recent Compute() call.
  bool resolved_{false};
};

/* Two unit cubes in face-face contact, laterally offset so the overlap
 region is a proper rectangle: the manifold must contain four points at the
 corners of the overlap rectangle, each with (approximately) the full
 penetration depth. This is the case where MuJoCo's manifold differs the most
 from a single-point contact. */
TEST_F(MujocoCcdPenetrationTest, FaceFaceContact) {
  const MujocoCcdMeshData cube =
      MakeMujocoCcdMeshData(MakeBoxMesh(Vector3d::Ones()));
  const double kDepth = 1e-3;
  const RigidTransformd X_WA;  // Bottom cube: z ∈ [-1, 1].
  // Top cube: z ∈ [1 - kDepth, 3 - kDepth], shifted in x and y.
  const RigidTransformd X_WB(Vector3d(0.4, 0.3, 2.0 - kDepth));

  const auto pairs = Compute(cube, X_WA, cube, X_WB);
  EXPECT_TRUE(resolved_);
  ASSERT_EQ(ssize(pairs), 4);
  CheckInvariants(pairs);

  // The overlap rectangle is x ∈ [-0.6, 1] × y ∈ [-0.7, 1] at z ≈ 1.
  // (Vertices pass through single precision, so expect ~1e-7 noise.)
  const double kTol = 1e-6;
  std::vector<Vector3d> corners = {
      {-0.6, -0.7, 0}, {1, -0.7, 0}, {1, 1, 0}, {-0.6, 1, 0}};
  for (const auto& pair : pairs) {
    EXPECT_NEAR(pair.depth, kDepth, 1e-8);
    // Normal points out of B (top cube) into A (bottom cube): -z.
    EXPECT_TRUE(CompareMatrices(pair.nhat_BA_W, -Vector3d::UnitZ(), 1e-9));
    // Witness on A is on the bottom cube's top face (z = 1); witness on B on
    // the top cube's bottom face (z = 1 - kDepth).
    EXPECT_NEAR(pair.p_WCa.z(), 1.0, kTol);
    EXPECT_NEAR(pair.p_WCb.z(), 1.0 - kDepth, kTol);
    // Each contact lies at one of the overlap-rectangle corners.
    const auto is_close = [&](const Vector3d& c) {
      return (pair.p_WCb.head<2>() - c.head<2>()).norm() < 1e-5;
    };
    EXPECT_TRUE(std::any_of(corners.begin(), corners.end(), is_close))
        << "Unexpected contact at (" << pair.p_WCb.x() << ", " << pair.p_WCb.y()
        << ")";
  }
}

/* A cube rotated 45° about z on top of another: the clipped contact polygon
 is an octagon, which MuJoCo prunes to its maximum-area quadrilateral. */
TEST_F(MujocoCcdPenetrationTest, RotatedFaceFaceContactPrunesToFour) {
  const MujocoCcdMeshData cube =
      MakeMujocoCcdMeshData(MakeBoxMesh(Vector3d::Ones()));
  const double kDepth = 1e-3;
  const RigidTransformd X_WA;
  const RigidTransformd X_WB(RotationMatrixd::MakeZRotation(M_PI / 4),
                             Vector3d(0, 0, 2.0 - kDepth));

  const auto pairs = Compute(cube, X_WA, cube, X_WB);
  ASSERT_EQ(ssize(pairs), 4);
  CheckInvariants(pairs);
  for (const auto& pair : pairs) {
    EXPECT_NEAR(pair.depth, kDepth, 1e-8);
    EXPECT_TRUE(CompareMatrices(pair.nhat_BA_W, -Vector3d::UnitZ(), 1e-9));
  }
}

/* A cube rotated 45° about x rests edge-down on the other's top face:
 edge-face contact produces exactly two contact points, the ends of the
 penetrating edge segment. */
TEST_F(MujocoCcdPenetrationTest, EdgeFaceContact) {
  const MujocoCcdMeshData cube =
      MakeMujocoCcdMeshData(MakeBoxMesh(Vector3d::Ones()));
  const double kDepth = 1e-3;
  const RigidTransformd X_WA;
  // Rotated 45° about x, the lowest feature is an edge along x at
  // z = -sqrt(2); place it kDepth below A's top face.
  const RigidTransformd X_WB(RotationMatrixd::MakeXRotation(M_PI / 4),
                             Vector3d(0, 0, 1.0 + std::sqrt(2.0) - kDepth));

  const auto pairs = Compute(cube, X_WA, cube, X_WB);
  ASSERT_EQ(ssize(pairs), 2);
  CheckInvariants(pairs);
  for (const auto& pair : pairs) {
    EXPECT_NEAR(pair.depth, kDepth, 1e-6);
    EXPECT_NEAR(pair.p_WCb.y(), 0.0, 1e-5);
    EXPECT_NEAR(std::abs(pair.p_WCb.x()), 1.0, 1e-5);
  }
}

/* A cube balanced on its corner produces a single (vertex-face) contact,
 exactly like Drake's existing single-point narrowphase. */
TEST_F(MujocoCcdPenetrationTest, VertexFaceContact) {
  const MujocoCcdMeshData cube =
      MakeMujocoCcdMeshData(MakeBoxMesh(Vector3d::Ones()));
  const double kDepth = 1e-3;
  const RigidTransformd X_WA;
  // Rotate the cube so a corner points straight down. The corner (1,1,1) has
  // direction (1,1,1)/sqrt(3); rotate that direction onto -z.
  const Eigen::Quaterniond q =
      Eigen::Quaterniond::FromTwoVectors(Vector3d(1, 1, 1), -Vector3d::UnitZ());
  const RigidTransformd X_WB(RotationMatrixd(q),
                             Vector3d(0, 0, 1.0 + std::sqrt(3.0) - kDepth));

  const auto pairs = Compute(cube, X_WA, cube, X_WB);
  ASSERT_EQ(ssize(pairs), 1);
  CheckInvariants(pairs);
  EXPECT_NEAR(pairs[0].depth, kDepth, 1e-6);
  EXPECT_NEAR(pairs[0].p_WCa.x(), 0.0, 1e-5);
  EXPECT_NEAR(pairs[0].p_WCa.y(), 0.0, 1e-5);
}

/* A slightly tilted face-face contact grades the per-point depths instead of
 assigning every point the deepest value. */
TEST_F(MujocoCcdPenetrationTest, TiltedContactHasGradedDepths) {
  const MujocoCcdMeshData cube =
      MakeMujocoCcdMeshData(MakeBoxMesh(Vector3d::Ones()));
  // The tilt is small enough that MuJoCo still treats the two faces as
  // parallel (its face-alignment tolerance is about 5 degrees), so the contact
  // is a face-face manifold rather than an edge-face pair.
  const double kTilt = 2e-4;
  const RigidTransformd X_WA;
  const RigidTransformd X_WB(RotationMatrixd::MakeXRotation(kTilt),
                             Vector3d(0, 0, 2.0 - 1e-3));

  const auto pairs = Compute(cube, X_WA, cube, X_WB);
  ASSERT_EQ(ssize(pairs), 4);
  CheckInvariants(pairs);
  double min_depth = 1.0, max_depth = 0.0;
  for (const auto& pair : pairs) {
    min_depth = std::min(min_depth, pair.depth);
    max_depth = std::max(max_depth, pair.depth);
  }
  // The tilt separates depths by roughly 2 * tan(kTilt) ≈ 4e-4.
  EXPECT_GT(max_depth - min_depth, 1e-4);
}

TEST_F(MujocoCcdPenetrationTest, SeparatedProducesNothing) {
  const MujocoCcdMeshData cube =
      MakeMujocoCcdMeshData(MakeBoxMesh(Vector3d::Ones()));
  const RigidTransformd X_WA;
  const RigidTransformd X_WB(Vector3d(0, 0, 2.5));
  EXPECT_TRUE(Compute(cube, X_WA, cube, X_WB).empty());
  // Separation is a definitive answer.
  EXPECT_TRUE(resolved_);
}

/* Two identical hulls at the same pose have coincident vertex centroids, so
 MuJoCo's GJK stalls at its starting point and reports a distance of exactly
 zero without running EPA. The function must report that it could not resolve
 the pair (appending nothing) so that the caller can fall back to another
 algorithm instead of reporting no contact for fully overlapping hulls. */
TEST_F(MujocoCcdPenetrationTest, CoincidentHullsAreUnresolved) {
  const MujocoCcdMeshData cube =
      MakeMujocoCcdMeshData(MakeBoxMesh(Vector3d::Ones()));
  const RigidTransformd X_WA;
  EXPECT_TRUE(Compute(cube, X_WA, cube, X_WA).empty());
  EXPECT_FALSE(resolved_);
}

TEST_F(MujocoCcdPenetrationTest, Deterministic) {
  const MujocoCcdMeshData cube =
      MakeMujocoCcdMeshData(MakeBoxMesh(Vector3d(0.7, 1.1, 0.9)));
  const RigidTransformd X_WA(RotationMatrixd::MakeYRotation(0.1),
                             Vector3d(0.05, -0.02, 0));
  const RigidTransformd X_WB(RotationMatrixd::MakeXRotation(-0.07),
                             Vector3d(0.3, 0.2, 1.7));
  const auto pairs1 = Compute(cube, X_WA, cube, X_WB);
  const auto pairs2 = Compute(cube, X_WA, cube, X_WB);
  ASSERT_EQ(ssize(pairs1), ssize(pairs2));
  for (int i = 0; i < ssize(pairs1); ++i) {
    EXPECT_EQ(pairs1[i].depth, pairs2[i].depth);
    EXPECT_EQ(pairs1[i].p_WCa, pairs2[i].p_WCa);
    EXPECT_EQ(pairs1[i].p_WCb, pairs2[i].p_WCb);
    EXPECT_EQ(pairs1[i].nhat_BA_W, pairs2[i].nhat_BA_W);
  }
}

/* An inward-wound input (every face loop reversed, e.g. a cached hull
 topology re-instantiated at a mirroring scale) is detected via its negative
 enclosed volume and produces the same manifold as the outward-wound cube. */
TEST_F(MujocoCcdPenetrationTest, InwardWoundInputIsRepaired) {
  const PolygonSurfaceMesh<double> quads = MakeBoxMesh(Vector3d::Ones());
  std::vector<Vector3d> vertices;
  for (int v = 0; v < quads.num_vertices(); ++v) {
    vertices.push_back(quads.vertex(v));
  }
  std::vector<int> face_data;
  for (int f = 0; f < quads.num_faces(); ++f) {
    const SurfacePolygon face = quads.element(f);
    face_data.push_back(face.num_vertices());
    for (int i = face.num_vertices() - 1; i >= 0; --i) {
      face_data.push_back(face.vertex(i));
    }
  }
  const MujocoCcdMeshData inward_cube = MakeMujocoCcdMeshData(
      PolygonSurfaceMesh<double>(std::move(face_data), std::move(vertices)));
  const MujocoCcdMeshData cube =
      MakeMujocoCcdMeshData(MakeBoxMesh(Vector3d::Ones()));
  const double kDepth = 1e-3;
  const RigidTransformd X_WA;
  const RigidTransformd X_WB(Vector3d(0.4, 0.3, 2.0 - kDepth));
  const auto pairs = Compute(inward_cube, X_WA, inward_cube, X_WB);
  const auto expected = Compute(cube, X_WA, cube, X_WB);
  ASSERT_EQ(ssize(pairs), 4);
  CheckInvariants(pairs);
  ASSERT_EQ(ssize(expected), 4);
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(CompareMatrices(pairs[i].p_WCa, expected[i].p_WCa, 1e-12));
    EXPECT_TRUE(
        CompareMatrices(pairs[i].nhat_BA_W, expected[i].nhat_BA_W, 1e-12));
    EXPECT_NEAR(pairs[i].depth, expected[i].depth, 1e-12);
  }
}

/* The triangulated cube (after the quantized-normal merge) must behave
 identically to the natively-quadrilateral cube. */
TEST_F(MujocoCcdPenetrationTest, TriangulatedCubeMatchesQuadCube) {
  const MujocoCcdMeshData quad_cube =
      MakeMujocoCcdMeshData(MakeBoxMesh(Vector3d::Ones()));
  const MujocoCcdMeshData tri_cube =
      MakeMujocoCcdMeshData(MakeTriangulatedBoxMesh(Vector3d::Ones()));
  const double kDepth = 1e-3;
  const RigidTransformd X_WA;
  const RigidTransformd X_WB(Vector3d(0.4, 0.3, 2.0 - kDepth));

  const auto pairs_quad = Compute(quad_cube, X_WA, quad_cube, X_WB);
  const auto pairs_tri = Compute(tri_cube, X_WA, tri_cube, X_WB);
  ASSERT_EQ(ssize(pairs_quad), 4);
  ASSERT_EQ(ssize(pairs_tri), 4);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
