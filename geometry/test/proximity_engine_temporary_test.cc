#include "drake/geometry/proximity_engine.h"

// TODO(DamrongGuoy) Remove this file when I finish development of
//  ComputeSignedDistanceToPoint for meshes.  For now, I use it to test my
//  prototype quickly.

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/query_results/signed_distance_to_point.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using math::RigidTransformd;

using Eigen::Vector3d;

GTEST_TEST(ComputeSignedDistanceToPoint, NonConvexTetrahedralMesh) {
  ProximityEngine<double> engine;
  const std::string mesh_path =
      FindResourceOrThrow("drake/geometry/test/non_convex_mesh.vtk");
  const GeometryId id = GeometryId::get_new_id();
  const std::unordered_map<GeometryId, math::RigidTransformd> X_WGs{
      {id, RigidTransformd::Identity()}};
  engine.AddDynamicGeometry(Mesh(mesh_path), {}, id);
  engine.UpdateWorldPoses(X_WGs);

  // This query point is -(.05, 0.05, 0.05) from the inflection
  // vertex (0.2, 0.2, 0.2) of the non-convex mesh.  The nearest point p_GN
  // will be at this inflection vertex.
  const Vector3d p_WQ{0.15, 0.15, 0.15};
  const double threshold = std::numeric_limits<double>::infinity();
  std::vector<SignedDistanceToPoint<double>> results =
      engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs, threshold);

  ASSERT_EQ(results.size(), 1);
  const SignedDistanceToPoint<double>& d = results[0];
  EXPECT_EQ(d.id_G, id);
  EXPECT_TRUE(CompareMatrices(d.p_GN, Vector3d(0.2, 0.2, 0.2), 1e-14));
  EXPECT_NEAR(d.distance, -0.05 * std::sqrt(3), 1e-14);
  EXPECT_TRUE(CompareMatrices(d.grad_W, Vector3d(1, 1, 1).normalized(), 1e-14));
}

GTEST_TEST(ComputeSignedDistanceToPoint, NonConvexTriangleMesh) {
  ProximityEngine<double> engine;
  const std::string mesh_path =
      FindResourceOrThrow("drake/geometry/test/cube_with_hole.obj");
  const GeometryId id = GeometryId::get_new_id();
  const std::unordered_map<GeometryId, math::RigidTransformd> X_WGs{
      {id, RigidTransformd::Identity()}};
  engine.AddDynamicGeometry(Mesh(mesh_path), {}, id);
  engine.UpdateWorldPoses(X_WGs);

  // This query point is outside near the inner wall of the hole. The
  // wall is at X = 0.499164, so the nearest point should be on the wall
  // at (0.499164, 0, 0), the signed distance should be positive
  // 0.499164 - 0.25 = 0.249164, and the gradient should be in -X direction.
  const Vector3d p_WQ{0.25, 0, 0};
  const double threshold = std::numeric_limits<double>::infinity();
  std::vector<SignedDistanceToPoint<double>> results =
      engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs, threshold);

  ASSERT_EQ(results.size(), 1);
  const SignedDistanceToPoint<double>& d = results[0];
  EXPECT_EQ(d.id_G, id);
  EXPECT_TRUE(CompareMatrices(d.p_GN, Vector3d(0.499164, 0, 0), 1e-14));
  EXPECT_NEAR(d.distance, 0.249164, 1e-14);
  EXPECT_TRUE(CompareMatrices(d.grad_W, -Vector3d::UnitX(), 1e-14));
}

// Use the convex hull of the same .obj file of the previous test
// of NonConvexTriangleMesh.  Since we use the convex hull, the result will
// be different from the previous test.
GTEST_TEST(ComputeSignedDistanceToPoint, ConvexHullOfTriangleMesh) {
  ProximityEngine<double> engine;
  const std::string mesh_path =
      FindResourceOrThrow("drake/geometry/test/cube_with_hole.obj");
  const GeometryId id = GeometryId::get_new_id();
  const std::unordered_map<GeometryId, math::RigidTransformd> X_WGs{
      {id, RigidTransformd::Identity()}};

  // Use Convex instead of Mesh.
  engine.AddDynamicGeometry(Convex(mesh_path), {}, id);
  engine.UpdateWorldPoses(X_WGs);

  // This query point is near the inner wall of the hole; however,
  // it has different result from the previous test because we are using
  // the convex hull, which is a unit cube [-1, 1]^3.  The outer wall is
  // at X = 1, so the nearest point should be on the wall at (1, 0, 0), the
  // signed distance should be negative of 1 - 0.25 = -0.75, and the
  // gradient should be in +X direction.
  const Vector3d p_WQ{0.25, 0, 0};
  const double threshold = std::numeric_limits<double>::infinity();
  std::vector<SignedDistanceToPoint<double>> results =
      engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs, threshold);

  ASSERT_EQ(results.size(), 1);
  const SignedDistanceToPoint<double>& d = results[0];
  EXPECT_EQ(d.id_G, id);
  EXPECT_TRUE(CompareMatrices(d.p_GN, Vector3d(1, 0, 0), 1e-14));
  EXPECT_NEAR(d.distance, -0.75, 1e-14);
  EXPECT_TRUE(CompareMatrices(d.grad_W, Vector3d::UnitX(), 1e-14));
}

// Use the convex hull of the same .vtk file of the prior test of
// NonConvexTetrahedralMesh. Since we use the convex hull, the result will be
// different from the previous test.
GTEST_TEST(ComputeSignedDistanceToPoint, ConvexHullOfTetrahedralMesh) {
  ProximityEngine<double> engine;
  const std::string mesh_path =
      FindResourceOrThrow("drake/geometry/test/non_convex_mesh.vtk");
  const GeometryId id = GeometryId::get_new_id();
  const std::unordered_map<GeometryId, math::RigidTransformd> X_WGs{
      {id, RigidTransformd::Identity()}};

  // Use Convex instead of Mesh.
  engine.AddDynamicGeometry(Convex(mesh_path), {}, id);
  engine.UpdateWorldPoses(X_WGs);

  // This query point is at the same location as the prior test of
  // NonConvexTetrahedralMesh, but the result is different. We are using the
  // convex hull, which is a standard tetrahedron. The query point is
  // inside the convex hull with three nearest points on the three standard
  // planes. In this implementation, it picked the nearest point of the X-Z
  // plane as (0.15, 0, 0.15).
  const Vector3d p_WQ{0.15, 0.15, 0.15};
  const double threshold = std::numeric_limits<double>::infinity();
  std::vector<SignedDistanceToPoint<double>> results =
      engine.ComputeSignedDistanceToPoint(p_WQ, X_WGs, threshold);

  ASSERT_EQ(results.size(), 1);
  const SignedDistanceToPoint<double>& d = results[0];
  EXPECT_EQ(d.id_G, id);
  // If the implementation changes, the selected nearest point may change.
  // There are three choices (0, 0.15, 0.15), (0.15, 0, 0.15), and
  // (0.15, 0.15, 0) with equal distances from the query point.
  EXPECT_TRUE(CompareMatrices(d.p_GN, Vector3d(0.15, 0, 0.15), 1e-14));
  EXPECT_NEAR(d.distance, -0.15, 1e-14);
  EXPECT_TRUE(CompareMatrices(d.grad_W, -Vector3d::UnitY(), 1e-14));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
