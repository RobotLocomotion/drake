#include "drake/geometry/query_results/deformable_rigid_contact.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

const GeometryId kDeformableId = GeometryId::get_new_id();

/* Tests the constructor constructs an empty DeformableRigidContact. */
GTEST_TEST(DeformableRigidContactTest, Constructor) {
  constexpr int kNumVertices = 3;
  DeformableRigidContact<double> dut(kDeformableId, kNumVertices);
  EXPECT_EQ(dut.deformable_id(), kDeformableId);
  EXPECT_EQ(dut.num_vertices_in_contact(), 0);
  EXPECT_EQ(dut.num_rigid_geometries(), 0);
  EXPECT_EQ(dut.num_contact_points(), 0);
  EXPECT_EQ(dut.rigid_ids().size(), 0);
  EXPECT_EQ(dut.barycentric_coordinates().size(), 0);
  EXPECT_EQ(dut.signed_distances().size(), 0);
  EXPECT_EQ(dut.R_CWs().size(), 0);

  multibody::contact_solvers::internal::PartialPermutation permutation =
      dut.CalcVertexPermutation();
  EXPECT_EQ(permutation.domain_size(), kNumVertices);
  // The expected permutation is the identity.
  std::vector<int> kExpectedPermutation = {0, 1, 2};
  EXPECT_EQ(permutation.permutation(), kExpectedPermutation);
}

/* Verifies that CalcVertexPermutation gives the expected result with a simple
 deformable geometry. The deformable mesh has 6 vertices, and two tetrahedra (5
 vertices) are in contact. */
GTEST_TEST(DeformableRigidContactTest, CalcVertexPermutation) {
  DeformableRigidContact<double> dut(
      kDeformableId, 6 /* number of vertices in deformable mesh */);

  const std::unordered_set<int> participating_vertices0 = {{0, 3, 2, 5}};
  // We set up one contact point with an arbitrary rigid geometry.
  PolygonSurfaceMesh<double> contact_mesh0_W(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{
          Vector3<double>::UnitX(),
          Vector3<double>::UnitY(),
          Vector3<double>::UnitZ(),
      });
  std::vector<double> penetration_distances0 = {-0.25};
  std::vector<int> tet_indexes0 = {2};
  std::vector<Vector4<double>> barycentric_centroids0 = {{0.1, 0.2, 0.3, 0.4}};

  dut.Append(GeometryId::get_new_id(), participating_vertices0,
             std::move(contact_mesh0_W), std::move(penetration_distances0),
             std::move(tet_indexes0),
             std::move(barycentric_centroids0));

  const std::unordered_set<int> participating_vertices1 = {{0, 3, 2, 4}};
  // We set up one contact point with another arbitrary rigid geometry.
  PolygonSurfaceMesh<double> contact_mesh1_W(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{
          2.0 * Vector3<double>::UnitX(),
          2.0 * Vector3<double>::UnitY(),
          2.0 * Vector3<double>::UnitZ(),
      });
  std::vector<double> penetration_distances1 = {-0.6};
  std::vector<int> tet_indexes1 = {3};
  std::vector<Vector4<double>> barycentric_centroids1 = {{0.4, 0.3, 0.2, 0.1}};

  dut.Append(GeometryId::get_new_id(), participating_vertices1,
             std::move(contact_mesh1_W), std::move(penetration_distances1),
             std::move(tet_indexes1),
             std::move(barycentric_centroids1));

  // Vertices 0, 2, 3, 4, 5 participate in contact.
  //
  //  original vertex index   participate         permuted vertex index
  //                                         (participate)  (non-participate)
  //           0                yes                0
  //           1                no                                 5
  //           2                yes                1
  //           3                yes                2
  //           4                yes                3
  //           5                yes                4
  //
  //
  //  permuted vertex index     original vertex index     participate
  //           0                          0                   yes
  //           1                          2                   yes
  //           2                          3                   yes
  //           3                          4                   yes
  //           4                          5                   yes
  //  ----------------------------------------------- ---------------
  //           5                          1                   no

  const std::vector<int> kExpectedPermutedVertexIndexes{0, 5, 1, 2, 3, 4};

  EXPECT_EQ(dut.deformable_id(), kDeformableId);
  EXPECT_EQ(dut.num_vertices_in_contact(), 5);
  EXPECT_EQ(dut.num_rigid_geometries(), 2);
  EXPECT_EQ(dut.num_contact_points(), 2);
  EXPECT_EQ(dut.CalcVertexPermutation().permutation(),
            kExpectedPermutedVertexIndexes);
}

/* Tests that per-contact point data is as expected when Append is invoked
 multiple times. */
GTEST_TEST(DeformableRigidContactTest, PerContactPointData) {
  constexpr int kNumVertices = 4;
  constexpr int kTetIndex = 3;
  const GeometryId kRigidId0 = GeometryId::get_new_id();
  const GeometryId kRigidId1 = GeometryId::get_new_id();
  // Consider the deformable geometry as a single-tetrahedron mesh
  // The vertices of the single tet are at (0,0,0), (1,0,0), (0,1,0), (0,0,1)
  // Within this tetrahedron, we define the approximated signed distance as
  //   approximated_sdf(x,y,z) = x + y + z - 1

  // Each of these two contact meshes has one polygon inside the only
  // tetrahedron of the volume mesh above. Any points with x > 0, y > 0,
  // z > 0, and x + y + z < 1 would be inside the tetrahedron
  PolygonSurfaceMesh<double> contact_mesh0_W(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{Vector3<double>(0.75, 0, 0),
                                   Vector3<double>(0, 0.75, 0),
                                   Vector3<double>(0, 0, 0.75)});
  const Vector3<double> C0z_W = contact_mesh0_W.face_normal(0);
  std::vector<double> penetration_distances0 = {-0.25};
  std::vector<int> tet_indexes0 = {kTetIndex};
  std::vector<Vector4<double>> barycentric_centroids0 = {
      {Vector4<double>(0.25, 0.25, 0.25, 0.25)}};

  PolygonSurfaceMesh<double> contact_mesh1_W(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{Vector3<double>(0.15, 0, 0),
                                   Vector3<double>(0, 0.30, 0),
                                   Vector3<double>(0, 0, 0.75)});
  const Vector3<double> C1z_W = contact_mesh1_W.face_normal(0);
  // Evaluation of the sdf at the centroid.
  std::vector<double> penetration_distances1 = {-0.6};
  std::vector<int> tet_indexes1 = {kTetIndex};
  std::vector<Vector4<double>> barycentric_centroids1 = {
      {Vector4<double>(0.6, 0.05, 0.1, 0.25)}};

  const std::unordered_set<int> participating_vertices = {{0, 1, 2, 3}};
  DeformableRigidContact<double> dut(kDeformableId, kNumVertices);
  dut.Append(kRigidId0, participating_vertices, std::move(contact_mesh0_W),
             std::move(penetration_distances0),
             std::move(tet_indexes0),
             std::move(barycentric_centroids0));
  dut.Append(kRigidId1, participating_vertices, std::move(contact_mesh1_W),
             std::move(penetration_distances1),
             std::move(tet_indexes1),
             std::move(barycentric_centroids1));

  ASSERT_EQ(dut.num_rigid_geometries(), 2);
  ASSERT_EQ(dut.num_contact_points(), 2);
  ASSERT_EQ(dut.num_vertices_in_contact(), 4);

  ASSERT_EQ(dut.signed_distances().size(), 2);
  EXPECT_EQ(dut.signed_distances()[0], -0.25);
  EXPECT_EQ(dut.signed_distances()[1], -0.6);

  ASSERT_EQ(dut.tetrahedra_indexes().size(), 2);
  EXPECT_EQ(dut.tetrahedra_indexes()[0], kTetIndex);
  EXPECT_EQ(dut.tetrahedra_indexes()[1], kTetIndex);

  ASSERT_EQ(dut.barycentric_coordinates().size(), 2);
  EXPECT_EQ(dut.barycentric_coordinates()[0],
            Vector4<double>(0.25, 0.25, 0.25, 0.25));
  EXPECT_EQ(dut.barycentric_coordinates()[1],
            Vector4<double>(0.6, 0.05, 0.10, 0.25));

  ASSERT_EQ(dut.barycentric_coordinates().size(), 2);
  EXPECT_EQ(dut.barycentric_coordinates()[0],
            Vector4<double>(0.25, 0.25, 0.25, 0.25));
  EXPECT_EQ(dut.barycentric_coordinates()[1],
            Vector4<double>(0.6, 0.05, 0.10, 0.25));

  const Vector3<double> Cz_C = Vector3<double>::UnitZ();
  const double kEps = 4 * std::numeric_limits<double>::epsilon();
  ASSERT_EQ(dut.R_CWs().size(), 2);
  EXPECT_TRUE(CompareMatrices(dut.R_CWs()[0] * C0z_W, Cz_C, kEps));
  EXPECT_TRUE(CompareMatrices(dut.R_CWs()[1] * C1z_W, Cz_C, kEps));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
