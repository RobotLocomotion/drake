#include "drake/geometry/query_results/deformable_contact_data.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

const GeometryId kDeformableId = GeometryId::get_new_id();

// Tests the constructor constructs an empty DeformableContactData.
GTEST_TEST(DeformableContactDataTest, Constructor) {
  constexpr int kNumVertices = 3;
  DeformableContactData<double> dut(kDeformableId, kNumVertices);
  EXPECT_EQ(dut.deformable_id(), kDeformableId);
  EXPECT_EQ(dut.num_vertices_in_contact(), 0);
  EXPECT_EQ(dut.num_contact_surfaces(), 0);
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
 case of one tetrahedron in contact. */
GTEST_TEST(DeformableContactDataTest, CalcVertexPermutation) {
  PolygonSurfaceMesh<double> contact_mesh_W(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{
          Vector3<double>::UnitX(),
          Vector3<double>::UnitY(),
          Vector3<double>::UnitZ(),
      });

  const std::unordered_set<int> participating_vertices = {{0, 2, 3, 5}};
  // We set up one contact point with arbitrary data.
  std::vector<double> penetration_distances = {-0.25};
  std::vector<Vector4<double>> barycentric_centroids = {{0.1, 0.2, 0.3, 0.4}};

  // Vertices 0, 2, 3, 5 participate in contact.
  //
  //  original vertex index   participate         permuted vertex index
  //                                         (participate)  (non-participate)
  //           0                yes                0
  //           1                no                                 4
  //           2                yes                1
  //           3                yes                2
  //           4                no                                 5
  //           5                yes                3
  //
  //
  //  permuted vertex index     original vertex index     participate
  //           0                          0                   yes
  //           1                          2                   yes
  //           2                          3                   yes
  //           3                          5                   yes
  //  ----------------------------------------------- ---------------
  //           4                          1                   no
  //           5                          4                   no

  const std::vector<int> kExpectedPermutedVertexIndexes{0, 4, 1, 2, 5, 3};

  DeformableContactData<double> dut(kDeformableId, 6);
  dut.Append(GeometryId::get_new_id(), participating_vertices,
             std::move(contact_mesh_W), std::move(penetration_distances),
             std::move(barycentric_centroids));

  EXPECT_EQ(dut.deformable_id(), kDeformableId);
  EXPECT_EQ(dut.num_vertices_in_contact(), 4);
  EXPECT_EQ(dut.num_contact_surfaces(), 1);
  EXPECT_EQ(dut.num_contact_points(), 1);
  EXPECT_EQ(dut.CalcVertexPermutation().permutation(),
            kExpectedPermutedVertexIndexes);
}

/* Tests the Append gives the expected result when invoked multiple times. */
GTEST_TEST(DeformableContactDataTest, Append) {
  constexpr int kNumVertices = 4;
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
  std::vector<double> penetration_distances0{-0.25};
  std::vector<Vector4<double>> barycentric_centroids0 = {
      {Vector4<double>(0.25, 0.25, 0.25, 0.25)}};

  PolygonSurfaceMesh<double> contact_mesh1_W(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{Vector3<double>(0.15, 0, 0),
                                   Vector3<double>(0, 0.30, 0),
                                   Vector3<double>(0, 0, 0.75)});
  const Vector3<double> C1z_W = contact_mesh1_W.face_normal(0);
  // Evaluation of the sdf at the centroid.
  std::vector<double> penetration_distances1{-0.6};
  std::vector<Vector4<double>> barycentric_centroids1 = {
      {Vector4<double>(0.6, 0.05, 0.1, 0.25)}};

  const std::unordered_set<int> participating_vertices = {{0, 1, 2, 3}};
  DeformableContactData<double> dut(kDeformableId, kNumVertices);
  dut.Append(kRigidId0, participating_vertices, std::move(contact_mesh0_W),
             std::move(penetration_distances0),
             std::move(barycentric_centroids0));
  dut.Append(kRigidId1, participating_vertices, std::move(contact_mesh1_W),
             std::move(penetration_distances1),
             std::move(barycentric_centroids1));

  ASSERT_EQ(dut.num_contact_surfaces(), 2);
  ASSERT_EQ(dut.num_contact_points(), 2);
  ASSERT_EQ(dut.num_vertices_in_contact(), 4);

  ASSERT_EQ(dut.signed_distances().size(), 2);
  EXPECT_EQ(dut.signed_distances()[0], -0.25);
  EXPECT_EQ(dut.signed_distances()[1], -0.6);

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
