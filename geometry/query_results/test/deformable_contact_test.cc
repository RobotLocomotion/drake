#include "drake/geometry/query_results/deformable_contact.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

const GeometryId kIdA = GeometryId::get_new_id();
const GeometryId kIdB = GeometryId::get_new_id();

GTEST_TEST(ContactParticipation, NoVertexIsParticipating) {
  constexpr int kNumVertices = 3;
  ContactParticipation dut(kNumVertices);
  multibody::contact_solvers::internal::PartialPermutation permutation =
      dut.CalcVertexPermutation();
  EXPECT_EQ(permutation.domain_size(), kNumVertices);
  // The expected permutation is the identity.
  std::vector<int> kExpectedPermutation = {0, 1, 2};
  EXPECT_EQ(permutation.permutation(), kExpectedPermutation);
  EXPECT_EQ(dut.num_vertices_in_contact(), 0);
  EXPECT_EQ(dut.num_vertices(), 3);
}

GTEST_TEST(ContactParticipation, SomeVerticesAreParticipating) {
  constexpr int kNumVertices = 6;
  ContactParticipation dut(kNumVertices);
  /* |   Original       |   Permuted       |   Participating   |
     |   vertex index   |   vertex index   |   in contact      |
     | :--------------: | :--------------: | :---------------: |
     |        0         |        0         |       yes         |
     |        1         |        4         |       no          |
     |        2         |        1         |       yes         |
     |        3         |        2         |       yes         |
     |        4         |        5         |       no          |
     |        5         |        3         |       yes         | */
  const std::unordered_set<int> participating_vertices = {{0, 3, 2, 5}};
  dut.Participate(participating_vertices);
  const std::vector<int> kExpectedVertexPermutation{0, 4, 1, 2, 5, 3};
  const std::vector<int> kExpectedVertexPartialPermutation{0, -1, 1, 2, -1, 3};
  const std::vector<int> kExpectedDofPermutation{
      0, 1, 2, 12, 13, 14, 3, 4, 5, 6, 7, 8, 15, 16, 17, 9, 10, 11};
  const std::vector<int> kExpectedDofPartialPermutation{
      0, 1, 2, -1, -1, -1, 3, 4, 5, 6, 7, 8, -1, -1, -1, 9, 10, 11};
  EXPECT_EQ(dut.CalcVertexPermutation().permutation(),
            kExpectedVertexPermutation);
  EXPECT_EQ(dut.CalcVertexPartialPermutation().permutation(),
            kExpectedVertexPartialPermutation);
  EXPECT_EQ(dut.CalcDofPermutation().permutation(), kExpectedDofPermutation);
  EXPECT_EQ(dut.CalcDofPartialPermutation().permutation(),
            kExpectedDofPartialPermutation);
}

/* Tests the constructor constructs an empty DeformableContactSurface. */
GTEST_TEST(DeformableContactSurface, EmptySurface) {
  DeformableContactSurface<double> dut(kIdA, kIdB, {}, {}, {}, {}, {}, {});
  EXPECT_EQ(dut.id_A(), kIdA);
  EXPECT_EQ(dut.id_B(), kIdB);
  EXPECT_EQ(dut.num_contact_points(), 0);
  EXPECT_EQ(dut.contact_points_W().size(), 0);
  EXPECT_EQ(dut.signed_distances().size(), 0);
  EXPECT_EQ(dut.barycentric_coordinates_A().size(), 0);
  EXPECT_EQ(dut.contact_vertex_indexes_A().size(), 0);
  EXPECT_EQ(dut.nhats_W().size(), 0);
  EXPECT_FALSE(dut.is_B_deformable());
}

GTEST_TEST(DeformableContactSurface, Getters) {
  PolygonSurfaceMesh<double> contact_mesh_W(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{
          Vector3<double>::UnitX(),
          Vector3<double>::UnitY(),
          Vector3<double>::UnitZ(),
      });
  const std::vector<Vector3<double>> nhats_W = {
      Vector3<double>::Ones().normalized()};
  const std::vector<Vector3<double>> contact_points_W = {
      {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}};
  const std::vector<double> signed_distances = {-0.25};
  const std::vector<Vector4<int>> vertex_indexes_A = {{1, 2, 3, 4}};
  const std::vector<Vector4<int>> vertex_indexes_B = {{5, 6, 7, 8}};
  const std::vector<Vector4<double>> barycentric_centroids_A = {
      {0.1, 0.2, 0.3, 0.4}};
  const std::vector<Vector4<double>> barycentric_centroids_B = {
      {0.4, 0.3, 0.2, 0.1}};
  DeformableContactSurface<double> dut(
      kIdA, kIdB, contact_mesh_W, signed_distances, vertex_indexes_A,
      barycentric_centroids_A, vertex_indexes_B, barycentric_centroids_B);
  EXPECT_EQ(dut.id_A(), kIdA);
  EXPECT_EQ(dut.id_B(), kIdB);
  EXPECT_EQ(dut.num_contact_points(), 1);
  EXPECT_EQ(dut.contact_points_W(), contact_points_W);
  EXPECT_EQ(dut.signed_distances(), signed_distances);
  EXPECT_EQ(dut.contact_vertex_indexes_A(), vertex_indexes_A);
  EXPECT_EQ(dut.contact_vertex_indexes_B(), vertex_indexes_B);
  EXPECT_EQ(dut.barycentric_coordinates_A(), barycentric_centroids_A);
  EXPECT_EQ(dut.barycentric_coordinates_B(), barycentric_centroids_B);
  EXPECT_EQ(dut.nhats_W(), nhats_W);
}

GTEST_TEST(DeformableContact, RegisterGeometry) {
  constexpr int kNumVertices = 6;
  DeformableContact<double> dut;
  EXPECT_FALSE(dut.IsRegistered(kIdA));
  dut.RegisterDeformableGeometry(kIdA, kNumVertices);
  EXPECT_TRUE(dut.IsRegistered(kIdA));
  EXPECT_EQ(dut.contact_participation(kIdA).num_vertices_in_contact(), 0);
  /* Querying unregistered geometry throws. */
  GeometryId fake_id = GeometryId::get_new_id();
  EXPECT_THROW(dut.contact_participation(fake_id), std::exception);

  EXPECT_THROW(dut.GetNumVerticesOrThrow(fake_id), std::exception);
  EXPECT_EQ(dut.GetNumVerticesOrThrow(kIdA), 6);
}

GTEST_TEST(DeformableContact, AddDeformableRigidContactSurface) {
  DeformableContact<double> dut;
  constexpr int kNumVertices = 6;
  dut.RegisterDeformableGeometry(kIdA, kNumVertices);

  PolygonSurfaceMesh<double> contact_mesh_W(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{
          Vector3<double>::UnitX(),
          Vector3<double>::UnitY(),
          Vector3<double>::UnitZ(),
      });
  const std::vector<Vector3<double>> contact_points_W = {
      {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}};
  const std::vector<double> signed_distances = {-0.25};
  const std::vector<Vector4<int>> contact_vertex_indexes = {{0, 3, 2, 5}};
  const std::vector<Vector4<double>> barycentric_centroids = {
      {0.1, 0.2, 0.3, 0.4}};

  dut.AddDeformableRigidContactSurface(kIdA, kIdB, {0, 3, 2, 5}, contact_mesh_W,
                                       signed_distances, contact_vertex_indexes,
                                       barycentric_centroids);
  /* Verify that the contact surface is as expected. */
  const std::vector<DeformableContactSurface<double>>& surfaces =
      dut.contact_surfaces();
  ASSERT_EQ(surfaces.size(), 1);
  const DeformableContactSurface<double>& s = surfaces[0];
  EXPECT_EQ(s.id_A(), kIdA);
  EXPECT_EQ(s.id_B(), kIdB);
  EXPECT_EQ(s.num_contact_points(), 1);
  EXPECT_EQ(s.contact_points_W(), contact_points_W);
  EXPECT_EQ(s.signed_distances(), signed_distances);
  EXPECT_EQ(s.contact_vertex_indexes_A(), contact_vertex_indexes);
  EXPECT_EQ(s.barycentric_coordinates_A(), barycentric_centroids);
  EXPECT_FALSE(s.is_B_deformable());
  /* Verify that contact participation is as expected. */
  EXPECT_EQ(dut.contact_participation(kIdA).num_vertices_in_contact(), 4);
}

GTEST_TEST(DeformableContact, Participate) {
  constexpr int kNumVertices = 6;
  DeformableContact<double> dut;
  dut.RegisterDeformableGeometry(kIdA, kNumVertices);
  EXPECT_EQ(dut.contact_participation(kIdA).num_vertices_in_contact(), 0);
  dut.Participate(kIdA, {0, 1, 5});
  EXPECT_EQ(dut.contact_participation(kIdA).num_vertices_in_contact(), 3);
}

GTEST_TEST(DeformableContact, AddDeformableDeformableContactSurface) {
  DeformableContact<double> dut;
  constexpr int kNumVertices = 6;
  dut.RegisterDeformableGeometry(kIdA, kNumVertices);
  dut.RegisterDeformableGeometry(kIdB, kNumVertices);

  PolygonSurfaceMesh<double> contact_mesh_W(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{
          Vector3<double>::UnitX(),
          Vector3<double>::UnitY(),
          Vector3<double>::UnitZ(),
      });
  const std::vector<Vector3<double>> contact_points_W = {
      {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}};
  const std::vector<double> signed_distances = {-0.25};
  const std::vector<Vector4<int>> contact_vertex_indexes_A = {{0, 3, 2, 5}};
  const std::vector<Vector4<int>> contact_vertex_indexes_B = {{1, 4, 3, 0}};
  const std::vector<Vector4<double>> barycentric_centroids_A = {
      {0.1, 0.2, 0.3, 0.4}};
  const std::vector<Vector4<double>> barycentric_centroids_B = {
      {0.11, 0.19, 0.29, 0.41}};

  dut.AddDeformableDeformableContactSurface(
      kIdA, kIdB, {0, 3, 2, 5}, {1, 4, 3, 0}, contact_mesh_W, signed_distances,
      contact_vertex_indexes_A, contact_vertex_indexes_B,
      barycentric_centroids_A, barycentric_centroids_B);
  // Verify that the contact surface is as expected.
  const std::vector<DeformableContactSurface<double>>& surfaces =
      dut.contact_surfaces();
  ASSERT_EQ(surfaces.size(), 1);
  const DeformableContactSurface<double>& s = surfaces[0];

  // TODO(DamrongGuoy) When C++20 is available, define `default operator==`
  //  for DeformableContactSurface (and PolygonSurfaceMesh), so we can
  //  verify it with the following one statement instead of 12 statements.
  //  EXPECT_EQ(dut.contact_surfaces()[0],
  //            DeformableContactSurface<double>(
  //                kIdA, kIdB, contact_mesh_W, signed_distances,
  //                contact_vertex_indexes_A, barycentric_centroids_A,
  //                contact_vertex_indexes_B, barycentric_centroids_B));
  EXPECT_EQ(s.id_A(), kIdA);
  EXPECT_EQ(s.id_B(), kIdB);
  EXPECT_EQ(s.contact_mesh_W().num_faces(), 1);
  EXPECT_EQ(s.num_contact_points(), 1);
  EXPECT_EQ(s.signed_distances(), signed_distances);
  EXPECT_EQ(s.contact_points_W(), contact_points_W);
  EXPECT_EQ(s.barycentric_coordinates_A(), barycentric_centroids_A);
  EXPECT_EQ(s.contact_vertex_indexes_A(), contact_vertex_indexes_A);
  EXPECT_EQ(s.barycentric_coordinates_B(), barycentric_centroids_B);
  EXPECT_EQ(s.contact_vertex_indexes_B(), contact_vertex_indexes_B);
  EXPECT_EQ(s.nhats_W().size(), 1);
  EXPECT_TRUE(s.is_B_deformable());

  // Verify that contact participation is as expected.
  EXPECT_EQ(dut.contact_participation(kIdA).num_vertices_in_contact(), 4);
  EXPECT_EQ(
      dut.contact_participation(kIdA).CalcVertexPermutation().permutation(),
      // Permutation table for contact_vertex_indexes_A = {{0, 3, 2, 5}}
      //   |   Original       |   Permuted       |   Participating   |
      //   |   vertex index   |   vertex index   |   in contact      |
      //   | :--------------: | :--------------: | :---------------: |
      //   |        0         |        0         |       yes         |
      //   |        1         |        4         |       no          |
      //   |        2         |        1         |       yes         |
      //   |        3         |        2         |       yes         |
      //   |        4         |        5         |       no          |
      //   |        5         |        3         |       yes         |
      std::vector<int>({0, 4, 1, 2, 5, 3}));
  EXPECT_EQ(dut.contact_participation(kIdB).num_vertices_in_contact(), 4);
  EXPECT_EQ(
      dut.contact_participation(kIdB).CalcVertexPermutation().permutation(),
      // Permutation table for contact_vertex_indexes_B = {{1, 4, 3, 0}}
      //   |   Original       |   Permuted       |   Participating   |
      //   |   vertex index   |   vertex index   |   in contact      |
      //   | :--------------: | :--------------: | :---------------: |
      //   |        0         |        0         |       yes         |
      //   |        1         |        1         |       yes         |
      //   |        2         |        4         |       no          |
      //   |        3         |        2         |       yes         |
      //   |        4         |        3         |       yes         |
      //   |        5         |        5         |       no          |
      std::vector<int>({0, 1, 4, 2, 3, 5}));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
