#include "drake/geometry/query_results/deformable_contact_data.h"

#include <gtest/gtest.h>

#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Test the constructor of DeformableContactData that it will compute the
// correct permutation of vertex indexes as documented in
// permuted_vertex_indexes() and permuted_to_original_indexes().
// For simplicity, we will use only one contact polygon with a mesh of only
// a few tetrahedra.
GTEST_TEST(DeformableContactDataTest, TestConstructorPermutedVertex) {
  // For simplicity, we set up one contact pair that contains one contact
  // polygon in tetrahedron 1 (index starts at 0) of `test_mesh_S` that we
  // will define later. The other values are not relevant for this test. The
  // unit normal is  not relevant to this test, but it must have unit length
  // in order to set up DeformableRigidContactPair, which is used by
  // DeformableContactData.
  const std::vector<ContactPolygonData<double>> polygon_data{
    {.unit_normal = Vector3<double>::UnitZ(), .tet_index = 1}};

  // We use a unit sphere as the shape specification for the reference
  // deformable geometry that we will create. Frame S is the intrinsic frame
  // of the sphere.
  const Sphere sphere_S(1.0);

  // For simplicity, we use the following peculiar mesh with only three
  // tetrahedra and six vertices. We can imagine that the three tetrahedra
  // are part of a mesh of the unit sphere. For this test, we do not need the
  // whole mesh of the sphere.  Positions of the vertices are shown
  // schematically in the picture below.
  //
  //          +Z
  //          |    -X
  //          v3  /
  //          |  v5
  //          | /
  //          |/
  //        v0+------v2---- +Y
  //         /|
  //        / |
  //       v1 |
  //      /   v4
  //    +X    |
  //         -Z
  //
  const VolumeMesh<double> test_mesh_S(
      // We use this particular list of tetrahedra, so we can infer the
      // expected permutation of vertex indexes easily.
      {{0, 1, 2, 3},   // Tetrahedron 0 does not participate in contact.
       {0, 3, 2, 5},   // Tetrahedron 1 participates in contact.
       {0, 2, 1, 4}},  // Tetrahedron 2 does not participate in contact.
      {Vector3<double>::Zero(), Vector3<double>::UnitX(),
       Vector3<double>::UnitY(), Vector3<double>::UnitZ(),
       -Vector3<double>::UnitZ(), -Vector3<double>::UnitX()});

  // Vertices 0, 3, 2, 5 belong to tetrahedron 1, which participates in
  // contact (referred by `polygon_data`).
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
  //
  // The entries among participating vertices are sorted, and the entries
  // among non-participating vertices are also sorted separately.
  const std::vector<int> kExpectedPermutedVertexIndexes{0, 4, 1, 2, 5, 3};
  const std::vector<int> kExpectedPermutedToOriginalIndexes {0, 2, 3, 5, 1, 4};

  const int kDeformableId = 3;

  DeformableContactData<double> dut(
      {DeformableRigidContactPair<double>(
          DeformableContactSurface<double>(polygon_data),
          // Use arbitrary values for simplicity.
          GeometryId::get_new_id(), kDeformableId, 0.1, 0.2, 0.3)},
      ReferenceDeformableGeometry(sphere_S, test_mesh_S));

  EXPECT_EQ(dut.deformable_body_index(), kDeformableId);
  EXPECT_EQ(dut.num_vertices_in_contact(), 4);
  EXPECT_EQ(dut.permuted_vertex_indexes(), kExpectedPermutedVertexIndexes);
  EXPECT_EQ(dut.permuted_to_original_indexes(),
            kExpectedPermutedToOriginalIndexes);
}

// Test the constructor of DeformableContactData that it creates the correct
// signed_distances() for each contact pair. For simplicity, we will use a
// couple of contact polygons with a mesh of only one tetrahedron.
GTEST_TEST(DeformableContactDataTest, TestConstructorSignedDistance) {
  // We use a unit sphere for the reference deformable geometry that we will
  // create. It is expressed in frame S.
  const Sphere unit_sphere_S(1.0);
  // For simplicity, we use this single-tetrahedron mesh, which is a part of
  // the whole mesh of the unit sphere.
  const VolumeMesh<double> mesh_S(
      {{0, 1, 2, 3}}, {Vector3<double>::Zero(), Vector3<double>::UnitX(),
                       Vector3<double>::UnitY(), Vector3<double>::UnitZ()});
  const ReferenceDeformableGeometry reference(unit_sphere_S, mesh_S);

  // We imagine the centroids of two contact polygons correspond to these
  // locations in the reference tetrahedron expressed in frame S.
  // Any points with x > 0, y > 0, z > 0, and x + y + z < 1 would work.
  const Vector3<double> centroid0_S(0.25, 0.25, 0.25);
  const Vector3<double> centroid1_S(0.05, 0.10, 0.25);
  const int kTetrahedronIndex = 0;
  // Barycentric coordinates of the above two centroids in the reference
  // tetrahedron.
  const Vector4<double> barycentric0 =
      mesh_S.CalcBarycentric(centroid0_S, kTetrahedronIndex);
  const Vector4<double> barycentric1 =
      mesh_S.CalcBarycentric(centroid1_S, kTetrahedronIndex);

  // Only the barycentric coordinates of the polygon's centroid and the
  // tetrahedron index are relevant to this test. The unit normal is not
  // relevant to this test, but it must have unit length in order to set up
  // DeformableRigidContactPair later.
  const std::vector<ContactPolygonData<double>> polygon_data0{
      {.unit_normal = Vector3<double>::UnitZ(),
       .b_centroid = barycentric0,
       .tet_index = kTetrahedronIndex}};
  const std::vector<ContactPolygonData<double>> polygon_data1{
      {.unit_normal = Vector3<double>::UnitZ(),
       .b_centroid = barycentric1,
       .tet_index = kTetrahedronIndex}};

  const int kDeformableId = 3;
  DeformableContactData<double> dut(
      {DeformableRigidContactPair<double>(
           DeformableContactSurface<double>(polygon_data0),
           // kDeformableId must be the same for every contact pair.
           // Other values are arbitrary.
           GeometryId::get_new_id(), kDeformableId, 0.1, 0.2, 0.3),
       DeformableRigidContactPair<double>(
           DeformableContactSurface<double>(polygon_data1),
           // kDeformableId must be the same for every contact pair.
           // Other values are arbitrary.
           GeometryId::get_new_id(), kDeformableId, 0.4, 0.5, 0.6)},
      reference);

  ASSERT_EQ(dut.num_contact_pairs(), 2);
  ASSERT_EQ(dut.signed_distances(0).size(), 1);
  ASSERT_EQ(dut.signed_distances(1).size(), 1);
  const double expected_signed_distances0 =
      reference.signed_distance_field().Evaluate(kTetrahedronIndex,
                                                 barycentric0);
  const double expected_signed_distances1 =
      reference.signed_distance_field().Evaluate(kTetrahedronIndex,
                                                 barycentric1);
  const double kEps = 4 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(dut.signed_distances(0)[0], expected_signed_distances0, kEps);
  EXPECT_NEAR(dut.signed_distances(1)[0], expected_signed_distances1, kEps);
}

// Test the constructor of DeformableContactData in the special case of empty
// contact pairs that the permutation of vertex indexes will be identity.
GTEST_TEST(DeformableContactDataTest, TestConstructorNoContactPair) {
  std::vector<DeformableRigidContactPair<double>> empty_contact_pairs{};
  // The kind of Shape is not relevant. We just pick Box for simplicity.
  const Box box(1, 2, 3);
  const VolumeMesh<double> box_mesh = MakeBoxVolumeMeshWithMa<double>(box);

  DeformableContactData dut(
      empty_contact_pairs,
      ReferenceDeformableGeometry(box, box_mesh));

  EXPECT_EQ(dut.num_contact_points(), 0);

  std::vector<int> expect_identity_permutation(box_mesh.num_vertices());
  std::iota(expect_identity_permutation.begin(),
            expect_identity_permutation.end(), 0);
  EXPECT_EQ(dut.permuted_vertex_indexes(), expect_identity_permutation);
  EXPECT_EQ(dut.permuted_to_original_indexes(), expect_identity_permutation);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
