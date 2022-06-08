#include "drake/geometry/query_results/deformable_contact_data.h"

#include <gtest/gtest.h>

#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

const GeometryId kDeformableId = GeometryId::get_new_id();

// Test the constructor of DeformableContactData that it will compute the
// correct permutation of vertex indexes as documented in
// permuted_vertex_indexes() and permuted_to_original_indexes().
// For simplicity, we will use only one contact polygon with a mesh of only
// a few tetrahedra.
GTEST_TEST(DeformableContactDataTest, TestConstructorPermutedVertex) {
  PolygonSurfaceMesh<double> contact_mesh_W(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{
          Vector3<double>::UnitX(),
          Vector3<double>::UnitY(),
          Vector3<double>::UnitZ(),
      });

  // For simplicity, we set up one DeformableRigidContactSurface that contains
  // one contact polygon in tetrahedron 1 (index starts at 0) of `test_mesh_S`
  // that we will define later. The other values are not relevant for this test.
  DeformableRigidContactSurface<double> contact_surface0(
      std::move(contact_mesh_W), std::vector<double>{-0.1}, std::vector<int>{1},
      std::vector<Vector4<double>>{Vector4<double>{0.1, 0.2, 0.3, 0.4}},
      // Use arbitrary value for rigid Id.
      GeometryId::get_new_id(), kDeformableId);

  // We use a unit sphere as the shape specification for the reference
  // deformable geometry that we will create. Frame S is the intrinsic
  // frame of the sphere.
  const Sphere sphere_S(1.0);

  // For simplicity, we use the following peculiar mesh with only three
  // tetrahedra and six vertices. We can imagine that the three
  // tetrahedra are part of a mesh of the unit sphere. For this test, we
  // do not need the whole mesh of the sphere.  Positions of the
  // vertices are shown schematically in the picture below.
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

  const std::vector<int> kExpectedPermutedVertexIndexes{0, 4, 1, 2, 5, 3};

  std::vector<DeformableRigidContactSurface<double>> contact_surfaces;
  contact_surfaces.emplace_back(std::move(contact_surface0));
  DeformableContactData<double> dut(std::move(contact_surfaces), test_mesh_S);

  EXPECT_EQ(dut.deformable_id(), kDeformableId);
  EXPECT_EQ(dut.num_vertices_in_contact(), 4);
  EXPECT_EQ(dut.vertex_permutation().permutation(),
            kExpectedPermutedVertexIndexes);
}

// Test the constructor of DeformableContactData. For simplicity, we use a
// couple of contact polygons with a deformable geometry with only one
// tetrahedron.
GTEST_TEST(DeformableContactDataTest, TestConstructor) {
  // For simplicity, we use this single-tetrahedron mesh, which is a part of
  // the whole mesh of the unit sphere.
  // This tetrahedron 0 with vertices (0,0,0), (1,0,0), (0,1,0), (0,0,1)
  // is the same as {(x,y,z) : x >= 0, y >= 0, z >= 0, and x + y + z <= 1 }.
  // Within this tetrahedron, we define the approximated signed distance as
  //   approximated_sdf(x,y,z) = x + y + z - 1
  const VolumeMesh<double> mesh_S(
      {{0, 1, 2, 3}}, {Vector3<double>::Zero(), Vector3<double>::UnitX(),
                       Vector3<double>::UnitY(), Vector3<double>::UnitZ()});

  const int kTetrahedronIndex = 0;

  // Each of these two contact meshes has one polygon inside the only
  // tetrahedron of the volume mesh above. Any points with x > 0, y > 0,
  // z > 0, and x + y + z < 1 would be inside the tetrahedron
  PolygonSurfaceMesh<double> contact_mesh0_W(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{Vector3<double>(0.75, 0, 0),
                                   Vector3<double>(0, 0.75, 0),
                                   Vector3<double>(0, 0, 0.75)});
  // Evaluation of the sdf at the centroid.
  std::vector<double> penetration_distances0{-0.25};

  PolygonSurfaceMesh<double> contact_mesh1_W(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{Vector3<double>(0.15, 0, 0),
                                   Vector3<double>(0, 0.30, 0),
                                   Vector3<double>(0, 0, 0.75)});
  // Evaluation of the sdf at the centroid.
  std::vector<double> penetration_distances1{-0.6};

  DeformableRigidContactSurface<double> deformable_rigid_contact_surface0(
      std::move(contact_mesh0_W), std::move(penetration_distances0),
      std::vector<int>{kTetrahedronIndex},
      // Use arbitrary barycentric coordinate as it doesn't affect signed
      // distance calculation.
      std::vector<Vector4<double>>{Vector4<double>{0.1, 0.2, 0.3, 0.4}},
      // Use arbitrary values for rigid Id.
      GeometryId::get_new_id(), kDeformableId);

  DeformableRigidContactSurface<double> deformable_rigid_contact_surface1(
      std::move(contact_mesh1_W), std::move(penetration_distances1),
      std::vector<int>{kTetrahedronIndex},
      // Use arbitrary barycentric coordinate as it doesn't affect signed
      // distance calculation.
      std::vector<Vector4<double>>{Vector4<double>{0.1, 0.2, 0.3, 0.4}},
      // Use arbitrary values for Id.
      GeometryId::get_new_id(), kDeformableId);

  std::vector<DeformableRigidContactSurface<double>> contact_surfaces;
  contact_surfaces.emplace_back(std::move(deformable_rigid_contact_surface0));
  contact_surfaces.emplace_back(std::move(deformable_rigid_contact_surface1));
  DeformableContactData<double> dut(std::move(contact_surfaces), mesh_S);

  ASSERT_EQ(dut.num_contact_surfaces(), 2);
  ASSERT_EQ(dut.signed_distances().size(), 2);
  EXPECT_EQ(dut.signed_distances()[0], -0.25);
  EXPECT_EQ(dut.signed_distances()[1], -0.6);
}

// Test the constructor of DeformableContactData in the special case of empty
// DeformableRigidContactSurface that the permutation of vertex indexes will be
// identity.
GTEST_TEST(DeformableContactDataTest, TestConstructorNoContactSurface) {
  std::vector<DeformableRigidContactSurface<double>> empty_contact_surfaces{};
  // The kind of Shape is not relevant. We just pick Box for simplicity.
  const Box box(1, 2, 3);
  const VolumeMesh<double> box_mesh = MakeBoxVolumeMeshWithMa<double>(box);

  DeformableContactData<double> dut(std::move(empty_contact_surfaces),
                                    box_mesh);

  EXPECT_EQ(dut.num_contact_points(), 0);

  std::vector<int> empty_permutation;
  EXPECT_EQ(dut.vertex_permutation().permutation(), empty_permutation);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
