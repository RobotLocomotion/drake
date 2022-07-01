#include "drake/geometry/query_results/deformable_contact_data.h"

#include <gtest/gtest.h>

#include "drake/geometry/proximity/deformable_contact_geometries.h"
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
  auto contact_mesh_W = std::make_unique<PolygonSurfaceMesh<double>>(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{
          Vector3<double>::UnitX(),
          Vector3<double>::UnitY(),
          Vector3<double>::UnitZ(),
      });

  PolygonSurfaceMeshFieldLinear<double, double> sdf(
      std::vector<double>{-0.1, -0.1, -0.1}, contact_mesh_W.get(),
      // Constant field values means the gradient is zero.
      std::vector<Vector3<double>>{Vector3<double>::Zero()});

  // For simplicity, we set up one DeformableRigidContactSurface that contains
  // one contact polygon in tetrahedron 1 (index starts at 0) of `test_mesh_S`
  // that we will define later. The other values are not relevant for this test.
  // The unit normal is  not relevant to this test, but it must have unit length
  // in order to set up DeformableRigidContactSurface, which is used by
  // DeformableContactData.
  auto contact_surface0 =
      std::make_unique<DeformableRigidContactSurface<double>>(
          std::move(contact_mesh_W), sdf, std::vector<int>{1},
          std::vector<Vector4<double>>{Vector4<double>{0.1, 0.2, 0.3, 0.4}},
          // Use arbitrary values for simplicity.
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
  //                                         (participate)
  //                                         (non-participate)
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

  std::vector<std::unique_ptr<DeformableRigidContactSurface<double>>>
      contact_surfaces;
  contact_surfaces.emplace_back(std::move(contact_surface0));
  DeformableContactData<double> dut(std::move(contact_surfaces), test_mesh_S);

  EXPECT_EQ(dut.deformable_id(), kDeformableId);
  EXPECT_EQ(dut.num_vertices_in_contact(), 4);
  EXPECT_EQ(dut.vertex_permutation().permutation(),
            kExpectedPermutedVertexIndexes);
}

// Test the constructor of DeformableContactData that it creates the correct
// signed_distances() for each DeformableRigidContactSurface. For simplicity, we
// will use a couple of contact polygons with a mesh of only one tetrahedron.
GTEST_TEST(DeformableContactDataTest, TestConstructorSignedDistance) {
  // We use a unit sphere for the reference deformable geometry that we will
  // create. It is expressed in frame S.
  const Sphere unit_sphere_S(1.0);
  // For simplicity, we use this single-tetrahedron mesh, which is a part of
  // the whole mesh of the unit sphere.
  const VolumeMesh<double> mesh_S(
      {{0, 1, 2, 3}}, {Vector3<double>::Zero(), Vector3<double>::UnitX(),
                       Vector3<double>::UnitY(), Vector3<double>::UnitZ()});

  const int kTetrahedronIndex = 0;
  // This tetrahedron 0 with vertices (0,0,0), (1,0,0), (0,1,0), (0,0,1)
  // is the same as {(x,y,z) : x >= 0, y >= 0, z >= 0, and x + y + z <= 1 }.
  // Within this tetrahedron, the approximated signed distance is
  //   approximated_sdf(x,y,z) = x + y + z - 1
  // and its gradient is (1,1,1).
  const std::vector<int> kExpectedPermutedToOriginalIndexes{0, 2, 3, 5, 1, 4};
  Vector3<double> kGradientApproximatedSdf(1, 1, 1);

  // Each of these two contact meshes has one polygon inside the only
  // tetrahedron of the volume mesh above. Any points with x > 0, y > 0,
  // z > 0, and x + y + z < 1 would be inside the tetrahedron
  auto contact_mesh0_W = std::make_unique<PolygonSurfaceMesh<double>>(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{
          Vector3<double>(0.75, 0, 0),
          Vector3<double>(0, 0.75, 0),
          Vector3<double>(0, 0, 0.75)
      });
  auto contact_mesh1_W = std::make_unique<PolygonSurfaceMesh<double>>(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{
          Vector3<double>(0.15, 0, 0),
          Vector3<double>(0, 0.30, 0),
          Vector3<double>(0, 0, 0.75)
      });

  PolygonSurfaceMeshFieldLinear<double, double> sdf0(
      // approximated_sdf(x,y,z) = x + y + z - 1
      std::vector<double>{-0.25, -0.25, -0.25}, contact_mesh0_W.get(),
      std::vector<Vector3<double>>{kGradientApproximatedSdf});

  PolygonSurfaceMeshFieldLinear<double, double> sdf1(
      // approximated_sdf(x,y,z) = x + y + z - 1
      std::vector<double>{-0.85, -0.7, -0.25}, contact_mesh1_W.get(),
      std::vector<Vector3<double>>{kGradientApproximatedSdf});

  auto deformable_rigid_contact_surface0 =
      std::make_unique<DeformableRigidContactSurface<double>>(
          std::move(contact_mesh0_W), sdf0, std::vector<int>{kTetrahedronIndex},
          // Use arbitrary barycentric coordinate as it doesn't affect signed
          // distance calculation.
          std::vector<Vector4<double>>{Vector4<double>{0.1, 0.2, 0.3, 0.4}},
          // Use arbitrary values for simplicity.
          GeometryId::get_new_id(), kDeformableId);

  auto deformable_rigid_contact_surface1 =
      std::make_unique<DeformableRigidContactSurface<double>>(
          std::move(contact_mesh1_W), sdf1, std::vector<int>{kTetrahedronIndex},
          // Use arbitrary barycentric coordinate as it doesn't affect signed
          // distance calculation.
          std::vector<Vector4<double>>{Vector4<double>{0.1, 0.2, 0.3, 0.4}},
          // Use arbitrary values for simplicity.
          GeometryId::get_new_id(), kDeformableId);

  std::vector<std::unique_ptr<DeformableRigidContactSurface<double>>>
      contact_surfaces;
  contact_surfaces.emplace_back(std::move(deformable_rigid_contact_surface0));
  contact_surfaces.emplace_back(std::move(deformable_rigid_contact_surface1));
  DeformableContactData<double> dut(std::move(contact_surfaces), mesh_S);

  ASSERT_EQ(dut.num_contact_surfaces(), 2);
  ASSERT_EQ(dut.signed_distances().size(), 2);
  const double kEps = 4 * std::numeric_limits<double>::epsilon();
  // We leverage the knowledge about the order of the contact point from the
  // implementation here.
  EXPECT_NEAR(dut.signed_distances()[0], -0.25, kEps);
  EXPECT_NEAR(dut.signed_distances()[1], -0.6, kEps);
}

// Test the constructor of DeformableContactData in the special case of empty
// DeformableRigidContactSurface that the permutation of vertex indexes will be
// identity.
GTEST_TEST(DeformableContactDataTest, TestConstructorNoContactPair) {
  std::vector<std::unique_ptr<DeformableRigidContactSurface<double>>>
      empty_contact_surfaces{};
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
