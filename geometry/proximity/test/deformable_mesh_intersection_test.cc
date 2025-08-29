#include "drake/geometry/proximity/deformable_mesh_intersection.h"

#include <limits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/make_box_field.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

const deformable::DeformableGeometry MakeDeformableGeometry(
    const VolumeMesh<double>& mesh) {
  std::vector<int> surface_vertices;
  std::vector<int> surface_tri_to_volume_tet;
  TriangleSurfaceMesh<double> surface_mesh =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(
          mesh, &surface_vertices, &surface_tri_to_volume_tet);
  return deformable::DeformableGeometry(mesh, std::move(surface_mesh),
                                        std::move(surface_vertices),
                                        std::move(surface_tri_to_volume_tet));
}

GTEST_TEST(ComputeContactSurfaceDeformableRigid, NoContact) {
  const GeometryId deformable_id = GeometryId::get_new_id();
  const Sphere unit_sphere(1.0);
  const deformable::DeformableGeometry deformable_W =
      MakeDeformableGeometry(MakeSphereVolumeMesh<double>(
          unit_sphere, 10.0 /* very coarse resolution */,
          TessellationStrategy::kDenseInteriorVertices));

  const GeometryId rigid_id = GeometryId::get_new_id();
  // The cube of edge length 2.0 occupies the space [-1,1]x[-1,1]x[-1,1].
  const VolumeMesh<double> rigid_mesh_R =
      MakeBoxVolumeMeshWithMa<double>(Box::MakeCube(2.0));
  const Bvh<Obb, VolumeMesh<double>> rigid_bvh_R(rigid_mesh_R);
  const VolumeMeshFieldLinear<double, double> pressure_field_R =
      MakeBoxPressureField<double>(Box::MakeCube(2.0), &rigid_mesh_R, 1e5);

  /* We use the knowledge that the coarsest sphere volume mesh is a octahedron
   to pose the rigid surface so that
     1. it does not intersect the deformable geometry, and
     2. it does intersect the bounding volume (AABB) of at least one
     tetrahedron.
   Projected to the xy-plane, the setup of the two geometries looks like
                                 ______
                                |      |
                          /|\   |      |  box surface (OBB)
                        /  |  \ |______|
      sphere volume   /____|____\
      as octeherdron  \    |    /
      (AABB)            \  |  /
                          \|/
   This setup helps us verify that when `CalcContactPolygon()` is called, but
   there is no new intersection polygon, the code does the right thing. */
  const double kTol = 1e-10;
  math::RigidTransform<double> X_WR(
      Vector3<double>{4.0 / 3 + kTol, 4.0 / 3 + kTol, 4.0 / 3 + kTol});

  // Verify that the two BVHs have indeed collided.
  bool bvhs_collide = false;
  deformable_W.deformable_surface().bvh().Collide(
      rigid_bvh_R, X_WR, [&bvhs_collide](int, int) {
        bvhs_collide = true;
        return BvttCallbackResult::Terminate;
      });
  ASSERT_TRUE(bvhs_collide);

  DeformableContact<double> contact_data;
  contact_data.RegisterDeformableGeometry(
      deformable_id, deformable_W.deformable_volume().mesh().num_vertices());
  AddDeformableRigidContactSurface(
      deformable_W.deformable_surface(), deformable_W.deformable_volume(),
      deformable_W.surface_index_to_volume_index(),
      deformable_W.surface_tri_to_volume_tet(), deformable_id, rigid_id,
      pressure_field_R, rigid_bvh_R, X_WR.inverse(), &contact_data);

  // Zero contact points and no vertices in contact are good enough indication
  // that the contact data is empty.
  EXPECT_TRUE(contact_data.contact_surfaces().empty());
}

// Note that the deformable rigid contact computation largely utilizes
// previously tested code in mesh_intersection.cc. On top of that, it evaluates
// pressure, reports deformable vertices participating in contact, and
// contact point barycentric coordinates. We test the correctness of these newly
// added operations here in this test.
GTEST_TEST(ComputeContactSurfaceDeformableRigid, OnePolygon) {
  const GeometryId deformable_id = GeometryId::get_new_id();
  const VolumeMesh<double> single_tetrahedron_mesh_W(
      {{0, 1, 2, 3}}, {Vector3<double>::Zero(), Vector3<double>::UnitX(),
                       Vector3<double>::UnitY(), Vector3<double>::UnitZ()});
  const deformable::DeformableGeometry deformable_W =
      MakeDeformableGeometry(single_tetrahedron_mesh_W);

  const GeometryId rigid_id = GeometryId::get_new_id();
  // Single-tet rigid geometry.
  const VolumeMesh<double> rigid_mesh_R(
      {{0, 1, 2, 3}}, {Vector3<double>::Zero(), Vector3<double>::UnitX(),
                       Vector3<double>::UnitY(), Vector3<double>::UnitZ()});
  const Bvh<Obb, VolumeMesh<double>> rigid_bvh_R(rigid_mesh_R);
  const double dummy_pressure = 123.0;
  std::vector<double> pressure_values{dummy_pressure, dummy_pressure,
                                      dummy_pressure, dummy_pressure};
  const VolumeMeshFieldLinear<double, double> pressure_field(
      std::move(pressure_values), &rigid_mesh_R);

  // Pose the rigid tet so that it intersects the bottom face
  // of the deformable tet. We choose a shift so that the intersection
  // triangle's centroid comes out to have a nice number.
  const math::RigidTransform<double> X_WR(
      Vector3<double>{1.0 / 6.0, 1.0 / 6.0, -0.5});

  DeformableContact<double> contact_data;
  contact_data.RegisterDeformableGeometry(deformable_id, 4);
  AddDeformableRigidContactSurface(
      deformable_W.deformable_surface(), deformable_W.deformable_volume(),
      deformable_W.surface_index_to_volume_index(),
      deformable_W.surface_tri_to_volume_tet(), deformable_id, rigid_id,
      pressure_field, rigid_bvh_R, X_WR.inverse(), &contact_data);
  constexpr int kExpectedNumContactPoints = 1;

  ASSERT_EQ(contact_data.contact_surfaces().size(), 1);
  const DeformableContactSurface<double>& contact_surface =
      contact_data.contact_surfaces()[0];

  ASSERT_EQ(contact_surface.num_contact_points(), kExpectedNumContactPoints);
  // The pressure value is equal to the dummy value because the interpolation is
  // constant.
  ASSERT_EQ(contact_surface.pressures().size(), kExpectedNumContactPoints);
  const double pressure = contact_surface.pressures()[0];
  EXPECT_NEAR(pressure, dummy_pressure, std::numeric_limits<double>::epsilon());
  // The centroid of the contact triangle in the world frame is (1/3, 1/3, 0).
  const Vector3<double> contact_point_W = contact_surface.contact_points_W()[0];
  constexpr double kTol = 1e-14;
  EXPECT_TRUE(CompareMatrices(
      contact_point_W, Vector3<double>(1.0 / 3.0, 1.0 / 3.0, 0.0), kTol));
  // The indexes of vertices incident to the bottom face of the triangle
  // containing the only contact point.
  const Vector3<int> vertex_indexes =
      contact_surface.tri_contact_vertex_indexes_A()[0];
  const Vector3<int> expected_vertex_indexes{2, 1, 0};
  EXPECT_EQ(vertex_indexes, expected_vertex_indexes);
  // The centroid is (1/3, 1/3, 0), and the vertex positions are (1, 0, 0),
  // (0, 1, 0), (0, 0, 0), (0, 0, 1). The the barycentric weights is (1/3, 1/3,
  // 1/3).
  ASSERT_EQ(contact_surface.tri_barycentric_coordinates_A().size(),
            kExpectedNumContactPoints);
  EXPECT_TRUE(
      CompareMatrices(contact_surface.tri_barycentric_coordinates_A()[0],
                      Vector3<double>(1.0 / 3, 1.0 / 3, 1.0 / 3), kTol));

  // Only one face is participating in contact and there are 3 vertices
  // incident to a triangle face.
  const ContactParticipation& contact_participation =
      contact_data.contact_participation(deformable_id);
  EXPECT_EQ(contact_participation.num_vertices_in_contact(), 3);
}

/* Tests that the per-contact-point data depends only on the relative pose
 between the deformable and the rigid geometry by verifying that they don't
 change when a rigid transform is applied to both geometries.
 In particular, this test verifies the correctness of signed distances on the
 contact surface isn't corrupted by updating vertex positions of the deformable
 geometry. */
GTEST_TEST(ComputeContactSurfaceDeformableRigid, OnlyRelativePoseMatters) {
  const GeometryId deformable_id = GeometryId::get_new_id();
  const Sphere unit_sphere(1.0);
  deformable::DeformableGeometry deformable_W =
      MakeDeformableGeometry(MakeSphereVolumeMesh<double>(
          unit_sphere, 10.0 /* very coarse resolution */,
          TessellationStrategy::kDenseInteriorVertices));

  const GeometryId rigid_id = GeometryId::get_new_id();
  // The cube of edge length 2.0 occupies the space [-1,1]x[-1,1]x[-1,1].
  const VolumeMesh<double> rigid_mesh_R =
      MakeBoxVolumeMeshWithMa<double>(Box::MakeCube(2.0));
  const Bvh<Obb, VolumeMesh<double>> rigid_bvh_R(rigid_mesh_R);
  const VolumeMeshFieldLinear<double, double> pressure_field_R =
      MakeBoxPressureField<double>(Box::MakeCube(2.0), &rigid_mesh_R, 1e5);
  math::RigidTransform<double> X_WR(Vector3<double>{1.5, 0.0, 0.0});

  /* Projected to the xy-plane, the setup of the two geometries looks like

                              ____________
                          /|\|            |
                        /  | |\           |
      sphere volume   /____|_|__\         |  box surface
      as octeherdron  \    | |  /         |
                        \  | |/           |
                          \|/|____________|                                 */

  /* Compute the first set of contact data. */
  const VolumeMesh<double> mesh_W = deformable_W.deformable_volume().mesh();
  DeformableContact<double> contact_data;
  contact_data.RegisterDeformableGeometry(deformable_id, mesh_W.num_vertices());
  AddDeformableRigidContactSurface(
      deformable_W.deformable_surface(), deformable_W.deformable_volume(),
      deformable_W.surface_index_to_volume_index(),
      deformable_W.surface_tri_to_volume_tet(), deformable_id, rigid_id,
      pressure_field_R, rigid_bvh_R, X_WR.inverse(), &contact_data);
  ASSERT_EQ(contact_data.contact_surfaces().size(), 1);
  const DeformableContactSurface<double>& contact_surface =
      contact_data.contact_surfaces()[0];
  EXPECT_GT(contact_surface.num_contact_points(), 0);

  /* We apply an arbitrary rigid transform to both the deformable and the rigid
   geometry but keep the relative pose between the deformable and the rigid
   geometry unchanged. */
  VectorX<double> q_WD(3 * mesh_W.num_vertices());
  const math::RigidTransform<double> arbitrary_transform(
      math::RollPitchYaw<double>(0, 0, 0), Vector3<double>(1.2, 3.4, 5.6));
  for (int v = 0; v < mesh_W.num_vertices(); ++v) {
    q_WD.segment<3>(3 * v) = arbitrary_transform * mesh_W.vertex(v);
  }
  const TriangleSurfaceMesh<double> surface_mesh_W =
      deformable_W.deformable_surface().mesh();
  VectorX<double> q_surface_WD(3 * surface_mesh_W.num_vertices());
  for (int v = 0; v < surface_mesh_W.num_vertices(); ++v) {
    q_surface_WD.segment<3>(3 * v) =
        arbitrary_transform * surface_mesh_W.vertex(v);
  }

  deformable_W.UpdateVertexPositions(q_WD, q_surface_WD);
  X_WR = arbitrary_transform * X_WR;
  /* Compute the second set of contact data. */
  DeformableContact<double> contact_data2;
  contact_data2.RegisterDeformableGeometry(deformable_id,
                                           mesh_W.num_vertices());
  AddDeformableRigidContactSurface(
      deformable_W.deformable_surface(), deformable_W.deformable_volume(),
      deformable_W.surface_index_to_volume_index(),
      deformable_W.surface_tri_to_volume_tet(), deformable_id, rigid_id,
      pressure_field_R, rigid_bvh_R, X_WR.inverse(), &contact_data2);
  ASSERT_EQ(contact_data2.contact_surfaces().size(), 1);
  const DeformableContactSurface<double>& contact_surface2 =
      contact_data2.contact_surfaces()[0];

  EXPECT_EQ(contact_surface.num_contact_points(),
            contact_surface2.num_contact_points());
  /* Verify that pressure values are not all equal simply because they are
   all zero -- some meaningful values do exist. */
  const auto [min_element, max_element] = std::minmax_element(
      contact_surface.pressures().begin(), contact_surface.pressures().end());
  EXPECT_GE(*min_element, 0.0);
  EXPECT_GT(*max_element, 0.0);
}

/* Tests that inverted elements are labeled correctly. */
GTEST_TEST(ComputeContactSurfaceDeformableRigid, InvertedElements) {
  const GeometryId deformable_id = GeometryId::get_new_id();
  const Sphere unit_sphere(1.0);
  deformable::DeformableGeometry deformable_W =
      MakeDeformableGeometry(MakeSphereVolumeMesh<double>(
          unit_sphere, 10.0 /* very coarse resolution */,
          TessellationStrategy::kDenseInteriorVertices));

  const GeometryId rigid_id = GeometryId::get_new_id();
  // The cube of edge length 2.0 occupies the space [-1,1]x[-1,1]x[-1,1].
  const VolumeMesh<double> rigid_mesh_R =
      MakeBoxVolumeMeshWithMa<double>(Box::MakeCube(2.0));
  const Bvh<Obb, VolumeMesh<double>> rigid_bvh_R(rigid_mesh_R);
  const VolumeMeshFieldLinear<double, double> pressure_field_R =
      MakeBoxPressureField<double>(Box::MakeCube(2.0), &rigid_mesh_R, 1e5);
  math::RigidTransform<double> X_WR(Vector3<double>{1.5, 0.0, 0.0});

  /* Projected to the xy-plane, the setup of the two geometries looks like

                              ____________
                          /|\|            |
                        /  | |\           |
      sphere volume   /____|_|__\         |  box surface
      as octeherdron  \    | |  /         |
                        \  | |/           |
                          \|/|____________|                                 */

  const VolumeMesh<double> mesh_W = deformable_W.deformable_volume().mesh();
  VectorX<double> q_WD(3 * mesh_W.num_vertices());
  for (int v = 0; v < mesh_W.num_vertices(); ++v) {
    q_WD.segment<3>(3 * v) = mesh_W.vertex(v);
  }
  const TriangleSurfaceMesh<double> surface_mesh_W =
      deformable_W.deformable_surface().mesh();
  VectorX<double> q_surface_WD(3 * surface_mesh_W.num_vertices());
  for (int v = 0; v < surface_mesh_W.num_vertices(); ++v) {
    q_surface_WD.segment<3>(3 * v) = surface_mesh_W.vertex(v);
  }

  auto check_inversion = [&](bool expect_inversion) {
    DeformableContact<double> contact_data;
    contact_data.RegisterDeformableGeometry(deformable_id,
                                            mesh_W.num_vertices());
    AddDeformableRigidContactSurface(
        deformable_W.deformable_surface(), deformable_W.deformable_volume(),
        deformable_W.surface_index_to_volume_index(),
        deformable_W.surface_tri_to_volume_tet(), deformable_id, rigid_id,
        pressure_field_R, rigid_bvh_R, X_WR.inverse(), &contact_data);
    ASSERT_EQ(contact_data.contact_surfaces().size(), 1);
    const DeformableContactSurface<double>& contact_surface =
        contact_data.contact_surfaces()[0];
    EXPECT_GT(contact_surface.num_contact_points(), 0);
    for (bool inverted : contact_surface.is_element_inverted()) {
      EXPECT_EQ(inverted, expect_inversion);
    }
  };

  /* Now we pull the only internal vertex of the deformable geometry out of its
   surface so that all participating tets are inverted. Here we use the
   knowledge that the coarsest sphere mesh (which is an octahedron) has only one
   internal vertex, and it's v0. */
  q_WD.segment<3>(0) = Vector3<double>(2.0, 0, 0);
  deformable_W.UpdateVertexPositions(q_WD, q_surface_WD);
  check_inversion(true);

  /* If we pull the vertex from "the other side", then the elements in contact
   are not inverted. */
  q_WD.segment<3>(0) = Vector3<double>(-2.0, 0, 0);
  deformable_W.UpdateVertexPositions(q_WD, q_surface_WD);
  check_inversion(false);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
