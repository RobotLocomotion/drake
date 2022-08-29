#include "drake/geometry/proximity/deformable_mesh_intersection.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(ComputeContactSurfaceDeformableRigid, NoContact) {
  const GeometryId deformable_id = GeometryId::get_new_id();
  const Sphere unit_sphere(1.0);
  const deformable::DeformableGeometry deformable_W(
      MakeSphereVolumeMesh<double>(
          unit_sphere, 10.0 /* very coarse resolution */,
          TessellationStrategy::kDenseInteriorVertices));

  const GeometryId rigid_id = GeometryId::get_new_id();
  // The cube of edge length 2.0 occupies the space [-1,1]x[-1,1]x[-1,1].
  const TriangleSurfaceMesh<double> rigid_mesh_R = MakeBoxSurfaceMesh<double>(
      Box::MakeCube(2.0), 10.0 /* very coarse resolution */);
  const Bvh<Obb, TriangleSurfaceMesh<double>> rigid_bvh_R(rigid_mesh_R);

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
  const double kEps = 1e-10;
  math::RigidTransform<double> X_WR(
      Vector3<double>{4.0 / 3 + kEps, 4.0 / 3 + kEps, 4.0 / 3 + kEps});

  // Verify that the two BVHs have indeed collided.
  bool bvhs_collide = false;
  deformable_W.deformable_mesh().bvh().Collide(
      rigid_bvh_R, X_WR, [&bvhs_collide](int, int) {
        bvhs_collide = true;
        return BvttCallbackResult::Terminate;
      });
  ASSERT_TRUE(bvhs_collide);

  DeformableRigidContact<double> contact_data(
      deformable_id, deformable_W.deformable_mesh().mesh().num_vertices());
  AppendDeformableRigidContact(deformable_W, rigid_id, rigid_mesh_R,
                               rigid_bvh_R, X_WR, &contact_data);

  // Zero contact points and no vertices in contact are good enough indication
  // that the contact data is empty.
  EXPECT_EQ(contact_data.num_contact_points(), 0);
  EXPECT_EQ(contact_data.num_vertices_in_contact(), 0);
}

// Note that the deformable rigid contact computation largely utilizes
// previously tested code in mesh_intersection.cc. On top of that, it evaluates
// signed distances, reports deformable vertices and tets participating in
// contact, and contact point barycentric coordinates. We test the correctness
// of these newly added operations here in this test.
GTEST_TEST(ComputeContactSurfaceDeformableRigid, OnePolygon) {
  const GeometryId deformable_id = GeometryId::get_new_id();
  const VolumeMesh<double> single_tetrahedron_mesh_W(
      {{0, 1, 2, 3}}, {Vector3<double>::Zero(), Vector3<double>::UnitX(),
                       Vector3<double>::UnitY(), Vector3<double>::UnitZ()});
  const deformable::DeformableGeometry deformable_W(
      std::move(single_tetrahedron_mesh_W));

  const GeometryId rigid_id = GeometryId::get_new_id();
  // Single-triangle surface mesh with the triangle large enough to intersect
  // the tetrahedron well.
  const TriangleSurfaceMesh<double> rigid_mesh_R(
      {{0, 1, 2}}, {Vector3<double>(-1, -1, 0), Vector3<double>(3, -1, 0),
                    Vector3<double>(-1, 3, 0)});
  const Bvh<Obb, TriangleSurfaceMesh<double>> rigid_bvh_R(rigid_mesh_R);

  // Pose the rigid surface at Wz=0.5 so that it intersects the deformable
  // octahedron somewhere on that surface (since the triangle is parallel to the
  // xy-plane).
  const math::RigidTransform<double> X_WR(Vector3<double>{0, 0, 0.5});

  DeformableRigidContact<double> contact_data(deformable_id, 4);
  AppendDeformableRigidContact(deformable_W, rigid_id, rigid_mesh_R,
                               rigid_bvh_R, X_WR, &contact_data);
  constexpr int kExpectedNumRigidGeometries = 1;
  constexpr int kExpectedNumContactPoints = 1;

  EXPECT_EQ(contact_data.contact_meshes_W().size(),
            kExpectedNumRigidGeometries);
  EXPECT_EQ(contact_data.contact_meshes_W()[0].num_faces(),
            kExpectedNumContactPoints);

  // The approximated signed distance function is zero because all vertices of
  // the tet mesh are on the boundary.
  ASSERT_EQ(contact_data.signed_distances().size(), kExpectedNumContactPoints);
  const double signed_distance_at_contact_point =
      contact_data.signed_distances()[0];
  EXPECT_NEAR(signed_distance_at_contact_point, 0.0,
              std::numeric_limits<double>::epsilon());

  ASSERT_EQ(contact_data.tetrahedra_indexes().size(),
            kExpectedNumContactPoints);
  constexpr int kTetIndex = 0;
  EXPECT_EQ(contact_data.tetrahedra_indexes()[0], kTetIndex);

  // Only on tetrahedron is participating in contact and there are 4 vertices
  // incident to a tetrahedron.
  EXPECT_EQ(contact_data.num_contact_points(), 1);
  EXPECT_EQ(contact_data.num_vertices_in_contact(), 4);

  // The centroid is (1/6, 1/6, 1/2), and the vertex positions are (0, 0, 0),
  // (1, 0, 0), (0, 1, 0), (0, 0, 1). The the barycentric weights is (1/6, 1/6,
  // 1/6, 1/2).
  ASSERT_EQ(contact_data.barycentric_coordinates().size(),
            kExpectedNumContactPoints);
  constexpr double kEps = 1e-14;
  EXPECT_TRUE(CompareMatrices(
      contact_data.barycentric_coordinates()[0],
      Vector4<double>(1.0 / 6, 1.0 / 6, 1.0 / 6, 1.0 / 2), kEps));

  // The correctnes of R_CWs are tested in the unit tests for
  // DeformableRigidContact. Here we only verify the correctness of the size.
  EXPECT_EQ(contact_data.R_CWs().size(), kExpectedNumContactPoints);
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
  deformable::DeformableGeometry deformable_W(MakeSphereVolumeMesh<double>(
      unit_sphere, 10.0 /* very coarse resolution */,
      TessellationStrategy::kDenseInteriorVertices));

  const GeometryId rigid_id = GeometryId::get_new_id();
  // The cube of edge length 2.0 occupies the space [-1,1]x[-1,1]x[-1,1].
  const TriangleSurfaceMesh<double> rigid_mesh_R = MakeBoxSurfaceMesh<double>(
      Box::MakeCube(2.0), 10.0 /* very coarse resolution */);
  const Bvh<Obb, TriangleSurfaceMesh<double>> rigid_bvh_R(rigid_mesh_R);
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
  const VolumeMesh<double> mesh_W = deformable_W.deformable_mesh().mesh();
  DeformableRigidContact<double> contact_data(deformable_id,
                                              mesh_W.num_vertices());
  AppendDeformableRigidContact(deformable_W, rigid_id, rigid_mesh_R,
                               rigid_bvh_R, X_WR, &contact_data);
  EXPECT_GT(contact_data.num_contact_points(), 0);

  /* We apply an arbitrary rigid transform to both the deformable and the rigid
   geometry but keep the relative pose between the deformable and the rigid
   geometry unchanged. */
  VectorX<double> q_WD(3 * mesh_W.num_vertices());
  const math::RigidTransform<double> arbitrary_transform(
      math::RollPitchYaw<double>(0, 0, 0), Vector3<double>(1.2, 3.4, 5.6));
  for (int v = 0; v < mesh_W.num_vertices(); ++v) {
    q_WD.segment<3>(3 * v) = arbitrary_transform * mesh_W.vertex(v);
  }
  deformable_W.UpdateVertexPositions(q_WD);
  X_WR = arbitrary_transform * X_WR;
  /* Compute the second set of contact data. */
  DeformableRigidContact<double> contact_data2(deformable_id,
                                               mesh_W.num_vertices());
  AppendDeformableRigidContact(deformable_W, rigid_id, rigid_mesh_R,
                               rigid_bvh_R, X_WR, &contact_data2);

  EXPECT_EQ(contact_data.num_contact_points(),
            contact_data2.num_contact_points());
  /* Verify that penetration distances are not all equal simply because they are
   all zero -- some meaningful values do exist. */
  const auto has_negative_distance =
      [](const DeformableRigidContact<double>& contact) {
        const std::vector<double>& sdf = contact.signed_distances();
        bool negative_distance_exists = false;
        for (const double d : sdf) {
          EXPECT_LE(d, 0.0);
          if (d < 0.0) {
            negative_distance_exists = true;
          }
        }
        EXPECT_TRUE(negative_distance_exists);
      };
  has_negative_distance(contact_data);
  const std::vector<double>& sdf = contact_data.signed_distances();
  const std::vector<double>& sdf2 = contact_data2.signed_distances();
  for (int i = 0; i < contact_data.num_contact_points(); ++i) {
    EXPECT_NEAR(sdf[i], sdf2[i], 1e-14);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
