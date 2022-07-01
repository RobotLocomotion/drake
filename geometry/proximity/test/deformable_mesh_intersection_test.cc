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
      unit_sphere, MakeSphereVolumeMesh<double>(
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

  std::unique_ptr<DeformableRigidContactSurface<double>>
      deformable_rigid_contact_surface =
          ComputeContactSurfaceFromDeformableVolumeRigidSurface(
              deformable_id, deformable_W, rigid_id, rigid_mesh_R, rigid_bvh_R,
              X_WR);

  EXPECT_EQ(deformable_rigid_contact_surface, nullptr);
}

// Note that the deformable rigid contact surface computation largely utilizes
// previously tested code in mesh_intersection.cc. The only thing it adds is the
// detection and addition of tet indices and centroid coordinates. We only test
// the newly added operations here in this test.
GTEST_TEST(ComputeContactSurfaceDeformableRigid, OnePolygon) {
  const GeometryId deformable_id = GeometryId::get_new_id();
  const Sphere unit_sphere(1.0);
  // We use only one tetrahedron for simplicity. The fact that it doesn't cover
  // the sphere is irrelevant to this test.
  const VolumeMesh<double> single_tetrahedron_mesh_W(
      {{0, 1, 2, 3}}, {Vector3<double>::Zero(), Vector3<double>::UnitX(),
                       Vector3<double>::UnitY(), Vector3<double>::UnitZ()});
  const deformable::DeformableGeometry deformable_W(
      unit_sphere, single_tetrahedron_mesh_W);

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

  std::unique_ptr<DeformableRigidContactSurface<double>> contact_surface_W =
      ComputeContactSurfaceFromDeformableVolumeRigidSurface(
          deformable_id, deformable_W, rigid_id, rigid_mesh_R, rigid_bvh_R,
          X_WR);
  ASSERT_NE(contact_surface_W, nullptr);
  constexpr int kExpectedNumContactPoints = 1;
  EXPECT_EQ(contact_surface_W->num_contact_points(), kExpectedNumContactPoints);
  const std::vector<math::RotationMatrixd>& R_CWs = contact_surface_W->R_CWs();
  ASSERT_EQ(R_CWs.size(), kExpectedNumContactPoints);
  constexpr double kEps = 1e-14;
  // Since the contact normal is the unit z vector in the world frame, we expect
  // the rotation matrix mapping world frame quantities into contact frame
  // quantities to preserve the unit z vector.
  EXPECT_TRUE(CompareMatrices(R_CWs[0] * Vector3<double>::UnitZ(),
                              Vector3<double>::UnitZ(), kEps));

  const double signed_distance_at_contact_point =
      contact_surface_W->penetration_distance(0);
  // The approximated signed distance function on the tetrahedron is
  // s(x,y,z) = x + y + z - 1.  Therefore, the centroid (1/6, 1/6, 1/2) has
  // the signed distance = 1/6 + 1/6 + 1/2 - 1 = -1/6
  EXPECT_NEAR(signed_distance_at_contact_point, -1.0 / 6, kEps);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
