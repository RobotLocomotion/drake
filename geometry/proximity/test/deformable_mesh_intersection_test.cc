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

  // Initialize these two variables as non-empty sequences of nonsense
  // values expected to be become empty later.
  std::vector<int> tetrahedron_index_of_polygons{2022, 6, 9};
  std::vector<VolumeMesh<double>::Barycentric<double>> barycentric_centroids{
      {0.1, 1.2, 2.3, 3.4}};
  std::unique_ptr<ContactSurface<double>> contact_surface_W =
      ComputeContactSurfaceFromDeformableVolumeRigidSurface(
          deformable_id, deformable_W,
          rigid_id, rigid_mesh_R, rigid_bvh_R, X_WR,
          &tetrahedron_index_of_polygons,
          &barycentric_centroids);

  EXPECT_EQ(contact_surface_W, nullptr);
  // Confirm that they become empty.
  EXPECT_EQ(tetrahedron_index_of_polygons.size(), 0);
  EXPECT_EQ(barycentric_centroids.size(), 0);
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

  std::vector<int> tetrahedron_index_of_polygons;
  std::vector<VolumeMesh<double>::Barycentric<double>> barycentric_centroids;
  std::unique_ptr<ContactSurface<double>> contact_surface_W =
      ComputeContactSurfaceFromDeformableVolumeRigidSurface(
          deformable_id, deformable_W,
          rigid_id, rigid_mesh_R, rigid_bvh_R, X_WR,
          &tetrahedron_index_of_polygons,
          &barycentric_centroids);

  const int kExpectedNumPolygons = 1;
  ASSERT_EQ(tetrahedron_index_of_polygons.size(), kExpectedNumPolygons);
  ASSERT_EQ(barycentric_centroids.size(), kExpectedNumPolygons);

  const std::vector<int> kExpectedTetrahedronIndexOfPolygons{0};
  EXPECT_EQ(tetrahedron_index_of_polygons, kExpectedTetrahedronIndexOfPolygons);

  // Since the triangle is quite large, its intersection with the
  // tetrahedron is an isosceles right triangle with two edges of length 0.5.
  // Its three vertices are at (0, 0, 0.5), (0.5, 0, 0.5), (0, 0.5, 0.5), not
  // necessarily in this order. Its centroid is at (1/6, 1/6, 1/2).
  const Vector3<double> kExpectedCentroid_W(1.0 / 6, 1.0 / 6, 1.0 / 2);
  const VolumeMesh<double>::Barycentric<double> expected_barycentric =
      single_tetrahedron_mesh_W.CalcBarycentric(kExpectedCentroid_W, 0);

  const double kEps = 1e-14;
  EXPECT_TRUE(
      CompareMatrices(barycentric_centroids[0], expected_barycentric, kEps));

  ASSERT_NE(contact_surface_W, nullptr);
  EXPECT_EQ(contact_surface_W->num_faces(), kExpectedNumPolygons);
  // Area of the isosceles right triangle with two edges of length 0.5 is
  // 0.5 * 0.5 / 2 = 0.125.
  EXPECT_NEAR(contact_surface_W->area(0), 0.125, kEps);
  EXPECT_EQ(contact_surface_W->face_normal(0), Vector3<double>::UnitZ());
  EXPECT_TRUE(CompareMatrices(contact_surface_W->centroid(0),
                              kExpectedCentroid_W, kEps));

  const double signed_distance_at_centroid_of_polygon0 =
      contact_surface_W->poly_e_MN().EvaluateCartesian(
          0, contact_surface_W->centroid(0));
  // The approximated signed distance function on the tetrahedron is
  // s(x,y,z) = x + y + z - 1.  Therefore, the centroid (1/6, 1/6, 1/2) has
  // the signed distance = 1/6 + 1/6 + 1/2 - 1 = -1/6
  EXPECT_NEAR(signed_distance_at_centroid_of_polygon0,
              -1.0 / 6, kEps);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
