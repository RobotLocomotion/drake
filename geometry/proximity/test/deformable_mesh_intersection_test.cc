#include "drake/geometry/proximity/deformable_mesh_intersection.h"

#include <utility>

#include <gtest/gtest.h>

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

  // Pose the rigid surface so that it does not intersect the deformable
  // geometry.
  math::RigidTransform<double> X_WR(Vector3<double>{0, 0, 10.0});

  // Initialize as an arbitrary non-empty sequence of indices, which is
  // expected to be become empty later.
  std::vector<int> tetrahedron_index_of_polygons {2022, 6, 9};
  std::unique_ptr<ContactSurface<double>> contact_surface_W =
      ComputeContactSurfaceFromDeformableVolumeRigidSurface(
          deformable_id, deformable_W,
          rigid_id, rigid_mesh_R, rigid_bvh_R, X_WR,
          &tetrahedron_index_of_polygons);

  EXPECT_EQ(contact_surface_W, nullptr);
  // Confirm that it becomes an empty sequence.
  EXPECT_EQ(tetrahedron_index_of_polygons.size(), 0);
}

GTEST_TEST(DeformableContactSurface, SmokeTest) {
  const GeometryId deformable_id = GeometryId::get_new_id();
  const Sphere unit_sphere(1.0);
  const deformable::DeformableGeometry deformable_W(
      unit_sphere, MakeSphereVolumeMesh<double>(
                       unit_sphere, 10.0 /* very coarse resolution */,
                       TessellationStrategy::kDenseInteriorVertices));
  // The coarse resolution should give 8 tetrahedron forming an octahedron.
  ASSERT_EQ(deformable_W.deformable_mesh().mesh().num_elements(), 8);

  const GeometryId rigid_id = GeometryId::get_new_id();
  // This cube of edge length 2.0 occupies the space [-1,1]x[-1,1]x[-1,1].
  const TriangleSurfaceMesh<double> rigid_mesh_R = MakeBoxSurfaceMesh<double>(
      Box::MakeCube(2.0), 10.0 /* very coarse resolution */);
  const Bvh<Obb, TriangleSurfaceMesh<double>> rigid_bvh_R(rigid_mesh_R);
  // The coarse resolution should give 12 triangles on the cube's surface.
  ASSERT_EQ(rigid_mesh_R.num_elements(), 12);

  // Pose the rigid surface so that it intersects the deformable octahedron at
  // Wz = 0.5
  math::RigidTransform<double> X_WR(Vector3<double>{0, 0, 1.5});

  std::vector<int> tetrahedron_index_of_polygons;
  std::unique_ptr<ContactSurface<double>> contact_surface_W =
          ComputeContactSurfaceFromDeformableVolumeRigidSurface(
              deformable_id, deformable_W,
              rigid_id, rigid_mesh_R, rigid_bvh_R, X_WR,
              &tetrahedron_index_of_polygons);

  const int kExpectedNumPolygons = 6;
  ASSERT_EQ(tetrahedron_index_of_polygons.size(), kExpectedNumPolygons);
  // The six contact polygons corresponds to this sequence of tetrahedra.
  // Specific values depend on connectivity of the input tetrahedral mesh
  // and the order of computation in the intersection algorithm.
  const std::vector<int> kExpectedTetrahedronIndexOfPolygons{0, 3, 0, 2, 1, 2};
  EXPECT_EQ(tetrahedron_index_of_polygons, kExpectedTetrahedronIndexOfPolygons);

  ASSERT_NE(contact_surface_W, nullptr);
  EXPECT_EQ(contact_surface_W->num_faces(), kExpectedNumPolygons);
  EXPECT_EQ(contact_surface_W->area(0), 0.0625);
  EXPECT_EQ(contact_surface_W->face_normal(0), -Vector3<double>::UnitZ());
  EXPECT_EQ(contact_surface_W->centroid(0),
            Vector3<double>(0.25, 1.0 / 12, 0.5));

  const double kExpectedSignedDistanceValue = -1.0 / 6;
  const double kEps = 1e-14;
  const double signed_distance_at_centroid_of_polygon0 =
      contact_surface_W->poly_e_MN().EvaluateCartesian(
          0, contact_surface_W->centroid(0));
  EXPECT_NEAR(signed_distance_at_centroid_of_polygon0,
              kExpectedSignedDistanceValue, kEps);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
