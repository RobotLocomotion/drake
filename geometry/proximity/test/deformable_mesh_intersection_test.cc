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
  const Sphere unit_sphere(1.0);
  const deformable::DeformableGeometry deformable_W(
      unit_sphere, MakeSphereVolumeMesh<double>(
                       unit_sphere, 10.0 /* very coarse resolution */,
                       TessellationStrategy::kDenseInteriorVertices));

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
  std::unique_ptr<DeformableContactSurface<double>>
      deformable_contact_surface_W =
          ComputeContactSurfaceFromDeformableVolumeRigidSurface(
              deformable_W, rigid_mesh_R, rigid_bvh_R, X_WR,
              &tetrahedron_index_of_polygons);

  EXPECT_EQ(deformable_contact_surface_W, nullptr);
  // Confirm that it becomes an empty sequence.
  EXPECT_EQ(tetrahedron_index_of_polygons.size(), 0);
}

GTEST_TEST(DeformableContactSurface, SmokeTest) {
  const Sphere unit_sphere(1.0);
  const deformable::DeformableGeometry deformable_W(
      unit_sphere, MakeSphereVolumeMesh<double>(
                       unit_sphere, 10.0 /* very coarse resolution */,
                       TessellationStrategy::kDenseInteriorVertices));
  // The coarse resolution should give 8 tetrahedron forming an octahedron.
  ASSERT_EQ(deformable_W.deformable_mesh().mesh().num_elements(), 8);

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
  std::unique_ptr<DeformableContactSurface<double>>
      deformable_contact_surface_W =
          ComputeContactSurfaceFromDeformableVolumeRigidSurface(
              deformable_W, rigid_mesh_R, rigid_bvh_R, X_WR,
              &tetrahedron_index_of_polygons);

  ASSERT_NE(deformable_contact_surface_W, nullptr);
  ASSERT_EQ(tetrahedron_index_of_polygons.size(), 6);

  EXPECT_FALSE(deformable_contact_surface_W->empty());
  EXPECT_EQ(deformable_contact_surface_W->num_polygons(), 6);
  EXPECT_EQ(deformable_contact_surface_W->polygon_data(0).area, 0.0625);
  EXPECT_EQ(deformable_contact_surface_W->polygon_data(0).unit_normal,
            -Vector3<double>::UnitZ());
  EXPECT_EQ(deformable_contact_surface_W->polygon_data(0).centroid,
            Vector3<double>(0.25, 1.0 / 12, 0.5));

  const int kExpectedTetIndex = 0;
  ASSERT_EQ(deformable_contact_surface_W->polygon_data(0).tet_index,
            kExpectedTetIndex);

  Vector4<double> expected_barycentric =
      deformable_W.deformable_mesh().mesh().CalcBarycentric(
          deformable_contact_surface_W->polygon_data(0).centroid,
          deformable_contact_surface_W->polygon_data(0).tet_index);
  EXPECT_EQ(deformable_contact_surface_W->polygon_data(0).b_centroid,
            expected_barycentric);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
