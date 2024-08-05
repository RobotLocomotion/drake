#include "drake/geometry/proximity/inflate_mesh.h"

#include <algorithm>
#include <numeric>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/proximity/make_mesh_from_vtk.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

std::vector<double> CalcCellVolumes(const VolumeMesh<double>& mesh) {
  std::vector<double> tet_volumes(mesh.num_elements());
  for (int e = 0; e < mesh.num_elements(); ++e) {
    tet_volumes[e] = mesh.CalcTetrahedronVolume(e);
  }
  return tet_volumes;
}

void VerifyInvariants(const VolumeMesh<double>& mesh,
                      const VolumeMesh<double>& inflated_mesh, double margin) {
  // Verify the one-to-one correspondence of vertices and faces between the
  // original and inflated meshes.
  ASSERT_EQ(mesh.num_vertices(), inflated_mesh.num_vertices());
  ASSERT_EQ(mesh.num_elements(), inflated_mesh.num_elements());
  ASSERT_EQ(mesh.tetrahedra(), inflated_mesh.tetrahedra());

  std::vector<int> surface_to_volume;
  const TriangleSurfaceMesh<double> mesh_surface =
      ConvertVolumeToSurfaceMeshWithBoundaryVerticesAndElementMap(
          mesh, &surface_to_volume);
  const TriangleSurfaceMesh<double> inflated_mesh_surface =
      ConvertVolumeToSurfaceMeshWithBoundaryVerticesAndElementMap(
          inflated_mesh);

  // There is at least one interior vertex. N.B. Not required for InflateMesh(),
  // but hydroelastic meshes do have this property.
  EXPECT_GT(mesh.num_vertices(), surface_to_volume.size());

  // Interior vertices.
  std::vector<int> all_vertices(mesh.num_vertices());
  std::iota(all_vertices.begin(), all_vertices.end(), 0);
  std::vector<int> interior_vertices;
  std::set_difference(all_vertices.begin(), all_vertices.end(),
                      surface_to_volume.begin(), surface_to_volume.end(),
                      std::back_inserter(interior_vertices));

  // Sanity check sets.
  ASSERT_EQ(surface_to_volume.size() + interior_vertices.size(),
            mesh.num_vertices());

  // We verify the interior vertices were not moved.
  for (int v : interior_vertices) {
    const Vector3d& p = mesh.vertex(v);
    const Vector3d& p_inflated = inflated_mesh.vertex(v);
    EXPECT_EQ(p, p_inflated);
  }

  // Verify that vertices produced a displacement of at least "margin" in the
  // direction of each adjacent face.
  for (int e = 0; e < mesh_surface.num_elements(); ++e) {
    const geometry::SurfaceTriangle& tri = mesh_surface.element(e);
    const Vector3d& n = mesh_surface.face_normal(e);

    // Verify the displacement for each vertex of the triangle.
    for (int i = 0; i < 3; ++i) {
      const int v = tri.vertex(i);
      const Vector3d& p = mesh_surface.vertex(v);
      const Vector3d& p_inflated = inflated_mesh_surface.vertex(v);
      const Vector3d u = p_inflated - p;
      ASSERT_GE(u.dot(n), margin);
    }
  }
}

GTEST_TEST(MakeInflatedMesh, NonConvexMesh) {
  // The mesh we use in this test has sides in the order of 1 m. We stress test
  // the inflation method with a large margin value of 0.1 m. In practice, a
  // typical margin value would be 1e-4 m to 1e-3 m for a mesh this size.
  const double kMargin = 0.1;

  const std::string model = "drake/geometry/test/non_convex_mesh.vtk";
  const VolumeMesh<double> mesh =
      MakeVolumeMeshFromVtk<double>(Mesh(FindResourceOrThrow(model)));
  std::vector<int> new_vertices;
  const VolumeMesh<double> inflated_mesh =
      MakeInflatedMesh(mesh, kMargin, &new_vertices);
  EXPECT_EQ(new_vertices.size(), 0);

  std::vector<double> inflated_volumes = CalcCellVolumes(inflated_mesh);
  double min_vol = std::numeric_limits<double>::max();
  for (int e = 0; e < mesh.num_elements(); ++e) {
    const double V = inflated_volumes[e];
    min_vol = std::min(min_vol, V);
  }
  // We expect no element inversion, not even for large margin values.
  EXPECT_GT(min_vol, 0.0);

  VerifyInvariants(mesh, inflated_mesh, kMargin);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
