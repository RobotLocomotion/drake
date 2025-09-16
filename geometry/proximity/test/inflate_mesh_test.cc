#include "drake/geometry/proximity/inflate_mesh.h"

#include <algorithm>
#include <limits>
#include <map>
#include <numeric>
#include <string>
#include <vector>

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
                      const VolumeMesh<double>& inflated_mesh, double margin,
                      bool has_interior_vertices = true) {
  // Verify the one-to-one correspondence of faces between the
  // original and inflated meshes. The inflated mesh may have added vertices.
  ASSERT_LE(mesh.num_vertices(), inflated_mesh.num_vertices());
  ASSERT_EQ(mesh.num_elements(), inflated_mesh.num_elements());

  std::vector<int> surface_to_volume;
  std::vector<TetFace> element_map;
  const TriangleSurfaceMesh<double> mesh_surface =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(mesh, &surface_to_volume,
                                                     &element_map);

  // If there is at least one interior vertex. N.B. Not required for
  // InflateMesh(), but hydroelastic meshes do have this property.
  if (has_interior_vertices) {
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
  }

  // Verify that vertices produced a displacement of at least "margin" in the
  // direction of each adjacent face.
  // N.B. Not all solvers guarantee strict feasibility. Therefore we verify
  // feasibility using a numerical slop. We write this as:
  //   u⋅n̂ ≥ δ⋅(1−εᵣ)
  // with εᵣ a dimensionless relative slop.
  const double kRelativeSlop = 8 * std::numeric_limits<double>::epsilon();
  for (int f = 0; f < mesh_surface.num_triangles(); ++f) {
    const Vector3d& n = mesh_surface.face_normal(f);
    // Face `f` corresponds to the `k`-th local face of element `e` in both the
    // original mesh and inflated mesh.
    auto [e, k] = element_map[f];

    // Verify the displacement for each vertex of the triangle.
    for (int i = 1; i < 4; ++i) {
      const Vector3d& p = mesh.vertex(mesh.element(e).vertex((k + i) % 4));
      const Vector3d& p_inflated =
          inflated_mesh.vertex(inflated_mesh.element(e).vertex((k + i) % 4));
      const Vector3d u = p_inflated - p;
      ASSERT_GE(u.dot(n), margin * (1.0 - kRelativeSlop));
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
  std::map<int, int> split_vertex_to_original;
  const VolumeMesh<double> inflated_mesh =
      MakeInflatedMesh(mesh, kMargin, &split_vertex_to_original);

  // All vertices in the input mesh should have a feasible set of constraints.
  EXPECT_EQ(ssize(split_vertex_to_original), 0);

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

GTEST_TEST(MakeInflatedMesh, SingleInfeasibleVertex) {
  // The mesh we use in this test has a single vertex whose adjacent face set
  // results in an infeasible set of constraints for the QP.
  const double kMargin = 0.1;

  const std::string model =
      "drake/geometry/test/inflation_infeasible_vertex.vtk";
  const VolumeMesh<double> mesh =
      MakeVolumeMeshFromVtk<double>(Mesh(FindResourceOrThrow(model)));
  std::map<int, int> split_vertex_to_original;
  // The index of the infeasible vertex in the input mesh. This must be updated
  // if the model is changed.
  const int infeasible_vertex_index = 7;
  // The number of tetrahedral elements adjacent to the infeasible vertex. This
  // must be updated if the model is changed.
  const int num_adjacent_elements = 8;

  // The infeasible vertex is adjacent to 9 faces and 8 elements. The vertex
  // will split into a vertex for each adjacent element, resulting in 7 new
  // vertices
  const VolumeMesh<double> inflated_mesh =
      MakeInflatedMesh(mesh, kMargin, &split_vertex_to_original);
  EXPECT_EQ(inflated_mesh.num_vertices(),
            mesh.num_vertices() + num_adjacent_elements - 1);

  EXPECT_EQ(ssize(split_vertex_to_original), num_adjacent_elements - 1);
  for (int i = mesh.num_vertices();
       i < mesh.num_vertices() + num_adjacent_elements - 1; ++i) {
    EXPECT_TRUE(split_vertex_to_original.contains(i));
    EXPECT_EQ(split_vertex_to_original.at(i), infeasible_vertex_index);
  }

  std::vector<double> inflated_volumes = CalcCellVolumes(inflated_mesh);
  double min_vol = std::numeric_limits<double>::max();
  for (int e = 0; e < mesh.num_elements(); ++e) {
    const double V = inflated_volumes[e];
    min_vol = std::min(min_vol, V);
  }
  // For this particular mesh, we expect no element inversion, not even for
  // large margin values.
  EXPECT_GT(min_vol, 0.0);

  VerifyInvariants(mesh, inflated_mesh, kMargin,
                   /* has_interior_vertices */ false);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
