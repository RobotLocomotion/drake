#include "drake/geometry/proximity/make_cylinder_mesh.h"

#include <cmath>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/meshing_utilities.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
VolumeMesh<T> MakeCylinderVolumeMeshWithMa(const Cylinder& cylinder,
                                           const double resolution_hint) {
  const int num_vertices_per_circle =
      static_cast<int>(ceil(2. * M_PI * cylinder.radius() / resolution_hint));

  std::vector<VolumeVertex<T>> mesh_vertices;
  // TODO(DamrongGuoy): Tighten the bound.
  mesh_vertices.reserve(3 * num_vertices_per_circle + 2);

  // For convenience, we add vertices at the centers of the top and bottom
  // caps.
  VolumeVertexIndex cv[2];
  const double top_z = cylinder.length() / 2.;
  const double bottom_z = -top_z;
  cv[0] = VolumeVertexIndex(mesh_vertices.size());
  mesh_vertices.emplace_back(0, 0, bottom_z);
  cv[1] = VolumeVertexIndex(mesh_vertices.size());
  mesh_vertices.emplace_back(0, 0, top_z);

  // Vertices on the two circular rims.
  // v[0][i] is on the bottom -Z circular rim.
  // v[1][i] is on the top +Z circular rim.
  VolumeVertexIndex v[2][num_vertices_per_circle];
  const double angle_step = 2. * M_PI / num_vertices_per_circle;
  for (int i=0; i < num_vertices_per_circle; ++i) {
    const double x = cylinder.radius() * cos(angle_step * i);
    const double y = cylinder.radius() * sin(angle_step * i);
    v[0][i] = VolumeVertexIndex(mesh_vertices.size());
    mesh_vertices.emplace_back(x, y, bottom_z);
    v[1][i] = VolumeVertexIndex(mesh_vertices.size());
    mesh_vertices.emplace_back(x, y, top_z);
  }

  // For a long cylinder, these two vertices are at the two end points of the
  // central medial line segment. For a short cylinder, these two virtual
  // vertices point to the same mesh vertex at the center of the central
  // medial circular disk for convenience. For a medium cylinder, both virtual
  // vertices point to the same mesh vertex at the origin of the frame of
  // the cylinder.
  VolumeVertexIndex cm[2];
  const double shorter_half = std::min(top_z, cylinder.radius());
  const double offset_distance = shorter_half;
  const double offset_top_z = top_z - offset_distance;
  const double offset_bottom_z = -offset_top_z;
  cm[0] = VolumeVertexIndex(mesh_vertices.size());
  mesh_vertices.emplace_back(0, 0, offset_bottom_z);
  if (top_z > cylinder.radius()) {
    // Long cylinder.
    cm[1] = VolumeVertexIndex(mesh_vertices.size());
    mesh_vertices.emplace_back(0, 0, offset_top_z);
  } else {
    // Medium cylinder or short cylinder.
    cm[1] = cm[0];
  }

  // For a long cylinder, all virtual vertices m[0][i] point to the same mesh
  // vertex as cm[0], and all virtual vertices m[1][i] point to the same mesh
  // vertex as cm[1]. For a short cylinder, all virtual vertices m[0][i] and
  // m[1][i] are at the same boundary circle of the central medial disk.
  // For a medium cylinder, all m[0][i] and m[1][i] point to the same vertex
  // as cm[0] = cm[1].
  VolumeVertexIndex m[2][num_vertices_per_circle];
  const double offset_radius = cylinder.radius() - offset_distance;
  const double scale_cylinder_radius_to_offset_surface =
  offset_radius / cylinder.radius();
  for (int i = 0; i < num_vertices_per_circle; ++i) {
    if (top_z >= cylinder.radius()) {
      // Long cylinder or medium cylinder.
      m[0][i] = cm[0];
      m[1][i] = cm[1];
    } else {
      // Short cylinder.
      const double x = ExtractDoubleOrThrow(mesh_vertices[v[0][i]].r_MV().x()) *
          scale_cylinder_radius_to_offset_surface;
      const double y = ExtractDoubleOrThrow(mesh_vertices[v[0][i]].r_MV().y()) *
          scale_cylinder_radius_to_offset_surface;
      m[0][i] = VolumeVertexIndex(mesh_vertices.size());
      mesh_vertices.emplace_back(x, y, offset_bottom_z);
      m[1][i] = m[0][i];
    }
  }

  std::vector<VolumeElement> mesh_elements;
  auto append =
      [&mesh_elements](const std::vector<VolumeElement>& new_elements) {
        mesh_elements.insert(mesh_elements.end(), new_elements.begin(),
                             new_elements.end());
      };

  // TODO(DamrongGuoy): Tighten the bound.
  mesh_elements.reserve(6 * num_vertices_per_circle +
                        6 * num_vertices_per_circle +
                        6 * num_vertices_per_circle);
  // Virtual rectangular boxes.
  // i = (n-1) 0 1 2 ... (n-2)
  // j =   0   1 2 3 ... (n-1)
  int i = num_vertices_per_circle - 1;
  for (int j = 0; j < num_vertices_per_circle; ++j) {
    append(SplitToTetrahedra(m[0][j], m[1][j], m[1][i], m[0][i],    // medial
                             v[0][j], v[1][j], v[1][i], v[0][i]));  // cylinder
    i = j;
  }

  // Bottom virtual triangular prisms.
  i = num_vertices_per_circle - 1;
  for (int j = 0; j < num_vertices_per_circle; ++j) {
    append(SplitToTetrahedra(m[0][j], v[0][j], v[0][i], m[0][i], cm[0], cv[0],
                             cv[0], cm[0]));
    i = j;
  }

  // Top virtual triangular prisms.
  i = num_vertices_per_circle - 1;
  for (int j = 0; j < num_vertices_per_circle; ++j) {
    append(SplitToTetrahedra(m[1][j], m[1][i], v[1][i], v[1][j], cm[1], cm[1],
                             cv[1], cv[1]));
    i = j;
  }

  return {std::move(mesh_elements), std::move(mesh_vertices)};
}

template VolumeMesh<double> MakeCylinderVolumeMeshWithMa(
    const Cylinder&, double resolution_hint);
template VolumeMesh<AutoDiffXd> MakeCylinderVolumeMeshWithMa(
    const Cylinder&, double resolution_hint);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
