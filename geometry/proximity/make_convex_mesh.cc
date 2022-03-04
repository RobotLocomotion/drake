#include "drake/geometry/proximity/make_convex_mesh.h"

#include <cmath>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

Vector3d ComputeCentroid(const TriangleSurfaceMesh<double>& surface_mesh) {
  double six_total_volume = 0;
  Vector3d centroid(0, 0, 0);

  // For convenience we tetrahedralize each triangle (p,q,r) about the
  // origin o = (0,0,0) in the mesh's frame. The centroid is then the
  // signed volume weighted sum of the centroids of the tetraheda. The
  // choice of o is arbitrary and need not be on the interior of surface_mesh.
  // For efficiency we compute 6*Vi for the signed volume Vi of the ith element
  // and 6*V for the signed volume V of the region.
  for (const auto& tri : surface_mesh.triangles()) {
    const Vector3d& p = surface_mesh.vertex(tri.vertex(0));
    const Vector3d& q = surface_mesh.vertex(tri.vertex(1));
    const Vector3d& r = surface_mesh.vertex(tri.vertex(2));

    // 6 * signed volume of (p,q,r,o)
    const double six_volume = (p).cross(q).dot(r);
    six_total_volume += six_volume;

    // Centroid of tetrahedron (p,q,r,o) is (p + q + r + o) / 4.
    // We factor out the division for added precision and accuracy.
    centroid += six_volume * (p + q + r);
  }

  return centroid / (4 * six_total_volume);
}

template <typename T>
VolumeMesh<T> MakeConvexVolumeMesh(
    const TriangleSurfaceMesh<double>& surface_mesh) {
  std::vector<Vector3<T>> mesh_vertices(surface_mesh.vertices().begin(),
                                        surface_mesh.vertices().end());

  const Vector3d& centroid = ComputeCentroid(surface_mesh);

  mesh_vertices.push_back(centroid);

  int centroid_index = mesh_vertices.size() - 1;

  std::vector<VolumeElement> mesh_elements;

  // Partition the volume encompased by `surface_mesh` into tetrahedra
  // that all share the interior vertex, `centroid`.
  // TODO(joemasterjohn): Add verification checks in debug builds for the
  //   input meshes.
  //   Things we could possibly check:
  //     - Convexity
  //     - Closure
  //     - Non-repeated vertices
  //     - Consistently oriented face normals
  for (auto e : surface_mesh.triangles()) {
    const Vector3d& p = surface_mesh.vertices()[e.vertex(0)];
    const Vector3d& q = surface_mesh.vertices()[e.vertex(1)];
    const Vector3d& r = surface_mesh.vertices()[e.vertex(2)];

    // Triangle pqr's normal vector.
    const Vector3d n = (q - p).cross(r - p);

    // Orient the tetrahedron such that it has positive signed volume.
    if (n.dot(p - centroid) > 0) {
      // Outward facing normal.
      mesh_elements.push_back(
          {e.vertex(0), e.vertex(2), e.vertex(1), centroid_index});
    } else {
      // Inward facing normal.
      mesh_elements.push_back(
          {e.vertex(0), e.vertex(1), e.vertex(2), centroid_index});
    }
  }

  return {std::move(mesh_elements), std::move(mesh_vertices)};
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&MakeConvexVolumeMesh<T>))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
