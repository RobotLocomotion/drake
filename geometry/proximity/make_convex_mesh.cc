#include "drake/geometry/proximity/make_convex_mesh.h"

#include <cmath>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

namespace {

/* Returns the centroid of the volume enclosed by the surface mesh.
   It is not necessarily the same as the centroid of the enclosing surface. */
Vector3d CalcCentroidOfEnclosedVolume(
    const TriangleSurfaceMesh<double>& surface_mesh) {
  // TODO(DamrongGuoy): Make the declaration of this function available in a
  // header file, possibly meshing_utilities.h, so we can have a simple unit
  // test.

  double six_total_volume = 0;
  Vector3d centroid(0, 0, 0);

  // For convenience we tetrahedralize each triangle (p,q,r) about the
  // origin o = (0,0,0) in the mesh's frame. The centroid is then the
  // signed volume weighted sum of the centroids of the tetrahedra. The
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

}  // namespace

template <typename T>
VolumeMesh<T> MakeConvexVolumeMesh(
    const TriangleSurfaceMesh<double>& surface_mesh) {
  std::vector<Vector3<T>> volume_mesh_vertices(surface_mesh.vertices().begin(),
                                               surface_mesh.vertices().end());

  const Vector3d centroid = CalcCentroidOfEnclosedVolume(surface_mesh);
  volume_mesh_vertices.push_back(centroid);

  const int centroid_index = volume_mesh_vertices.size() - 1;

  // The number of tetrahedra in the volume mesh is the same as the number of
  // triangles in the surface mesh.
  std::vector<VolumeElement> volume_mesh_elements;
  volume_mesh_elements.reserve(surface_mesh.num_elements());

  // Partition the volume encompassed by `surface_mesh` into tetrahedra
  // that all share the interior vertex, `centroid`.
  // TODO(joemasterjohn): Add verification checks in debug builds for the
  //   input meshes.
  //   Things we could possibly check:
  //     - Convexity
  //     - Closure
  //     - Non-repeated vertices
  //     - Consistently outwardly oriented face normals
  //   Note: If this is invoked with the mesh corresponding to Drake's computed
  //   convex hull, such tests are unnecessary. Currently, we *do* invoke it
  //   with such a mesh, but are not signaling it (because this code wouldn't do
  //   anything differently if we did). When we have the ability to test/check
  //   arbitrary, meshes we should exclude testing when we know it is a valid
  //   convex mesh by construction.
  for (const SurfaceTriangle& e : surface_mesh.triangles()) {
    // Orient the tetrahedron such that it has positive signed volume (assuming
    // outward facing normal for the surface triangle).
    volume_mesh_elements.push_back(
        {centroid_index, e.vertex(0), e.vertex(1), e.vertex(2)});
  }

  return {std::move(volume_mesh_elements), std::move(volume_mesh_vertices)};
}

template <typename T>
VolumeMesh<T> MakeConvexVolumeMesh(const Convex& convex) {
  return MakeConvexVolumeMesh<T>(
      internal::MakeTriangleFromPolygonMesh(convex.GetConvexHull()));
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (static_cast<VolumeMesh<T> (*)(const Convex&)>(&MakeConvexVolumeMesh<T>),
     static_cast<VolumeMesh<T> (*)(const TriangleSurfaceMesh<double>&)>(
         &MakeConvexVolumeMesh<T>)));

}  // namespace internal
}  // namespace geometry
}  // namespace drake
