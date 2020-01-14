#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/** Creates a volume mesh for the given `ellipsoid`; the level of
 tessellation is guided by the `resolution_hint` parameter.

 `resolution_hint` influences the resolution of the mesh. Smaller values
 create higher-resolution meshes with smaller tetrahedra. The resolution hint
 is interpreted as an upper bound on the edge lengths along the great
 ellipses on each of the three coordinate planes of the frame of `ellipsoid`.

 The resolution of the final mesh will change discontinuously. Small changes
 to `resolution_hint` will likely produce the same mesh. However, in the
 current implementation, cutting `resolution_hint` in half _will_ increase
 the number of tetrahedra.

 Ultimately, successively smaller values of `resolution_hint` will no longer
 change the output mesh. This algorithm will not produce a tetrahedral mesh with
 more than approximately 100 million tetrahedra. Similarly, for arbitrarily
 large values of `resolution_hint`, the coarsest possible mesh is a tessellated
 octohedron.

 @param ellipsoid           The ellipsoid for which a mesh is created.
 @param resolution_hint     The positive characteristic edge length for the
                            ellipsoid. The coarsest possible mesh (an
                            octahedron) is guaranteed for any value of
                            `resolution_hint` greater than or equal to the
                            `ellipsoid`'s major axis.
 @return The volume mesh for the given ellipsoid.
 @tparam T The Eigen-compatible scalar for representing the mesh vertex
           positions.
*/
template <typename T>
VolumeMesh<T> MakeEllipsoidVolumeMesh(const Ellipsoid& ellipsoid,
                                      double resolution_hint) {
  DRAKE_DEMAND(resolution_hint > 0.0);
  const double a = ellipsoid.a();
  const double b = ellipsoid.b();
  const double c = ellipsoid.c();
  const double r = std::max({a, b, c});

  const double unit_sphere_resolution = resolution_hint / r;
  auto unit_sphere_mesh =
      MakeSphereVolumeMesh<T>(Sphere(1.0), unit_sphere_resolution);

  const Vector3<T> scale{a, b, c};
  std::vector<VolumeVertex<T>> vertices;
  vertices.reserve(unit_sphere_mesh.num_vertices());
  for (const auto& sphere_vertex : unit_sphere_mesh.vertices()) {
    vertices.emplace_back(scale.cwiseProduct(sphere_vertex.r_MV()));
  }
  std::vector<VolumeElement> tetrahedra = unit_sphere_mesh.tetrahedra();

  return VolumeMesh<T>(std::move(tetrahedra), std::move(vertices));
}

/** Creates a surface mesh for the given `ellipsoid`; the level of
 tessellation is guided by the `resolution_hint` parameter in the same way as
 MakeEllipsoidVolumeMesh.

 @param ellipsoid           The ellipsoid for which a surface mesh is created.
 @param resolution_hint     The positive characteristic edge length for the
                            ellipsoid. The coarsest possible mesh (an
                            octahedron) is guaranteed for any value of
                            `resolution_hint` greater than or equal to the
                            `ellipsoid`'s major axis.
 @return The triangulated surface mesh for the given ellipsoid.
 @tparam T The Eigen-compatible scalar for representing the mesh vertex
           positions.
*/
template <typename T>
SurfaceMesh<T> MakeEllipsoidSurfaceMesh(const Ellipsoid& ellipsoid,
                                        double resolution_hint) {
  DRAKE_DEMAND(resolution_hint > 0.0);
  return ConvertVolumeToSurfaceMesh<T>(
      MakeEllipsoidVolumeMesh<T>(ellipsoid, resolution_hint));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
