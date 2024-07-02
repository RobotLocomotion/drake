#include "drake/geometry/proximity/calc_signed_distance_to_volume_mesh.h"

#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/calc_signed_distance_to_surface_mesh.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

VolumeMeshSignedDistance::VolumeMeshSignedDistance(
    const VolumeMesh<double>& mesh_M)
    : tri_mesh_M_{ConvertVolumeToSurfaceMesh(mesh_M)},
      tri_bvh_M_{tri_mesh_M_},
      vertex_edge_normal_M_{tri_mesh_M_} {}

VolumeMeshSignedDistance::SignedDistanceData
VolumeMeshSignedDistance::CalcSignedDistance(
    const Vector3<double>& p_MQ) const {
  // I had considered an alternative algorithm to find a tetrahedron
  // containing the query point Q. It would determine whether Q is inside or
  // outside. However, it cannot tell how far Q is from the boundary surface,
  // where the nearest point on the surface is, and what is the gradient.
  //    For the current implementation, I decided to calculate the signed
  // distance data from the boundary surface mesh.
  const SignedDistanceToSurfaceMesh d = CalcSignedDistanceToSurfaceMesh(
      p_MQ, tri_mesh_M_, tri_bvh_M_, vertex_edge_normal_M_);
  return {d.signed_distance, d.nearest_point, d.gradient};
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
