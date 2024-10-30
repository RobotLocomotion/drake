#include "drake/geometry/proximity/mesh_distance_boundary.h"

#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/calc_signed_distance_to_surface_mesh.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

MeshDistanceBoundary::MeshDistanceBoundary(const VolumeMesh<double>& mesh_M)
    : tri_mesh_M_{ConvertVolumeToSurfaceMesh(mesh_M)},
      tri_bvh_M_{tri_mesh_M_},
      feature_normal_M_{FeatureNormalSet::MaybeCreate(tri_mesh_M_)} {}

MeshDistanceBoundary::MeshDistanceBoundary(TriangleSurfaceMesh<double>&& mesh_M)
    : tri_mesh_M_(std::move(mesh_M)),
      tri_bvh_M_{tri_mesh_M_},
      feature_normal_M_{FeatureNormalSet::MaybeCreate(tri_mesh_M_)} {}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
