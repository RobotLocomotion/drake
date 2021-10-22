#include "drake/geometry/proximity/mesh_deformer.h"

#include <fmt/format.h>

#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename MeshType>
void MeshDeformer<MeshType>::SetAllPositions(
    const Eigen::Ref<const VectorX<T>>& p_MVs) {
  if (p_MVs.size() != 3 * mesh_.num_vertices()) {
    throw std::runtime_error(fmt::format(
        "MeshDeformer::SetAllPositions(): Attempting to deform a mesh with {} "
        "vertices with data for {} vertices",
        mesh_.num_vertices(), p_MVs.size()));
  }
  for (int v = 0, i = 0; v < mesh_.num_vertices(); ++v, i += 3) {
    mesh_.vertices_[v] = Vector3<T>(p_MVs[i], p_MVs[i + 1], p_MVs[i + 2]);
  }
}

template class MeshDeformer<VolumeMesh<double>>;
template class MeshDeformer<VolumeMesh<AutoDiffXd>>;
template class MeshDeformer<TriangleSurfaceMesh<double>>;
template class MeshDeformer<TriangleSurfaceMesh<AutoDiffXd>>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
