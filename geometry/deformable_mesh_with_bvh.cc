#include "drake/geometry/deformable_mesh_with_bvh.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename MeshType>
void DeformableMeshWithBvh<MeshType>::UpdateVertexPositions(
    const Eigen::Ref<const VectorX<T>>& q) {
  deformable_mesh_.SetAllPositions(q);
  bvh_updater_.Update();
}

template class DeformableMeshWithBvh<VolumeMesh<double>>;
template class DeformableMeshWithBvh<VolumeMesh<AutoDiffXd>>;
template class DeformableMeshWithBvh<TriangleSurfaceMesh<double>>;
template class DeformableMeshWithBvh<TriangleSurfaceMesh<AutoDiffXd>>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
