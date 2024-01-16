#include "drake/geometry/proximity/deformable_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename MeshType>
void DeformableMesh<MeshType>::UpdateVertexPositions(
    const Eigen::Ref<const VectorX<T>>& q) {
  deformer_.SetAllPositions(q);
  bvh_updater_.Update();
}

template class DeformableMesh<VolumeMesh<double>>;
template class DeformableMesh<VolumeMesh<AutoDiffXd>>;
template class DeformableMesh<TriangleSurfaceMesh<double>>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
