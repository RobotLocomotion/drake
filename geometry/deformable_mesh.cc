#include "drake/geometry/deformable_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename MeshType>
void DeformableMesh<MeshType>::UpdateVertexPositions(
    const Eigen::Ref<const VectorX<T>>& q) {
  deformer_.SetAllPositions(q);
}

template class DeformableMesh<TriangleSurfaceMesh<double>>;
template class DeformableMesh<VolumeMesh<double>>;
template class DeformableMesh<VolumeMesh<AutoDiffXd>>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
