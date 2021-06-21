#include "drake/geometry/proximity/deformable_volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
void DeformableVolumeMesh<T>::UpdateVertexPositions(
    const Eigen::Ref<const VectorX<T>>& q) {
  deformer_.SetAllPositions(q);
  bvh_updater_.Update();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::internal::DeformableVolumeMesh)
