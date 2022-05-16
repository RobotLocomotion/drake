#include "drake/geometry/proximity/volume_mesh.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

template <typename T>
void VolumeMesh<T>::TransformVertices(
    const math::RigidTransform<T>& transform) {
  const math::RigidTransform<T>& X_NM = transform;
  for (Vector3<T>& vertex : vertices_) {
    const Vector3<T> p_MV = vertex;
    vertex = X_NM * p_MV;
  }
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class VolumeMesh)

}  // namespace geometry
}  // namespace drake
