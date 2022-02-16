#include "drake/geometry/proximity/volume_mesh.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

template <typename T>
void VolumeMesh<T>::Transform(const math::RigidTransform<T>& transform) {
  for (Vector3<T>& vertex : vertices_) {
    vertex = Vector3<T>(transform * vertex);
  }
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class VolumeMesh)

}  // namespace geometry
}  // namespace drake
