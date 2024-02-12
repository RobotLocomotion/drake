#include "drake/geometry/proximity/volume_mesh.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

template <typename T>
void VolumeMesh<T>::TransformVertices(
    const math::RigidTransform<T>& transform) {
  const math::RigidTransform<T>& X_NM = transform;
  for (Vector3<T>& vertex : vertices_M_) {
    const Vector3<T> p_MV = vertex;
    vertex = X_NM * p_MV;
  }
}

template <typename T>
void VolumeMesh<T>::SetAllPositions(const Eigen::Ref<const VectorX<T>>& p_MVs) {
  if (p_MVs.size() != 3 * num_vertices()) {
    throw std::runtime_error(
        fmt::format("SetAllPositions(): Attempting to deform a mesh with {} "
                    "vertices with data for {} DoFs",
                    num_vertices(), p_MVs.size()));
  }
  for (int v = 0, i = 0; v < num_vertices(); ++v, i += 3) {
    vertices_M_[v] = Vector3<T>(p_MVs[i], p_MVs[i + 1], p_MVs[i + 2]);
  }
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class VolumeMesh)

}  // namespace geometry
}  // namespace drake
