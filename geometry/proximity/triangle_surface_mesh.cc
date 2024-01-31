#include "drake/geometry/proximity/triangle_surface_mesh.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

template <typename T>
void TriangleSurfaceMesh<T>::SetAllPositions(
    const Eigen::Ref<const VectorX<T>>& p_MVs) {
  if (p_MVs.size() != 3 * num_vertices()) {
    throw std::runtime_error(
        fmt::format("SetAllPositions(): Attempting to deform a mesh with {} "
                    "vertices with data for {} DoFs",
                    num_vertices(), p_MVs.size()));
  }
  for (int v = 0, i = 0; v < num_vertices(); ++v, i += 3) {
    vertices_M_[v] = Vector3<T>(p_MVs[i], p_MVs[i + 1], p_MVs[i + 2]);
  }
  // Update position dependent quantities after the vertex positions have been
  // updated.
  ComputePositionDependentQuantities();
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class TriangleSurfaceMesh)

}  // namespace geometry
}  // namespace drake
