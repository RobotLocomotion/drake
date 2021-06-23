#include "drake/multibody/fixed_fem/dev/reference_deformable_geometry.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
ReferenceDeformableGeometry<T>::ReferenceDeformableGeometry(
    std::unique_ptr<geometry::VolumeMesh<T>> mesh,
    std::unique_ptr<geometry::VolumeMeshFieldLinear<T, T>> signed_distance)
    : mesh_(std::move(mesh)), signed_distance_(std::move(signed_distance)) {
  DRAKE_DEMAND(mesh_.get() == &signed_distance_->mesh());
  const std::vector<T>& vertex_values = signed_distance_->values();
  for (const T& value : vertex_values) {
    DRAKE_ASSERT(value <= 0.0);
  }
}

template <typename T>
ReferenceDeformableGeometry<T>::ReferenceDeformableGeometry(
    const ReferenceDeformableGeometry<T>& s) {
  *this = s;
}

template <typename T>
ReferenceDeformableGeometry<T>& ReferenceDeformableGeometry<T>::operator=(
    const ReferenceDeformableGeometry<T>& s) {
  if (this == &s) return *this;

  mesh_ = std::make_unique<geometry::VolumeMesh<T>>(s.mesh());
  /* We can't simply copy the mesh field; the copy must contain a pointer to
   the new mesh. So, we use CloneAndSetMesh() instead. */
  signed_distance_ = s.signed_distance().CloneAndSetMesh(mesh_.get());

  return *this;
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::ReferenceDeformableGeometry);
