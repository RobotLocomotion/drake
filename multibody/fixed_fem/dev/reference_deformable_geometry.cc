#include "drake/multibody/fixed_fem/dev/reference_deformable_geometry.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

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
  signed_distance_field_ =
      s.signed_distance_field().CloneAndSetMesh(mesh_.get());

  return *this;
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::ReferenceDeformableGeometry);
