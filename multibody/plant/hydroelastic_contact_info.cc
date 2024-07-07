#include "drake/multibody/plant/hydroelastic_contact_info.h"

#include "drake/common/never_destroyed.h"

namespace drake {
namespace multibody {

using geometry::ContactSurface;

// If `other` has its surface already shared, then we can alias it.
// Otherwise, we need to clone the bare pointer to be shared.
template <typename T>
HydroelasticContactInfo<T>::HydroelasticContactInfo(
    const HydroelasticContactInfo& other)
    : contact_surface_((other.contact_surface_.use_count() > 0)
                           ? other.contact_surface_
                           : std::make_shared<const ContactSurface<T>>(
                                 *other.contact_surface_)),
      F_Ac_W_(other.F_Ac_W_) {}

// If `other` has its surface already shared, then we can alias it.
// Otherwise, we need to clone the bare pointer to be shared.
template <typename T>
HydroelasticContactInfo<T>& HydroelasticContactInfo<T>::operator=(
    const HydroelasticContactInfo& other) {
  if (this != &other) {
    contact_surface_ = (other.contact_surface_.use_count() > 0)
                           ? other.contact_surface_
                           : std::make_shared<const ContactSurface<T>>(
                                 *other.contact_surface_);
    F_Ac_W_ = other.F_Ac_W_;
  }
  return *this;
}

template <typename T>
HydroelasticContactInfo<T>::~HydroelasticContactInfo() = default;

template <typename T>
const std::vector<HydroelasticQuadraturePointData<T>>&
HydroelasticContactInfo<T>::quadrature_point_data() const {
  using Result = std::vector<HydroelasticQuadraturePointData<T>>;
  static const never_destroyed<Result> result;
  return result.access();
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::HydroelasticContactInfo);
