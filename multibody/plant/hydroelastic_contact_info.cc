#include "drake/multibody/plant/hydroelastic_contact_info.h"

#include "drake/common/never_destroyed.h"

namespace drake {
namespace multibody {
namespace {

using geometry::ContactSurface;

// If `other` is already shared, then we can alias it. Otherwise, we need to
// clone the bare pointer so that it can be shared from now on.
template <typename T>
std::shared_ptr<const ContactSurface<T>> ConstructSharedSurface(
    const std::shared_ptr<const ContactSurface<T>>& other) {
  DRAKE_DEMAND(other != nullptr);
  return (other.use_count() > 0)
             ? other
             : std::make_shared<const ContactSurface<T>>(*other);
}

}  // namespace

template <typename T>
HydroelasticContactInfo<T>::HydroelasticContactInfo(
    const HydroelasticContactInfo& other)
    : contact_surface_(ConstructSharedSurface(other.contact_surface_)),
      F_Ac_W_(other.F_Ac_W_) {}

template <typename T>
HydroelasticContactInfo<T>& HydroelasticContactInfo<T>::operator=(
    const HydroelasticContactInfo& other) {
  if (this != &other) {
    contact_surface_ = ConstructSharedSurface(other.contact_surface_);
    F_Ac_W_ = other.F_Ac_W_;
  }
  return *this;
}

template <typename T>
HydroelasticContactInfo<T>::~HydroelasticContactInfo() = default;

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::HydroelasticContactInfo);
