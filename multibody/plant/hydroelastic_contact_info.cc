#include "drake/multibody/plant/hydroelastic_contact_info.h"

#include "drake/common/never_destroyed.h"

namespace drake {
namespace multibody {

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
