#include "drake/multibody/plant/hydroelastic_contact_info.h"

namespace drake {
namespace multibody {

template <typename T>
HydroelasticContactInfo<T>::~HydroelasticContactInfo() = default;

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::HydroelasticContactInfo)
