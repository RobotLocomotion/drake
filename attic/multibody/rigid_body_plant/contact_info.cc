#include "drake/multibody/rigid_body_plant/contact_info.h"

#include "drake/common/default_scalars.h"

using std::move;

namespace drake {
namespace systems {

template <typename T>
ContactInfo<T>::ContactInfo(drake::multibody::collision::ElementId element1,
                            drake::multibody::collision::ElementId element2)
    : element1_(element1),
      element2_(element2),
      resultant_force_(),
      contact_details_() {}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::ContactInfo)
