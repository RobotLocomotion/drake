#include "drake/multibody/rigid_body_plant/contact_detail.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

template <typename T>
ContactDetail<T>::ContactDetail() {}

template <typename T>
ContactDetail<T>::~ContactDetail() {}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::ContactDetail)
