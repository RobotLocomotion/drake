#include "drake/multibody/rigid_body_plant/contact_detail.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace systems {

template <typename T>
ContactDetail<T>::ContactDetail() {}

template <typename T>
ContactDetail<T>::~ContactDetail() {}

// Explicitly instantiates on the most common scalar types.
template class ContactDetail<double>;
template class ContactDetail<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
