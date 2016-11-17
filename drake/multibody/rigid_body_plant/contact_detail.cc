#include "drake/multibody/rigid_body_plant/contact_detail.h"

namespace drake {
namespace systems {

template <typename T>
ContactDetail<T>::~ContactDetail() {}

// Explicitly instantiates on the most common scalar types.
template class ContactDetail<double>;

}  // namespace systems
}  // namespace drake
