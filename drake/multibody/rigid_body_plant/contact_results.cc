#include "drake/multibody/rigid_body_plant/contact_results.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

template <typename T>
int ContactResults<T>::get_num_contacts() const {
  return static_cast<int>(contacts_.size());
}

template <typename T>
const ContactInfo<T>& ContactResults<T>::get_contact_info(int i) const {
  DRAKE_ASSERT(i >= 0 && i < static_cast<int>(contacts_.size()));
  return contacts_[i];
}

template <typename T>
ContactResults<T>::ContactResults() : contacts_() {}

template <typename T>
void ContactResults<T>::Clear() {
  contacts_.clear();
}

template <typename T>
ContactInfo<T>& ContactResults<T>::AddContact(
    DrakeCollision::ElementId element_a,
    DrakeCollision::ElementId element_b) {
  contacts_.emplace_back(element_a, element_b);
  return contacts_.back();
}

// Explicitly instantiates on the most common scalar types.
template class ContactResults<double>;

}  // namespace systems
}  // namespace drake
