#include "drake/systems/plants/rigid_body_plant/contact_detail.h"

namespace drake {
namespace systems {
template <typename T>
ContactDetail<T>::ContactDetail(const Vector3<T>& point,
                                const WrenchVector<T>& wrench)
    : application_point_(point), wrench_(wrench) {}

template <typename T>
std::unique_ptr<ContactDetail<T>> ContactDetail<T>::Clone() const {
  return std::make_unique<ContactDetail<T>>(*this);
}

// Explicitly instantiates on the most common scalar types.
template class ContactDetail<double>;

}  // namespace systems
}  // namespace drake
