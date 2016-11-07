#include "drake/systems/plants/rigid_body_plant/contact_info.h"

using std::move;

namespace drake {
namespace systems {

template <typename T>
ContactInfo<T>::ContactInfo(DrakeCollision::ElementId element1,
                            DrakeCollision::ElementId element2)
    : element1_(element1),
      element2_(element2),
      resultant_force_(),
      contact_details_() {}

template <typename T>
ContactInfo<T>::ContactInfo(const ContactInfo<T>& other)
    : element1_(other.element1_),
      element2_(other.element2_),
      resultant_force_(move(other.resultant_force_)) {
  contact_details_.reserve(other.contact_details_.size());
  for (const auto& detail : other.contact_details_) {
    std::unique_ptr<ContactDetail<T>> detail_copy(detail->Clone());
    contact_details_.push_back(std::move(detail_copy));
  }
}

template <typename T>
ContactInfo<T>& ContactInfo<T>::operator=(const ContactInfo<T>& other) {
  if (this == &other) return *this;
  element1_ = other.element1_;
  element2_ = other.element2_;
  resultant_force_ = other.resultant_force_;
  contact_details_.clear();
  contact_details_.reserve(other.contact_details_.size());
  for (const auto& detail : other.contact_details_) {
    std::unique_ptr<ContactDetail<T>> detail_copy(detail->Clone());
    contact_details_.push_back(std::move(detail_copy));
  }
  return *this;
}

// Explicitly instantiates on the most common scalar types.
template class ContactInfo<double>;

}  // namespace systems
}  // namespace drake
