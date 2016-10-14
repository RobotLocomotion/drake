#include "drake/systems/plants/rigid_body_plant/contact_detail.h"

namespace drake {
namespace systems {

template<typename T>
ContactDetail<T>::ContactDetail(const ContactDetail<T>& other) :
    application_point_(other.application_point_),
    wrench_(other.wrench_) {
}

template<typename T>
ContactDetail<T>::ContactDetail(const Vector3 <T> &point,
                                const WrenchVector <T> &wrench)
    : application_point_(point), wrench_(wrench) {}

template<typename T>
ContactDetail<T>* ContactDetail<T>::clone() const {
  return new ContactDetail<T>(*this);
}


// Explicitly instantiates on the most common scalar types.
template class ContactDetail<double>;
}
}
