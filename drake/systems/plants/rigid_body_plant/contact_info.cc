#include "drake/systems/plants/rigid_body_plant/contact_info.h"

using std::move;

namespace drake {
namespace systems {

template <typename T>
ContactInfo<T>::ContactInfo(DrakeCollision::ElementId element1,
                            DrakeCollision::ElementId element2,
                            std::unique_ptr<ContactManifold<T>> manifold) :
    element1_(element1), element2_(element2), contact_manifold_(move(manifold))
{}

template <typename T>
ContactInfo<T>::ContactInfo(const ContactInfo<T>& other) :
    element1_(other.element1_),
    element2_(other.element2_),
    contact_manifold_(move(other.contact_manifold_->Clone())) {
}

template <typename T>
ContactInfo<T>& ContactInfo<T>::operator=(const ContactInfo<T>& other) {
  if ( this == &other) return *this;
  element1_ = other.element1_;
  element2_ = other.element2_;
  contact_manifold_ = move(other.contact_manifold_->Clone());
  return *this;
}

template <typename T>
DrakeCollision::ElementId ContactInfo<T>::get_element_id_1() const {
  return element1_;
}

template <typename T>
DrakeCollision::ElementId ContactInfo<T>::get_element_id_2() const {
  return element2_;
}

template <typename T>
const ContactManifold<T>& ContactInfo<T>::get_contact_manifold() const {
  return *contact_manifold_.get();
}

// Explicitly instantiates on the most common scalar types.
template class ContactInfo<double>;

}  // namespace systems
}  // namespace drake
