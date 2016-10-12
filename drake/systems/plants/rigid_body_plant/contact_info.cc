#include "drake/systems/plants/rigid_body_plant/contact_info.h"

namespace drake {
namespace systems {

template <typename T>
ContactInfo<T>::ContactInfo(DrakeCollision::ElementId element1,
                            DrakeCollision::ElementId element2) :
    element1_(element1), element2_(element2) {}

template <typename T>
ContactInfo<T>::ContactInfo(const ContactInfo<T>& other) :
    element1_(other.element1_),
    element2_(other.element2_),
    contact_manifold_(other.contact_manifold_->clone())
{
}

template <typename T>
ContactInfo<T>* ContactInfo<T>::clone() const {
  return new ContactInfo<T>(*this);
}

template <typename T>
void ContactInfo<T>::set_manifold(std::unique_ptr<ContactManifold<T>> manifold)
{
  contact_manifold_.reset(manifold.release());
}

// explicit instantiation
template class ContactInfo<double>;

}
}