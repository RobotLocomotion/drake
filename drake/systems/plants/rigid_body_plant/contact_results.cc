#include "drake/systems/plants/rigid_body_plant/contact_results.h"

#include "drake/common/drake_assert.h"
#include "drake/systems/plants/rigid_body_plant/sampled_contact_manifold.h"

using std::make_unique;
using std::move;

namespace drake {
namespace systems {


template <typename T>
size_t ContactResults<T>::get_num_contacts() const { return contacts_.size(); }

template <typename T>
const ContactInfo<T>& ContactResults<T>::get_contact_info(size_t i) const {
  DRAKE_ASSERT(i < contacts_.size());
  return contacts_[i];
}

template <typename T>
ContactResults<T>::ContactResults() : contacts_() {}

template <typename T>
void ContactResults<T>::Clear() {
  contacts_.clear();
}

template <typename T>
void ContactResults<T>::AddContact(DrakeCollision::ElementId elementA,
                 DrakeCollision::ElementId elementB,
                 const Vector3<T> & point, const WrenchVector<T> & force) {
  auto manifold = std::make_unique<SampledContactManifold<T>>();
  auto detail = make_unique<ContactDetail<T>>(point, force);
  manifold->AddContactDetail(move(detail));
  contacts_.emplace_back(elementA, elementB, move(manifold));
}


// explicit instantiation
template class ContactResults<double>;

}  // namespace systems
}  // namespace drake
