#pragma once

#include <memory>
#include <set>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/plants/rigid_body_plant/contact_detail.h"

namespace drake {
namespace systems {

/**
 The base class which defines a single point of contact and its corresponding
 Force.

 The point of contact and Force are defined in the world frame.
 */
template <typename T>
class DRAKE_EXPORT ContactManifold {
 public:
  ContactManifold();

  /**
   Computes a single contact detail -- Force and application point -- which
   is equivalent to applying all individual contact forces individually.
   * @returns The single net Force.
   */
  virtual ContactDetail<T> ComputeNetResponse() const = 0;

  /**
   Reports the number of distinct contact details for this manifold.
   */
  size_t get_num_contacts() const { return contact_details_.size(); }

  /**
   Access the ith contact detail in this manifold.

   @param[in] i      The index of the requested contact detail.
   @returns  A pointer to the ith contact detail (or null for invalid index
             values.
   */
  const ContactDetail<T>* get_ith_contact(size_t i) const;

  /**
   Add a new contact detail to the manifold.
   @param[in] detail    The contact detail to add.
   */
  void AddContactDetail(std::unique_ptr<ContactDetail<T>> detail);

 private:
  std::set<std::unique_ptr<ContactDetail<T>>> contact_details_;
};

template <typename T>
ContactManifold<T>::ContactManifold() {}

template <typename T>
const ContactDetail<T>* ContactManifold<T>::get_ith_contact(size_t i) const {
  if ( i < contact_details_.size()) {
    auto itr = contact_details_.begin();
    std::advance(itr, i);
    return (*itr).get();
  }
  return nullptr;
}

template <typename T>
void ContactManifold<T>::AddContactDetail(std::unique_ptr<ContactDetail<T>> detail) {
  contact_details_.insert(move(detail));
}

extern template class DRAKE_EXPORT ContactManifold<double>;

} // namespace systems
} // namespace drake
