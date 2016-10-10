#pragma once

#include <memory>
#include <set>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/plants/rigid_body_plant/contact_detail.h"
#include "drake/systems/plants/rigid_body_plant/contact_manifold.h"

namespace drake {
namespace systems {

/**
 This manifold represents a contact surface with a limited number of sampled
 points.  It is, at best, a coarse approximation of the contact surface.  It
 doesn't guarantee that the points from a complete convex hull of the full
 contact area.  It merely provides one or more points which in some meaningful
 sense attempt to sample the contact area.
 */
template <typename T>
class DRAKE_EXPORT SampledContactManifold : public ContactManifold<T> {
 public:

  /**
   Computes a single contact detail -- Force and application point -- which
   is equivalent to applying all individual contact forces individually.
   * @returns The single net Force.
   */
  ContactDetail<T> ComputeNetResponse() const override;

  /**
   Reports the number of distinct contact details for this manifold.
   */
  size_t get_num_contacts() const override { return contact_details_.size(); }

  /**
   Access the ith contact detail in this manifold.

   @param[in] i      The index of the requested contact detail.
   @returns  A pointer to the ith contact detail (or null for invalid index
             values.
   */
  const ContactDetail<T>* get_ith_contact(size_t i) const override;

  /**
   Add a new contact detail to the manifold.
   @param[in] detail    The contact detail to add.
   */
  // TODO(SeanCurtis-TRI): Determine if this is the right interface for this.
  // would it not be better to make sure that this ia a purely read-only
  // interface?  Although, making sure only const versions of the manifold are
  // available to the end user would alleviate this issue.
  void AddContactDetail(std::unique_ptr<ContactDetail<T>> detail);

 private:
  std::set<std::unique_ptr<ContactDetail<T>>> contact_details_;
};

template <typename T>
ContactDetail<T> SampledContactManifold<T>::ComputeNetResponse() const {
  Vector3<T> force = Vector3<T>::Zero(3,1);
  Vector3<T> accumTorque = Vector3<T>::Zero(3,1);
  Vector3<T> point = Vector3<T>::Zero(3,1);
  T scale = 0;

  for ( const auto & detail : contact_details_ ) {
    const Vector3<T>& contactPoint = detail->get_application_point();
    const WrenchVector<T>& contactWrench = detail->get_force();
    Vector3<T> contactForce;
    contactForce << contactWrench(0), contactWrench(1), contactWrench(2);
    Vector3<T> contactTorque;
    contactTorque << contactWrench(3), contactWrench(4), contactWrench(5);

    force += contactForce;
    accumTorque += contactTorque;

    T weight = contactForce.norm();
    scale += weight;
    point += contactPoint * weight;
  }

  point /= scale;

  for ( const auto & detail : contact_details_ ) {
    const Vector3<T>& contactPoint = detail->get_application_point();
    const WrenchVector<T>& contactWrench = detail->get_force();
    Vector3<T> contactForce;
    contactForce << contactWrench(0), contactWrench(1), contactWrench(2);
    accumTorque += (contactPoint - point).cross(contactForce);
  }

  WrenchVector<T> wrench;
  wrench << force(0), force(1), force(2),
            accumTorque(0), accumTorque(1), accumTorque(2);
  return ContactDetail<T>(point, wrench);
}

template <typename T>
const ContactDetail<T>* SampledContactManifold<T>::get_ith_contact(
    size_t i) const {
  if ( i < contact_details_.size()) {
    auto itr = contact_details_.begin();
    std::advance(itr, i);
    return (*itr).get();
  }
  return nullptr;
}

template <typename T>
void SampledContactManifold<T>::AddContactDetail(
    std::unique_ptr<ContactDetail<T>> detail) {
  contact_details_.insert(move(detail));
}

extern template class DRAKE_EXPORT SampledContactManifold<double>;

}
}
