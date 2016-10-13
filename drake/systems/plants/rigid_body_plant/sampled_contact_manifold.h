#pragma once

#include <memory>
#include <vector>

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
  /** Default constructor */
  SampledContactManifold();

  /** Copy constructor */
  SampledContactManifold(const SampledContactManifold& other);

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

  ContactManifold<T>* clone() const override;

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
  std::vector<std::unique_ptr<ContactDetail<T>>> contact_details_;
};

template <typename T>
SampledContactManifold<T>::SampledContactManifold() {}

template <typename T>
SampledContactManifold<T>::SampledContactManifold(
    const SampledContactManifold<T>& other)
{
  for (const auto & detail : other.contact_details_) {
    std::unique_ptr<ContactDetail<T>> copy(detail->clone());
    contact_details_.push_back(std::move(copy));
  }
}

template <typename T>
ContactDetail<T> SampledContactManifold<T>::ComputeNetResponse() const {
  // The "net" contact is defined as follows:
  //  point = sum_i [p_i * |f_i|] / sum_i |f_i|
  //  Force = sum_i F_i + sum_i [(point - p_i) x f_i , 0, 0, 0]
  //
  //  where p_i is the ith application point.
  //  F_i is the ith spatial force (aka wrench).
  //  f_i, |f_i| are the force component (and its magnitude) of the ith spatial
  //  force, respectively.
  //  [ f, 0, 0, 0] is a zero-torque wrench built off the given force.
  //
  //  The net application point is an approximation of the center of pressure.
  WrenchVector<T> wrench;
  wrench.setZero();

  auto accumTorque = wrench.tail(3);
  Vector3<T> point = Vector3<T>::Zero(3,1);
  T scale = 0;

  for ( const auto & detail : contact_details_ ) {
    const Vector3<T>& contactPoint = detail->get_application_point();
    const WrenchVector<T>& contactWrench = detail->get_force();

    wrench += contactWrench;

    T weight = contactWrench.head(3).norm();
    scale += weight;
    point += contactPoint * weight;
  }

  // TODO(SeanCurtis-TRI): Figure out where this epislon definition belongs.
  const double kEpsilon = 1e-10;
  if (scale > kEpsilon) point /= scale;

  Vector3<T> tempForce;
  for (const auto & detail : contact_details_) {
    const Vector3<T>& contactPoint = detail->get_application_point();
    // cross product doesn't work on "head"
    const WrenchVector<T>& contactWrench = detail->get_force();
    tempForce = contactWrench.head(3);
    accumTorque += (point - contactPoint).cross(tempForce);
  }

  return ContactDetail<T>(point, wrench);
}

template <typename T>
const ContactDetail<T>* SampledContactManifold<T>::get_ith_contact(
    size_t i) const {
  if ( i < contact_details_.size()) {
    return contact_details_[i].get();
  }
  return nullptr;
}

template <typename T>
void SampledContactManifold<T>::AddContactDetail(
    std::unique_ptr<ContactDetail<T>> detail) {
  contact_details_.push_back(move(detail));
}

template <typename T>
ContactManifold<T>* SampledContactManifold<T>::clone() const {
  return new SampledContactManifold(*this);
}

extern template class DRAKE_EXPORT SampledContactManifold<double>;

}
}
