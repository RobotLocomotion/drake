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
  void AddContactDetail(std::unique_ptr<ContactDetail<T>>& detail);

 private:
  std::vector<std::unique_ptr<ContactDetail<T>>> contact_details_;
};

extern template class DRAKE_EXPORT SampledContactManifold<double>;

}
}
