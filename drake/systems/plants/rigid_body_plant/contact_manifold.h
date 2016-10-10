#pragma once

#include <memory>
#include <set>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/plants/rigid_body_plant/contact_detail.h"

namespace drake {
namespace systems {

/**
 The contact manifold represents, abstractly, the domain of contact between
 two bodies and the Forces generated.  The actual representation of that
 contact can vary based on the nature of the simulatin parameters.

 However, all manifolds support the concept of a "net" applied force which is
 the accumulated affect of the underlying contact details into a single Force
 applied at a single point (@see ContactDetail).

 Individual /contact details/ can be examined (if they exist).  The underlying
 representation will include, at least, the contact point and contact Force.
 However, sub-classes may also include additional data.
 */
template <typename T>
class DRAKE_EXPORT ContactManifold {
 public:

  /**
   Computes a single contact detail -- Force and application point -- which
   is equivalent to applying all individual contact forces individually.
   * @returns The single net Force.
   */
  virtual ContactDetail<T> ComputeNetResponse() const = 0;

  /**
   Reports the number of distinct contact details for this manifold.
   */
  virtual size_t get_num_contacts() const = 0;

  /**
   Access the ith contact detail in this manifold.

   @param[in] i      The index of the requested contact detail.
   @returns  A pointer to the ith contact detail (or null for invalid index
             values.
   */
  virtual const ContactDetail<T>* get_ith_contact(size_t i) const = 0;
};

} // namespace systems
} // namespace drake
