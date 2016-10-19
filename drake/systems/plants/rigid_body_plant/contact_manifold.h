#pragma once

#include <memory>
#include <set>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/plants/rigid_body_plant/contact_detail.h"

namespace drake {
namespace systems {

// The intention is that this will be sub-classed to support contact manifolds
// defined with arbitrary data..
//
// For example: Hertzian contact.  The Force will still be applied at a single
// point, but the manifold includes the information of the size of the contact
// ellipse which can be used for debugging/visualization.
//
// Similarly, contact with soft bodies will produce a contact manifold
// consisting of a patch on the deformable mesh.  That would be another kind of
// contact manifold.
/**
 The contact manifold represents, abstractly, the domain of contact between
 two bodies and the contact wrenches generated.  The actual representation of
 that contact can vary based on the nature of the collision and contact models.

 However, all manifolds support the concept of a "net" applied wrench which is
 the accumulated affect of applying the full manifold into a single
 equivalent wrench applied at a single point represented as a ContactDetail.
 (@see ContactDetail).

 Individual /contact details/ can be examined (if they exist).

 Sub-classes must be copy-constructable; they must override the clone method as
 well as defining the copy constructor.

 @tparam T      The scalar type. It must be a valid Eigen scalar.
 */
template <typename T>
class DRAKE_EXPORT ContactManifold {
 public:
  /**
   Computes a single contact detail -- wrench and application point -- which
   is equivalent to applying all individual contact wrenches, independently.

   @returns the single net contact response.
   */
  virtual ContactDetail<T> ComputeNetResponse() const = 0;

  /** Reports the number of distinct contact details for this manifold. */
  virtual size_t get_num_contacts() const = 0;

  /**
   Access the ith contact detail in this manifold.

   @param[in] i      The index of the requested contact detail.
   @returns  A pointer to the ith contact detail (or null for invalid index
             values).
   */
  virtual const ContactDetail<T>* get_ith_contact(size_t i) const = 0;

  virtual ContactManifold<T>* clone() const = 0;
};

}  // namespace systems
}  // namespace drake
