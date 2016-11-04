#pragma once

#include <memory>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/plants/rigid_body_plant/contact_force.h"

namespace drake {
namespace systems {

/**
  The base class for defining a contact detail.  The contact can take different
  forms (e.g., single ContactForce, a collection of ContactForce instances,
  a patch with pressure defined over the patch domain, etc.)  The details of
  the contact detail are a function of the contact response model that generates
  it.

  All ContactDetail implementations provide a common interface; they have the
  ability to produce a single equivalent resultant ContactForce to the
  underlying data.

  @tparam T      The scalar type. It must be a valid Eigen scalar.
 */
template <typename T>
class DRAKE_EXPORT ContactDetail {
 public:
  virtual ~ContactDetail() {}

  virtual std::unique_ptr<ContactDetail<T>> Clone() const;

  /**
   Computes a single equivalent contact force to the underlying contact details.
   */
  virtual ContactForce<T> ComputeContactForce() const = 0;
};

}  // namespace systems
}  // namespace drake
