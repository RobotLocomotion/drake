#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/rigid_body_plant/contact_info.h"

namespace drake {
namespace systems {

// Forward declaration
template <typename T>
class RigidBodyPlant;

/**
 A class containing the contact results (contact points and response spatial
 forces for each colliding pair of collision elements) as well as the sum of
 all JᵀF for all contact, where J is the contact point Jacobian, and F is
 the contact force.

 @tparam T      The scalar type. It must be a valid Eigen scalar.

 Instantiated templates for the following ScalarTypes are provided:

 - double
 - AutoDiffXd
 */
template <typename T>
class ContactResults {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactResults)

  ContactResults();

  /** Returns the number of unique collision element pairs in contact. */
  int get_num_contacts() const;

  /** Retrieves the ith ContactInfo instance.  No bounds checking will be done
   in a release build (but will be done in debug).  It is assumed the caller
   will only use values in the range [0, get_num_contacts() -1], inclusive.
   */
  const ContactInfo<T>& get_contact_info(int i) const;

  // Clears the set of contact information for when the old data becomes
  // invalid.
  void Clear();

  // Reports a contact between two elements and prepares a ContactInfo. The
  // caller should populate the ContactInfo with the appropriate details.
  ContactInfo<T>& AddContact(drake::multibody::collision::ElementId element_a,
                             drake::multibody::collision::ElementId element_b);

  /**
   * Stores the contact forces as a force in the generalized coordinate.
   * @param f = J^T * contact_force, where J is the stacked contact Jacobian,
   * and contact_force is the stacked contact forces.
   */
  void set_generalized_contact_force(const VectorX<T>& f) {
    generalized_contact_force_ = f;
  }

  /**
   * Returns the stored generalized force that represents the contact forces.
   */
  const VectorX<T>& get_generalized_contact_force() const {
    return generalized_contact_force_;
  }

 private:
  std::vector<ContactInfo<T>> contacts_;
  VectorX<T> generalized_contact_force_;
};

}  // namespace systems
}  // namespace drake
