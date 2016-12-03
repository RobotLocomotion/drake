#pragma once

#include <vector>

#include "drake/multibody/collision/element.h"
#include "drake/multibody/rigid_body_plant/contact_info.h"

namespace drake {
namespace systems {

// Forward declaration
template <typename T>
class RigidBodyPlant;

/**
 A class containg the contact results (contact points and response spatial
 forces for each colliding pair of collision elements).

 @tparam T      The scalar type. It must be a valid Eigen scalar.

 Instantiated templates for the following ScalarTypes are provided:
   - double
 */
template <typename T>
class ContactResults {
 public:
  ContactResults();

  ContactResults(const ContactResults<T>& other) = default;
  ContactResults<T>& operator=(const ContactResults<T>& other) = default;
  ContactResults(ContactResults<T>&& other) = delete;
  ContactResults<T>& operator=(ContactResults<T>&& other) = delete;

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
  ContactInfo<T>& AddContact(DrakeCollision::ElementId element_a,
                             DrakeCollision::ElementId element_b);

 private:
  std::vector<ContactInfo<T>> contacts_;
};

}  // namespace systems
}  // namespace drake
