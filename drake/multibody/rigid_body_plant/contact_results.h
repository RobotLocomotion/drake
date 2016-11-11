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
 forces for each colliding pair of collision elements) produced by a
 RigidBodyPlant system.

 @tparam T      The scalar type. It must be a valid Eigen scalar.

 Instantiated templates for the following ScalarTypes are provided:
   - double
 */
template <typename T>
class ContactResults {
 public:
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

  // TODO(SeanCurtis-TRI): Explore additional interfaces for accessing collision
  // information (e.g, query by body, etc.)
 private:
  // RigidBodyPlant is the only class allowed to instantiate/update this class
  // through Clear and AddContact().
  // TODO(SeanCurtis-TRI): when ContactResults can reference entries in the
  // cache this friendship and the method UpdateFromContext() won't be needed.
  friend class RigidBodyPlant<T>;

  // Only RigidBodyPlant can construct a ContactResults.
  ContactResults();

  // Clears the set of contact information for when the old data becomes
  // invalid.
  void Clear();

  // Reports a contact between two elements and prepares a ContactInfo. The
  // caller should populate the ContactInfo with the appropriate details.
  ContactInfo<T>& AddContact(DrakeCollision::ElementId element_a,
                             DrakeCollision::ElementId element_b);

  std::vector<ContactInfo<T>> contacts_;
};

}  // namespace systems
}  // namespace drake
