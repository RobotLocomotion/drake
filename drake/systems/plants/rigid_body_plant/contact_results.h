#pragma once

#include <vector>

#include "drake/common/drake_export.h"
#include "drake/systems/plants/collision/Element.h"
#include "drake/systems/plants/rigid_body_plant/contact_info.h"

namespace drake {
namespace systems {

// Forward declaration
template <typename T> class RigidBodyPlant;

/**
 A class containg the contact results (contact points and response Forces) for a
 RigidBodyPlant system.
 @tparam T      The scalar type[  It must be a valid Eigen scalar;
 */
template <typename T>
class DRAKE_EXPORT ContactResults {
 public:
  /** Returns the number of contacts between unique collision element pairs. */
  size_t get_num_contacts() const;

  /** Returns the ContactInfor for the ith collision element pair. */
  const ContactInfo<T>& get_contact_info(size_t i) const;

  // TODO(SeanCurtis-TRI): Explore additional interfaces for accessing collision
  // information (e.g, query by body, etc.)
 private:
  // RigidBodyPlant is the only class allowed to instantiate/update this class
  // through Clear and AddContact().
  // TODO(SeanCurtis-TRI): when ContactResults can reference entries in the
  // cache this friendship and the method UpdateFromContext() won't be needed.
  friend class RigidBodyPlant<T>;

  // Only RigidBodyPlant can construct a ContactResults from the underlying
  // RigidBodyTree.
  explicit ContactResults();

  /** Clears the set of contact information for when the old data becomes
   invalid */
  void Clear();

  /**
   Adds a contact between two collision elements into the contact set.
   @param elementA      The first element in the collision.
   @param elementB      The second element in the collision.
   @param point         The point (expressed in the world frame) at which the
                        force is applied.
   @param force         The contact Force applied to A.  Its negative would be
                        applied to B.
   */
  void AddContact( DrakeCollision::ElementId elementA,
                   DrakeCollision::ElementId elementB,
                   const Vector3<T> & point, const WrenchVector<T> & force);

  std::vector<ContactInfo<T>> contacts_;
};

extern template class DRAKE_EXPORT ContactResults<double>;
}  // namespace systems
}  // namespace drake