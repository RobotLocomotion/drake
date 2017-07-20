#pragma once

#include <Eigen/Core>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace rigid_contact {

/// A tracked point of contact between two rigid bodies that use polytopical
/// geometries. The point is defined with respect to exactly one of the two
/// rigid bodies. This point would ideally be used to define constraints only if
/// the distance from it to the second rigid body were zero; floating point
/// truncation and rounding errors imply that such distances are unlikely to
/// ever actually be zero. Instead, we track the point of contact over time and
/// use ancillary information (e.g., are Lagrangle Multipliers zero?) and
/// store the contact state (sliding, not sliding) to compose necessary
/// constraint equations.
struct RigidContactContact {
  enum class ContactSlidingState {
    /// The bodies are to be considered as sliding at this point of contact.
    kSliding,

    /// The bodies are to be considered as not sliding at this point of contact.
    kNotSliding,
  };

  /// Whether the contact is to be treated as sliding or not sliding.
  ContactSlidingState sliding_state;

  /// The vector from the center-of-mass of the rigid body to the point of
  /// contact, expressed in the body frame of one of the rigid bodies.
  Eigen::Vector3d u;
};

}  // namespace rigid_contact
}  // namespace multibody
}  // namespace drake
