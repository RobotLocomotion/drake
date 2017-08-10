#pragma once

#include <Eigen/Core>

#include "drake/systems/framework/value.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace rigid_contact {

/// A tracked point of contact between two rigid bodies. This point, p, would
/// ideally be used to define constraints at time t if and only if the distance
/// between the rigid bodies at p(t) were zero; floating point truncation and
/// rounding errors mean that such distances are unlikely to ever actually be
/// zero. Incorporating tracked points of contact (as well as the sliding
/// state- sliding or not sliding) into a System's abstract state and using
/// ancillary information (e.g., zero Lagrange Multipliers) allows avoiding
/// these floating point issues.
template <class T>
struct RigidContactPoint {
  enum class ContactSlidingState {
    /// The bodies are to be considered as sliding at this point of contact.
    kSliding,

    /// The bodies are to be considered as not sliding at this point of contact.
    kNotSliding,
  };

  /// Whether the contact is to be treated as sliding or not sliding.
  ContactSlidingState sliding_state;

  /// Abstract data sufficient (in combination with other continuous and/or
  /// discrete state variables) to uniquely determine a point of contact between
  /// two rigid bodies.
  systems::Value<T> contact_point_key;
};

}  // namespace rigid_contact
}  // namespace multibody
}  // namespace drake
