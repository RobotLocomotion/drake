#pragma once

#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/discrete_contact_pair.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// Struct used to store the contact configuration between two objects A and B
// that the contact constraints of SAP (SapHuntCrossleyConstraint and
// SapFrictionConeConstraint) require. The configuration is given by a contact
// point C, its position relative to objects A and B, and the contact frame,
// also denoted with C, defined by its orientation in the world frame W.
template <typename T>
struct ContactConfiguration {
  // Index to a physical object A.
  int objectA;

  // Position of contact point C relative to a point P on A, expressed in the
  // world frame W. Point P will usually be defined by convention and/or
  // convenience. Typical choices might be the object's center of mass or its
  // body frame, though other choices might exist.
  Vector3<T> p_ApC_W;

  // Index to a physical object B.
  int objectB;

  // Position of contact point C relative to a point Q on B, expressed in the
  // world frame W. Point Q will usually be defined by convention and/or
  // convenience. Typical choices might be the object's center of mass or its
  // body frame, though other choices might exist.
  Vector3<T> p_BqC_W;

  // Signed distance. Positive if bodies do not overlap and negative otherwise.
  T phi;

  // Normal velocity, defined as the rate of change of phi. Therefore vn > 0
  // implies bodies are moving away from each other.
  T vn;

  // Elastic force (does not include dissipation) value in the contact
  // configuration. For point contact this will be stiffness times phi.
  T fe;

  // Orientation of contact frame C in the world frame W.
  // Rz_WC = R_WC.col(2) corresponds to the normal from object A into object B.
  math::RotationMatrix<T> R_WC;
};

// Extracts a ContactConfiguration from the given DiscreteContactPair.
template <typename T>
ContactConfiguration<T> MakeContactConfiguration(
    const multibody::internal::DiscreteContactPair<T>& input) {
  return ContactConfiguration<T>{.objectA = input.object_A,
                                 .p_ApC_W = input.p_ApC_W,
                                 .objectB = input.object_B,
                                 .p_BqC_W = input.p_BqC_W,
                                 .phi = input.phi0,
                                 .vn = input.vn0,
                                 .fe = input.fn0,
                                 .R_WC = input.R_WC};
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
