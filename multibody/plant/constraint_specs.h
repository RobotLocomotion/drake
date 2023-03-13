#pragma once

/// @file
/// This files contains simple structs used to store constraint specifications
/// defined by the user through MultibodyPlant API calls. These specifications
/// are later on used by our discrete solvers to build a model.

#include <limits>

#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

// Struct to store coupler constraint parameters.
// Coupler constraints are modeled as a holonomic constraint of the form q₀ =
// ρ⋅q₁ + Δq, where q₀ and q₁ are the positions of two one-DOF joints, ρ the
// gear ratio and Δq a fixed offset. Per equation above, ρ has units of q₀/q₁
// and Δq has units of q₀.
struct CouplerConstraintSpecs {
  // First joint with position q₀.
  JointIndex joint0_index;
  // Second joint with position q₁.
  JointIndex joint1_index;
  // Gear ratio ρ.
  double gear_ratio{1.0};
  // Offset Δq.
  double offset{0.0};
};

// Struct to store the specification for a distance constraint. A distance
// constraint is modeled as a holonomic constraint. Distance constraints can
// be "soft" which imposes the the condition:
//   (d(q)-d₀) + c/k⋅ḋ(q) + 1/k⋅f = 0
// where d₀ is a fixed length, k a stiffness parameter in N/m and c a damping
// parameter in N⋅s/m. We use d(q) to denote the Euclidean distance between two
// points P and Q, rigidly affixed to bodies A and B respectively, as a function
// of the configuration of the model q. This constraint reduces to d(q) = d₀ in
// the limit to infinite stiffness and it behaves as a linear spring damper for
// finite values of stiffness and damping.
//
// @warning A distance constraint is the wrong modeling choice if the
// distance needs to go through zero. To constrain two points to be
// coincident we need a 3-dof ball constraint, the 1-dof distance constraint
// is singular in this case. Therefore we require the distance parameter to
// be strictly positive.
//
// @pre d₀ > 0, k >= 0, c >= 0. @see IsValid().
struct DistanceConstraintSpecs {
  // Returns `true` iff `this` specification is valid to define a distance
  // constraint. A distance constraint specification is considered to be valid
  // iff distance > 0, stiffness >= 0 and damping >= 0.
  bool IsValid() {
    return distance > 0.0 && stiffness >= 0.0 && damping >= 0.0;
  }

  BodyIndex body_A;      // Index of body A.
  Vector3<double> p_AP;  // Position of point P in body frame A.
  BodyIndex body_B;      // Index of body B.
  Vector3<double> p_BQ;  // Position of point Q in body frame B.
  double distance{0.0};  // Free length d₀.
  double stiffness{
      std::numeric_limits<double>::infinity()};  // Constraint stiffness
                                                 // k in N/m.
  double damping{0.0};  // Constraint damping c in N⋅s/m.
};

// Struct to store the specification for a ball constraint. A ball
// constraint is modeled as a holonomic constraint:
//   p_PQ_W(q) = 0
// P is a point rigidly affixed to body A and Q is a point rigidly affixed to
// body B. p_PQ_W(q) denotes the relative position of point Q with respect to
// point P, expressed in the world frame, as a function of the configuration of
// the model q. Imposing this constraint forces P and Q to be coincident, but
// does not restrict the rotational degrees of freedom.
//
// @pre body_A != body_B. @see IsValid().
struct BallConstraintSpecs {
  // Returns `true` iff `this` specification is valid to define a ball
  // constraint. A ball constraint specification is considered to be valid iff:
  //   body_A != body_B.
  bool IsValid() { return body_A != body_B; }

  BodyIndex body_A;      // Index of body A.
  Vector3<double> p_AP;  // Position of point P in body frame A.
  BodyIndex body_B;      // Index of body B.
  Vector3<double> p_BQ;  // Position of point Q in body frame B.
};

struct PdControllerConstraintSpecs {
  // Joint on which the PD controller is added.
  JointActuatorIndex actuator_index;
  // Proportional gain, with units consistent to the type of joint (i.e. N/m for
  // prismatic and Nm/rad for revolute).
  double proportional_gain{NAN};
  // Derivative gain, with units consistent to the type of joint (i.e. Ns/m for
  // prismatic and Nms/rad for revolute)
  double derivative_gain{0.0};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
