#pragma once

/// @file
/// This files contains simple structs used to store constraint specifications
/// defined by the user through MultibodyPlant API calls. These specifications
/// are later on used by our discrete solvers to build a model.

#include <limits>
#include <vector>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/deformable_ids.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

// Struct to store coupler constraint parameters.
// Coupler constraints are modeled as a holonomic constraint of the form q₀ =
// ρ⋅q₁ + Δq, where q₀ and q₁ are the positions of two one-DOF joints, ρ the
// gear ratio and Δq a fixed offset. Per equation above, ρ has units of q₀/q₁
// and Δq has units of q₀.
struct CouplerConstraintSpec {
  // First joint with position q₀.
  JointIndex joint0_index;
  // Second joint with position q₁.
  JointIndex joint1_index;
  // Gear ratio ρ.
  double gear_ratio{1.0};
  // Offset Δq.
  double offset{0.0};
  // Id of this constraint in the plant.
  MultibodyConstraintId id;
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
// @pre body_A != body_B, d₀ > 0, k >= 0, c >= 0. @see IsValid().
struct DistanceConstraintSpec {
  // Returns `true` iff `this` specification is valid to define a distance
  // constraint. A distance constraint specification is considered to be valid
  // iff body_A != body_B, distance > 0, stiffness >= 0 and damping >= 0.
  bool IsValid() const {
    return body_A != body_B && distance > 0.0 && stiffness >= 0.0 &&
           damping >= 0.0;
  }

  BodyIndex body_A;      // Index of body A.
  Vector3<double> p_AP;  // Position of point P in body frame A.
  BodyIndex body_B;      // Index of body B.
  Vector3<double> p_BQ;  // Position of point Q in body frame B.
  double distance{0.0};  // Free length d₀.
  double stiffness{
      std::numeric_limits<double>::infinity()};  // Constraint stiffness
                                                 // k in N/m.
  double damping{0.0};       // Constraint damping c in N⋅s/m.
  MultibodyConstraintId id;  // Id of this constraint in the plant.
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
struct BallConstraintSpec {
  // Returns `true` iff `this` specification is valid to define a ball
  // constraint. A ball constraint specification is considered to be valid iff:
  //   body_A != body_B.
  bool IsValid() const { return body_A != body_B; }

  BodyIndex body_A;          // Index of body A.
  Vector3<double> p_AP;      // Position of point P in body frame A.
  BodyIndex body_B;          // Index of body B.
  Vector3<double> p_BQ;      // Position of point Q in body frame B.
  MultibodyConstraintId id;  // Id of this constraint in the plant.
};

// Struct to store the specification for a weld constraint. A weld constraint is
// modeled as a holonomic constraint:
//   X_PQ(q) = I
// P is a frame rigidly affixed to body A and Q is a frame rigidly affixed to
// body B. X_PQ(q) denotes the relative pose of frame Q in frame P as a
// function of the configuration of the model, q. Imposing this constraint
// forces P and Q to be coincident.
//
// @pre body_A != bodyB. @see IsValid().
struct WeldConstraintSpec {
  // Returns `true` iff `this` specification is valid to define a weld
  // constraint. A weld constraint specification is considered to be valid iff:
  //   body_A != body_B.
  bool IsValid() { return body_A != body_B; }

  BodyIndex body_A;                   // Index of body A.
  math::RigidTransform<double> X_AP;  // Pose of frame P in A's body frame
  BodyIndex body_B;                   // Index of body B.
  math::RigidTransform<double> X_BQ;  // Pose of frame Q in B's body frame.
  MultibodyConstraintId id;           // Id of this constraint in the plant.
};

// Struct to store the specification for a fixed constraint between vertices of
// a deformable body A and a rigid body B. Such a fixed constraint is modeled as
// zero-distance holonomic constraints:
//
//   p_PᵢQᵢ_W(q) = 0 for each i in `vertices`
//
// where Pᵢ is the i-th vertex of the deformable body under constraint and Qᵢ is
// a point rigidly affixed to the rigid body B. p_PᵢQᵢ_W denotes the relative
// position of point Qᵢ with respect to point Pᵢ, expressed in the world frame
// W, as a function of the configuration of the model q. Imposing this
// constraint forces Pᵢ and Qᵢ to be coincident for each vertex i of the
// deformable body specified to be under constraint.
// @pre each entry in `vertices` refers to a valid vertex index in deformable
// body A.
struct DeformableRigidFixedConstraintSpec {
  DeformableBodyId body_A;    // Index of the deformable body A.
  BodyIndex body_B;           // Index of the rigid body B.
  std::vector<int> vertices;  // Indices of the Pᵢ in the deformable body A.
  std::vector<Vector3<double>>
      p_BQs;                 // Positions of points Qᵢ in body frame B.
  MultibodyConstraintId id;  // Id of this constraint in the plant.
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
