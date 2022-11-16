#pragma once

/// @file
/// This files contains simple structs used to store constraint specifications
/// defined by the user through MultibodyPlant API calls. These specifications
/// are later on used by our discrete solvers to build a model.

#include <limits>

#include "drake/common/default_scalars.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

// Struct to store coupler constraint parameters.
// Coupler constraints are modeled as a holonomic constraint of the form q₀ =
// ρ⋅q₁ + Δq, where q₀ and q₁ are the positions of two one-DOF joints, ρ the
// gear ratio and Δq a fixed offset. Per equation above, ρ has units of q₀/q₁
// and Δq has units of q₀.
// @tparam_default_scalar
template <typename T>
struct CouplerConstraintSpecs {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CouplerConstraintSpecs);

  CouplerConstraintSpecs(JointIndex j0, JointIndex j1, const T& gear_ratio_in,
                         const T& offset_in)
      : joint0_index(j0),
        joint1_index(j1),
        gear_ratio(gear_ratio_in),
        offset(offset_in) {}

  template <typename U>
  CouplerConstraintSpecs(const CouplerConstraintSpecs<U>& other) {
    joint0_index = other.joint0_index;
    joint1_index = other.joint1_index;
    if constexpr (std::is_same_v<T, double>) {
      gear_ratio = ExtractDoubleOrThrow(other.gear_ratio);
      offset = ExtractDoubleOrThrow(other.offset);
    } else {
      gear_ratio = other.gear_ratio;
      offset = other.offset;
    }
  }

  // First joint with position q₀.
  JointIndex joint0_index;
  // Second joint with position q₁.
  JointIndex joint1_index;
  // Gear ratio ρ.
  T gear_ratio{1.0};
  // Offset Δq.
  T offset{0.0};
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
// @note To constrain two points to be coincident we need a 3-dof ball
// constraint, the 1-dof distance constraint is singular in this case.
// Therefore we require the distance parameter to be strictly positive.
//
// @pre d₀ > 0, k >= 0, c >= 0. @see AreParametersValid().
struct DistanceConstraintSpecs {
  // Returns `true` iff the given parameters are valid to define the
  // specifications for a distance constraint .
  static bool AreParametersValid(double distance, double stiffness,
                                 double damping) {
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

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::internal::CouplerConstraintSpecs);
