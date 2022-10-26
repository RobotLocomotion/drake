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
// @tparam_nonsymbolic_scalar
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

template <typename T>
struct DistanceConstraintSpecs {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DistanceConstraintSpecs);

  DistanceConstraintSpecs(BodyIndex body_A_in, BodyIndex body_B_in,
                          const Vector3<T>& p_AP_in, const Vector3<T>& p_BQ_in,
                          const T& distance_in, const T& stiffness_in,
                          const T& damping_in)
      : body_A(body_A_in),
        body_B(body_B_in),
        p_AP(p_AP_in),
        p_BQ(p_BQ_in),
        distance(distance_in),
        stiffness(stiffness_in),
        damping(damping_in) {}

  template <typename U>
  DistanceConstraintSpecs(const DistanceConstraintSpecs<U>& other) {
    body_A = other.body_A;
    body_B = other.body_B;
    if constexpr (std::is_same_v<T, double>) {
      p_AP = ExtractDoubleOrThrow(other.p_AP);
      p_BQ = ExtractDoubleOrThrow(other.p_BQ);
      distance = ExtractDoubleOrThrow(other.distance);
      stiffness = ExtractDoubleOrThrow(other.stiffness);
      damping = ExtractDoubleOrThrow(other.damping);
    } else {
      p_AP = other.p_AP;
      p_BQ = other.p_BQ;
      distance = other.distance;
      stiffness = other.stiffness;
      damping = other.damping;
    }
  }

  BodyIndex body_A;
  BodyIndex body_B;
  Vector3<T> p_AP;
  Vector3<T> p_BQ;
  T distance{0.0};
  T stiffness{std::numeric_limits<double>::infinity()};
  T damping{0.0};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
