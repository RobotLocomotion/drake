#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace hard_constraint {

/// Structure for describing unilateral constraints on configuration variables.
/// The constraints can be in the form:<pre>
/// 0 ≤ z⋅(q̅ᵢ - qᵢᴸ)  ⊥  z⋅fᵢᶜ ≥ 0
/// </pre>
/// denoting that the iᵗʰ quasi-coordinate variable is greater than or
/// equal to its limit, qᵢᴸ (qᵢᴸ ≤ q̅ᵢ), that the generalized force variable is
/// greater than or equal to zero (fᵢᶜ ≥ 0), and that both expressions cannot be
/// non-zero (q(̅ᵢ - qᵢᴸ)⋅fᵢᶜ = 0). In English, the constraint can apply no
/// force if the configuration variable is not at its limit. The constant z
/// allows both upper and lower limits to be expressed easily (z = +1 indicates
/// a lower limit, z = -1 an upper limit). We call the equation above a
/// "simple constraint" because no transformation of configuration variables is
/// necessary. The constraints can also be of the more general form:<pre>
/// 0 ≤ z⋅g(q)  ⊥  z⋅f'ᶜ ≥ 0
/// </pre>
/// where Lᵀf'ᶜ gives the generalized force resulting from the constraint force
/// and L ≡ ∂g/∂v (i.e., the Jacobian matrix of the partial derivatives of the
/// constraint function taken with respect to the generalized velocity
/// variables; see System for formal definitions of the relationships between
/// the generalized coordinates, quasi-coordinates, and generalized velocity
/// variables.
///
/// The distinction is made between the two types in this code only for reasons
/// of computational efficiency; it should be clear notationally that the
/// simple constraint is a special case of the general constraint.
///
/// Configuration variables are currently assumed to be feasible (i.e., lying
/// within their limits), because this structure does not currently
/// store the distance from limits. Consequently, limits are currently intended
/// to be processed at the exact time that z⋅g(q) = 0.
template <class T>
struct ConfigurationLimitConstraint {
  enum ConstraintType {
    /// Denotes a constraint of the form 0 ≤ z⋅(q̅ᵢ - qᵢᴸ)  ⊥  z⋅fᵢᶜ ≥ 0.
    kSimpleConstraint,

    /// Denotes a constraint of the form 0 ≤ z⋅g(q)  ⊥  z⋅f'ᶜ ≥ 0.
    kGeneralConstraint
  };

  enum LimitType {
    /// Denotes a lower limit (i.e., the case z = +1).
    kLowerLimit,

    /// Denotes an upper limit (i.e., the case z = -1).
    kUpperLimit
  };

  ConstraintType constraint_type;
  LimitType limit_type;

  /// The index of the quasi-coordinate / generalized velocity to be restricted
  /// (only used for simple constraints).
  int coordinate_index{-1};

  /// The ℝⁱˣᵐ Jacobian matrix that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into time derivatives of g(q) ∈ ℝⁱ,
  /// or, in other words, ġ = Lv. Only used for general constraints.
  MatrixX<T> L;

  /// This ℝⁱ vector is the time derivative of the matrix L (defined above)
  /// times the generalized velocity (∈ ℝᵐ) of the rigid body system. Only used
  /// for general constraints.
  VectorX<T> Ldot_x_v;
};

/// Structure for describing unilateral constraints on configuration variables,
/// for the special case of constraint forces solved at the acceleration-level.
template <class T>
struct ConfigurationLimitAccelConstraint :
    public ConfigurationLimitConstraint<T> {
  /// This ℝⁱ vector is the time derivative of the matrix L (defined in
  /// ConfigurationLimitConstraint) times the generalized velocity (∈ ℝᵐ) of the
  /// rigid body system. Only used for general constraints.
  VectorX<T> Ldot_x_v;
};

}  // namespace hard_constraint
}  // namespace multibody
}  // namespace drake
