#pragma once

#include <limits>
#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Implements an arbitrary holonomic constraint for the SAP formulation [Castro
 et al., 2022].

 Constraint kinematics:
  We can write an arbitrary holonomic constraint as g(q, t) = 0, with g(q, t) ∈
  ℝⁿ and n the number of constraint equations.
  This constraint can be written at the velocity level by taking the time
  derivative to obtain
    ġ(q, t) = J⋅v + b = 0
  where J is the contraint's Jacobian, v the vector of generalized velocities of
  the model and b is the bias term b = ∂g/∂t.

 Compliant impulses:
  We will need an impulse for each component in the constraint equation in g(q,
  t) = 0. Here we consider the more general case in which each impulse γᵢ is
  constrained to live in the (convex) set 𝒞ᵢ = [γₗᵢ, γᵤᵢ] where γₗᵢ and γᵤᵢ are
  the lower and upper bounds, respectively.

  Constraints in the SAP formulation model a compliant impulse γ according to:
    y/dt = −k⋅(g+τ⋅ġ)
    γ/δt = P(y)
  where δt is the time step used in the formulation, k is the constraint
  stiffness (in N/m), τ is the dissipation relaxation time (in seconds) and
  P(y) is a projection into the (convex) set 𝒞ᵢ. In this case the projection can
  trivially be computed analytically as:
    P(y) = max(γₗ, min(γᵤ, y))
  independent of the compliant regularization.

 [Castro et al., 2022] Castro A., Permenter F. and Han X., 2021. An
   Unconstrained Convex Formulation of Compliant Contact. Available at
   https://arxiv.org/abs/2110.10107

 @tparam_nonsymbolic_scalar */
// TODO(amcastro-tri): allow to instantinate holonomic constraints with a non
// zero bias term b = ∂g/∂t.
template <typename T>
class SapHolonomicConstraint final : public SapConstraint<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapHolonomicConstraint);

  /* Numerical parameters that define the constraint. Refer to this class's
   documentation for details. */
  class Parameters {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Parameters);

    /* Constructs a valid set of parameters.
     @param impulse_lower_limits vector of lower limits γₗ.
     @param impulse_upper_limits vector of upper limits γᵤ.
     @param stiffnesses vector of stiffnesses kᵢ for each constraint.
     @param relaxation_times vector of relaxation times τᵢ for each constraint.
     @param beta Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the
     constraint frequency ωᵢ for the i-th constraint is below the limit ωᵢ ≤
     2π/δt. That is, the period is limited to Tᵢ = β⋅δt. w corresponds to a
     diagonal approximation of the Delassuss operator for the constraint. See
     [Castro et al., 2022] for details.

     @pre impulse_lower_limits, impulse_upper_limits, stiffnesses and
     relaxation_times must all have the same size.
     @pre impulse_lower_limits <= +∞, componentwise.
     @pre impulse_upper_limits >= -∞, componentwise.
     @pre lower_limit <= upper_limit, componentwise.
     @pre stiffnesses > 0, componentwise.
     @pre relaxation_times >= 0, componentwise
     @pre beta > 0 */
    Parameters(VectorX<T> impulse_lower_limits, VectorX<T> impulse_upper_limits,
               VectorX<T> stiffnesses, VectorX<T> relaxation_times,
               double beta = 0.1);

    const VectorX<T>& impulse_lower_limits() const {
      return impulse_lower_limits_;
    }
    const VectorX<T>& impulse_upper_limits() const {
      return impulse_upper_limits_;
    }
    const VectorX<T>& stiffnesses() const { return stiffnesses_; }
    const VectorX<T>& relaxation_times() const { return relaxation_times_; }
    double beta() const { return beta_; }
    int num_constraint_equations() const {
      return impulse_lower_limits_.size();
    }

   private:
    VectorX<T> impulse_lower_limits_;
    VectorX<T> impulse_upper_limits_;
    VectorX<T> stiffnesses_;
    VectorX<T> relaxation_times_;
    double beta_{0.1};
  };

  /* Constructs a holonomic constraint involving a single clique.
   @param[in] clique The clique involved in the constraint.
   @param[in] g The value of the constraint function.
   @param[in] J The Jacobian, such that vc = J⋅v.
   @param[in] parameters Constraint parameters. See Parameters for details.

   @pre clique is non-negative.
   @pre g.size() == J.rows() == parameters.num_constraint_equations(). */
  SapHolonomicConstraint(int clique, VectorX<T> g, MatrixX<T> J,
                         Parameters parameters);

  /* Constructs a holonomic constraint involving two cliques.
   @param[in] first_clique First clique involved in the constraint.
   @param[in] second_clique Second clique involved in the constraint.
   @param[in] g The value of the constraint function.
   @param[in] J_first_clique The Jacobian w.r.t. to the first clique's
   generalized velocities.
   @param[in] J_second_clique The Jacobian w.r.t. to the second clique's
   generalized velocities.
   @param[in] parameters Constraint parameters. See Parameters for details.

   @pre first_clique and second_clique are non-negative.
   @pre g.size() == J_first_clique.rows() == J_second_clique.rows() ==
   parameters.num_constraint_equations(). */
  SapHolonomicConstraint(int first_clique, int second_clique, VectorX<T> g,
                         MatrixX<T> J_first_clique, MatrixX<T> J_second_clique,
                         Parameters parameters);

  const Parameters& parameters() const { return parameters_; }

  /* Implements the projection operation P(y) = max(γₗ, min(γᵤ, y)), independent
   of the regularization R. Refer to SapConstraint::Project() for details. */
  void Project(const Eigen::Ref<const VectorX<T>>& y,
               const Eigen::Ref<const VectorX<T>>& R,
               EigenPtr<VectorX<T>> gamma,
               MatrixX<T>* dPdy = nullptr) const final;

  // TODO(amcastro-tri): Extend SapConstraint so that wi can be a vector with an
  // entry for each constraint equation.
  VectorX<T> CalcBiasTerm(const T& time_step, const T& wi) const final;

  VectorX<T> CalcDiagonalRegularization(const T& time_step,
                                        const T& wi) const final;

  std::unique_ptr<SapConstraint<T>> Clone() const final;

 private:
  Parameters parameters_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
