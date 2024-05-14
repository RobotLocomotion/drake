#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// TODO(amcastro-tri): update reference [Castro et al., 2022] to the follow up
// paper on arbitrary constraints.

/* Structure to store data needed for SapHolonomicConstraint computations.
 @tparam_nonsymbolic_scalar */
template <typename T>
class SapHolonomicConstraintData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapHolonomicConstraintData);

  /* Constructs data for a SapHolonomicConstraintData.
     Refer to SapHolonomicConstraint's documentation for further details.
     @param R Regularization parameters.
     @param v_hat Bias term. */
  SapHolonomicConstraintData(VectorX<T> R, VectorX<T> v_hat) {
    const int nk = R.size();
    parameters_.R_inv = R.cwiseInverse();
    parameters_.R = std::move(R);
    parameters_.v_hat = std::move(v_hat);
    vc_.resize(nk);
    y_.resize(nk);
    gamma_.resize(nk);
    dPdy_.resize(nk, nk);
  }

  /* Regularization R. */
  const VectorX<T>& R() const { return parameters_.R; }

  /* Inverse of the regularization, R⁻¹. */
  const VectorX<T>& R_inv() const { return parameters_.R_inv; }

  /* Constraint bias. */
  const VectorX<T>& v_hat() const { return parameters_.v_hat; }

  /* Const access. */
  const VectorX<T>& vc() const { return vc_; }
  const VectorX<T>& y() const { return y_; }
  const VectorX<T>& gamma() const { return gamma_; }
  const MatrixX<T>& dPdy() const { return dPdy_; }

  /* Mutable access. */
  VectorX<T>& mutable_vc() { return vc_; }
  VectorX<T>& mutable_y() { return y_; }
  VectorX<T>& mutable_gamma() { return gamma_; }
  MatrixX<T>& mutable_dPdy() { return dPdy_; }

 private:
  struct ConstParameters {
    VectorX<T> R;      // Regularization R.
    VectorX<T> R_inv;  // Inverse of the regularization, R⁻¹.
    VectorX<T> v_hat;  // Constraint velocity bias.
  };
  ConstParameters parameters_;

  VectorX<T> vc_;     // Constraint velocity.
  VectorX<T> y_;      // Un-projected impulse y = −R⁻¹⋅(vc−v̂)
  VectorX<T> gamma_;  // Projected impulse γ = P(y).
  MatrixX<T> dPdy_;   // Gradient of the projection γ = P(y) w.r.t. y.
};

/* Implements an arbitrary holonomic constraint for the SAP formulation [Castro
 et al., 2022].

 Constraint kinematics:
  We can write an arbitrary holonomic constraint as g(q, t) = 0, with g(q, t) ∈
  ℝⁿ and n the number of constraint equations.
  This constraint can be written at the velocity level by taking the time
  derivative to obtain
    ġ(q, t) = J⋅v + b = 0
  where J is the constraint's Jacobian, v the vector of generalized velocities
  of the model and b is the bias term b = ∂g/∂t.

 Compliant impulses:
  We will need an impulse for each component in the constraint equation in g(q,
  t) = 0. Here we consider the more general case in which each impulse γᵢ (Greek
  letter gamma) is constrained to live in the (convex) set 𝒞ᵢ = [γₗᵢ, γᵤᵢ]
  where γₗᵢ and γᵤᵢ are the lower and upper bounds, respectively.

  Constraints in the SAP formulation model a compliant impulse γ according to:
    y/δt = −k⋅(g+τ⋅ġ)
    γ/δt = P(y)
  where we use the Roman character y for the "unprojected impulses",
  δt is the time step used in the formulation, k is the constraint stiffness (in
  N/m), τ is the dissipation relaxation time (in seconds) and P(y) is a
  projection into the (convex) set 𝒞ᵢ. In this case the projection can
  trivially be computed analytically as:
    P(y) = max(γₗ, min(γᵤ, y))
  independent of the compliant regularization.

  On SAP regularization and bias:
   Here we provide details on the computation of the regularization terms R
   performed by CalcDiagonalRegularization() and the velocity bias v̂ performed
   by CalcBiasTerm(). SAP approximates the constraint fuction as:
     g(v) ≈ g₀ + δt⋅ġ(v) = g₀ + δt⋅(J⋅v + b)
   With this approximation the unprojected impulses y(v) = −δt⋅k⋅(g + τ⋅ġ) can
   be written as:
     y(v) = −R⁻¹⋅(J⋅v − v̂)
   with the regularization R defined as:
     R⁻¹ = δt⋅(δt + τ)⋅k
   and the velocity bias v̂ as:
     v̂ = −g₀/(δt + τ) − b

 [Castro et al., 2022] Castro A., Permenter F. and Han X., 2021. An
   Unconstrained Convex Formulation of Compliant Contact. Available at
   https://arxiv.org/abs/2110.10107

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapHolonomicConstraint : public SapConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing.
    Protected copy construction is enabled for sub-classes to use in their
    implementation of DoClone(). */
  //@{
  SapHolonomicConstraint& operator=(const SapHolonomicConstraint&) = delete;
  SapHolonomicConstraint(SapHolonomicConstraint&&) = delete;
  SapHolonomicConstraint& operator=(SapHolonomicConstraint&&) = delete;
  //@}

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

    bool operator==(const Parameters&) const = default;

   private:
    VectorX<T> impulse_lower_limits_;
    VectorX<T> impulse_upper_limits_;
    VectorX<T> stiffnesses_;
    VectorX<T> relaxation_times_;
    double beta_{0.1};
  };

  /* Struct to store the kinematic values for the constraint function g, its
   Jacobian J and bias b. See this class's documentation for details. */
  struct Kinematics {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Kinematics);

    /* Constraint kinematics values of the constraint function g, Jacobian J and
     bias b. Refer to SapHolonomicConstraint for details.
     @throws std::exception if sizes g_in.size(), b_in.size() and J_in.rows()
     are not all equal. */
    Kinematics(VectorX<T> g_in, SapConstraintJacobian<T> J_in, VectorX<T> b_in)
        : g(std::move(g_in)), J(std::move(J_in)), b(std::move(b_in)) {
      DRAKE_THROW_UNLESS(g.size() == J.rows());
      DRAKE_THROW_UNLESS(b.size() == g.size());
    }

    VectorX<T> g;                // Constraint function g.
    SapConstraintJacobian<T> J;  // Constraint Jacobian J.
    VectorX<T> b;                // Bias term.
  };

  /* Constructor for a holonomic constraint given its kinematics and
   parameters. */
  SapHolonomicConstraint(Kinematics kinematics, Parameters parameters,
                         std::vector<int> objects);

  /* Constructs a holonomic constraint with zero bias term.
   @param[in] g The value of the constraint function.
   @param[in] J The Jacobian w.r.t. to the clique's generalized velocities.
   @param[in] parameters Constraint parameters. See Parameters for details.

   @pre clique is non-negative.
   @pre g.size() == J.rows() == parameters.num_constraint_equations(). */
  SapHolonomicConstraint(VectorX<T> g, SapConstraintJacobian<T> J,
                         Parameters parameters);

  /* Constructs a holonomic constraint with a non-zero bias b.
   @param[in] g The value of the constraint function.
   @param[in] J The Jacobian w.r.t. to the clique's generalized velocities.
   @param[in] b The bias term, such that ġ = J⋅v + b.
   @param[in] parameters Constraint parameters. See Parameters for details.

   @pre clique is non-negative.
   @pre g.size() == J.rows() == b.size() ==
   parameters.num_constraint_equations(). */
  SapHolonomicConstraint(VectorX<T> g, SapConstraintJacobian<T> J, VectorX<T> b,
                         Parameters parameters);

  const Parameters& parameters() const { return parameters_; }

  /* Returns the value of the constraint function provided at construction. */
  const VectorX<T>& constraint_function() const { return g_; }

  /* Returns the holonomic constraint bias b. */
  const VectorX<T>& bias() const { return bias_; }

 protected:
  /* Protected copy construction is enabled to use in the implementation of
      DoClone() here and for specific derived holonomic constraints. */
  SapHolonomicConstraint(const SapHolonomicConstraint&) = default;

 private:
  /* Implementations of SapConstraint NVI functions. */
  std::unique_ptr<AbstractValue> DoMakeData(
      const T& time_step,
      const Eigen::Ref<const VectorX<T>>& delassus_estimation) const final;
  void DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                  AbstractValue* abstract_data) const final;
  T DoCalcCost(const AbstractValue& abstract_data) const final;
  void DoCalcImpulse(const AbstractValue& abstract_data,
                     EigenPtr<VectorX<T>> gamma) const final;
  void DoCalcCostHessian(const AbstractValue& abstract_data,
                         MatrixX<T>* G) const final;
  std::unique_ptr<SapConstraint<T>> DoClone() const override {
    return std::unique_ptr<SapHolonomicConstraint<T>>(
        new SapHolonomicConstraint<T>(*this));
  }
  std::unique_ptr<SapConstraint<double>> DoToDouble() const override;

  VectorX<T> g_;
  VectorX<T> bias_;
  Parameters parameters_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
