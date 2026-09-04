#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Structure to store data needed for SapJointFrictionConstraint computations.
 @tparam_nonsymbolic_scalar */
template <typename T>
class SapJointFrictionConstraintData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapJointFrictionConstraintData);

  /* Constructs data for a SapJointFrictionConstraint.
   Refer to SapJointFrictionConstraint's documentation for further details.
   @param R Regularization parameter. It must be strictly positive.
   @param gamma_max Friction impulse bound γₘₐₓ = δt⋅τ_c. */
  SapJointFrictionConstraintData(const T& R, const T& gamma_max)
      : parameters_{R, 1.0 / R, gamma_max} {}

  /* Regularization R. */
  const T& R() const { return parameters_.R; }

  /* Inverse of the regularization, R⁻¹. */
  const T& R_inv() const { return parameters_.R_inv; }

  /* Friction impulse bound γₘₐₓ. */
  const T& gamma_max() const { return parameters_.gamma_max; }

  /* Const access. */
  const T& vc() const { return vc_; }
  const T& y() const { return y_; }
  const T& gamma() const { return gamma_; }

  /* Mutable access. */
  T& mutable_vc() { return vc_; }
  T& mutable_y() { return y_; }
  T& mutable_gamma() { return gamma_; }

 private:
  // Values stored in this struct remain const after construction.
  struct ConstParameters {
    T R;          // Regularization R.
    T R_inv;      // Inverse of the regularization R⁻¹.
    T gamma_max;  // Friction impulse bound γₘₐₓ = δt⋅τ_c.
  };
  ConstParameters parameters_;

  T vc_{};     // Constraint velocity, i.e. the velocity of the constrained DOF.
  T y_{};      // Un-projected impulse y = −vc/R.
  T gamma_{};  // Projected impulse γ = clamp(y, −γₘₐₓ, γₘₐₓ).
};

/* Implements a dry (Coulomb) friction constraint on the i-th degree of freedom
 (DOF) of a given clique in a SapContactProblem, for the SAP solver [Castro et
 al., 2021]. This constraint models load-independent friction within a joint,
 such as gearbox friction, in the same spirit as MuJoCo's `frictionloss`
 attribute. The friction generalized force τ opposes the joint velocity and its
 magnitude is bounded by a constant τ_c ≥ 0 with units of generalized force
 (N⋅m for a revolute DOF or N for a prismatic DOF). In stiction the friction
 force takes whatever value in [−τ_c, τ_c] is needed to hold the DOF at rest,
 and during sliding it saturates at τ = −τ_c⋅sign(v).

 Constraint kinematics:
  We consider the i-th DOF of a clique with m DOFs. The constraint velocity is
  simply the generalized velocity of that DOF:
    vc = vᵢ = eᵢᵀ⋅v
  where eᵢ is the i-th element of the standard basis of ℝᵐ. Therefore the
  constraint Jacobian is J = eᵢᵀ and this constraint has a single equation.

 Regularized friction:
  With δt the time step of the SapContactProblem, the friction impulse is
  bounded by γₘₐₓ = δt⋅τ_c. The ideal maximum dissipation cost γₘₐₓ⋅|vc| is not
  differentiable at vc = 0 and therefore cannot be used directly by SAP. As for
  the friction in contact constraints (see SapFrictionConeConstraint), we
  regularize it. We use the convex cost:
    ℓ(vc) = vc²/(2R)                if |vc| ≤ R⋅γₘₐₓ  (stiction)
            γₘₐₓ⋅|vc| − R⋅γₘₐₓ²/2   otherwise          (sliding)
  which has a continuous first derivative, so that the impulse
    γ(vc) = −∂ℓ/∂vc = clamp(−vc/R, −γₘₐₓ, γₘₐₓ)
  is continuous, and a piecewise constant second derivative, the Hessian
    G(vc) = ∂²ℓ/∂vc² = R⁻¹  in stiction and zero in sliding.
  Notice that:
    1. The friction impulse always opposes the constraint velocity, satisfying
       the principle of maximum dissipation.
    2. It obeys |γ| ≤ γₘₐₓ, i.e. the friction force γ/δt is bounded by τ_c.

  The regularization R is parameterized in a scale independent manner as
    R = σ⋅w
  where w is the diagonal approximation of the Delassus operator for this
  constraint (an estimate of the inverse effective inertia of the constrained
  DOF) and σ is a dimensionless parameter, see Parameters::sigma. During
  stiction the DOF creeps with a residual velocity bounded by
    vₛ = R⋅γₘₐₓ = σ⋅w⋅δt⋅τ_c,
  that is, σ times the change in velocity that the friction force alone would
  produce on the DOF in a single time step. Under a sustained load τₐ with
  |τₐ| < τ_c the DOF settles to a steady creep velocity v∞ = σ⋅w⋅δt⋅τₐ, which
  is below vₛ.

 [Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An
   Unconstrained Convex Formulation of Compliant Contact. Available at
   https://arxiv.org/abs/2110.10107

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapJointFrictionConstraint final : public SapConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing.
    Protected copy construction is enabled for sub-classes to use in their
    implementation of DoClone(). */
  //@{
  SapJointFrictionConstraint& operator=(const SapJointFrictionConstraint&) =
      delete;
  SapJointFrictionConstraint(SapJointFrictionConstraint&&) = delete;
  SapJointFrictionConstraint& operator=(SapJointFrictionConstraint&&) = delete;
  //@}

  /* Numerical parameters that define the constraint. Refer to this class's
   documentation for details. */
  struct Parameters {
    bool operator==(const Parameters&) const = default;

    /* Dry friction bound τ_c on the generalized force of the constrained DOF.
     It has units of generalized force, i.e. N⋅m for a revolute DOF and N for a
     prismatic DOF. It must be strictly positive. */
    T friction{0.0};
    /* Dimensionless parameterization of the regularization of friction, R =
     σ⋅w. Refer to this class's documentation for details. It must be strictly
     positive. */
    double sigma{1.0e-3};
  };

  /* Constructs a dry friction constraint for the DOF with index `clique_dof`
   within the clique with index `clique` in a given SapContactProblem.
   @param[in] clique The clique involved in the constraint. Must be
   non-negative.
   @param[in] clique_dof DOF in `clique` to be constrained. It must be in [0,
   clique_nv).
   @param[in] clique_nv Number of generalized velocities for `clique`.
   @param[in] parameters Parameters of the constraint.
   @pre parameters.friction > 0.
   @pre parameters.sigma > 0. */
  SapJointFrictionConstraint(int clique, int clique_dof, int clique_nv,
                             Parameters parameters);

  const Parameters& parameters() const { return parameters_; }

  /* Returns the degree of freedom, DOF, that this constraint acts on. Provided
   at construction. */
  int clique_dof() const { return clique_dof_; }

 private:
  /* Private copy construction is enabled to use in the implementation of
    DoClone(). */
  SapJointFrictionConstraint(const SapJointFrictionConstraint&) = default;

  /* Computes the constraint Jacobian J = eᵢᵀ, with i = clique_dof. */
  static SapConstraintJacobian<T> CalcConstraintJacobian(int clique,
                                                         int clique_dof,
                                                         int clique_nv);

  /* Implementations of SapConstraint NVI functions. */
  std::unique_ptr<AbstractValue> DoMakeData(
      const T& time_step,
      const Eigen::Ref<const VectorX<T>>& delassus_estimation) const override;
  void DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                  AbstractValue* abstract_data) const override;
  T DoCalcCost(const AbstractValue& abstract_data) const override;
  void DoCalcImpulse(const AbstractValue& abstract_data,
                     EigenPtr<VectorX<T>> gamma) const override;
  void DoCalcCostHessian(const AbstractValue& abstract_data,
                         MatrixX<T>* G) const override;
  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<SapJointFrictionConstraint<T>>(
        new SapJointFrictionConstraint<T>(*this));
  }
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final;
  void DoAccumulateGeneralizedImpulses(
      int c, const Eigen::Ref<const VectorX<T>>& gamma,
      EigenPtr<VectorX<T>> tau) const final;
  // no-op for this constraint.
  void DoAccumulateSpatialImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                   SpatialForce<T>*) const final {};

  Parameters parameters_;
  int clique_dof_{-1};  // Initialized to an invalid value.
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
