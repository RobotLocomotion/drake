#pragma once

#include <memory>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint_jacobian.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Implements contact constraints for the SAP solver.
 Here we provide a brief description of the contact forces modeled by this
 constraint, enough to introduce notation and the constraint's parameters.
 Please refer to [Castro et al., 2021] for an in-depth discussion.

 Normal Compliance:
  Contact constraints in the SAP formulation model a compliant normal impulse γₙ
  according to:
    γₙ/δt = (−k⋅ϕ−d⋅vₙ)₊
  where δt is the time step used in the formulation, k is the contact stiffness
  (in N/m), d is the dissipation (in N⋅s/m) and (x)₊ = max(0, x). ϕ and vₙ are
  the next time step values of the signed distance function and normal velocity
  respectively. ϕ is defined to be negative when bodies overlap and positive
  otherwise. vₙ is defined to be positive when bodies move away from each other
  and negative when they move towards each other. Dissipation is parameterized
  as d = tau_d⋅k, where tau_d is the "dissipation time scale".

 Regularized Friction:
  SAP contact constraints regularize friction. That is, in stiction the
  tangential contact impulses obeys:
    γₜ = −vₜ/Rₜ
  where vₜ is the tangential velocity and Rₜ is the regularization parameter for
  the tangential direction.
  During sliding, the friction impulse obeys:
    γₜ = −μγₙt̂
  where t̂ = vₜ/‖vₜ‖.
  Notice that:
    1. The friction impulse always opposes the tangential velocity, satisfying
       the principle of maximum dissipation.
    2. It obeys Coulomb's law of friction, i.e. ‖γₜ‖ ≤ μγₙ.

  Regularization of friction means that during stiction there might be a
  residual non-zero slip velocity, even if small and negligible for a particular
  application. [Castro et al., 2021] estimate this slip velocity vₛ to be in the
  order of vₛ ≈ σ⋅δt⋅g, where g is the acceleration of gravity (≈9.81m/s² on
  planet Earth) and σ is a dimensionless parameter used to parameterize the
  regularization introduced by the constraint in a scale independent manner.
  Typical values reside in the range σ ∈ (10⁻⁴,10⁻²).

 The contact velocity vc = [vₜ, vₙ] ∈ ℝ³ for this constraint is defined such
 that:
   vc = J⋅v
 where J is the constraint's Jacobian and v is the vector of generalized
 velocities for the cliques involved in the constraint.

 [Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An
   Unconstrained Convex Formulation of Compliant Contact. Available at
   https://arxiv.org/abs/2110.10107

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapFrictionConeConstraint final : public SapConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing.
    Protected copy construction is enabled for sub-classes to use in their
    implementation of DoClone(). */
  //@{
  SapFrictionConeConstraint& operator=(const SapFrictionConeConstraint&) =
      delete;
  SapFrictionConeConstraint(SapFrictionConeConstraint&&) = delete;
  SapFrictionConeConstraint& operator=(SapFrictionConeConstraint&&) = delete;
  //@}

  /* Numerical parameters that define the constraint. Refer to this class's
   documentation for details. */
  struct Parameters {
    /* Coefficient of friction μ, dimensionless. It must be non-negative. */
    T mu{0.0};
    /* Contact stiffness k, in N/m. It must be strictly positive. */
    T stiffness{0.0};
    /* Dissipation time scale tau_d, in seconds. It must be non-negative. */
    T dissipation_time_scale{0.0};
    /* Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the contact frequency
     ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. w
     corresponds to a diagonal approximation of the Delassuss operator for
     each contact. See [Castro et al., 2021] for details. */
    double beta{1.0};
    /* Dimensionless parameterization of the regularization of friction. An
     approximation for the bound on the slip velocity is vₛ ≈ σ⋅δt⋅g.
     Refer to [Castro et al., 2021] for details. */
    double sigma{1.0e-3};
  };

  /* Constructs a contact constraint given its Jacobian and signed distance at
   the current configuration.
   @param[in] J The contact Jacobian, which defines the contact velocity `vc` as
   vc = J⋅v. It must have three rows or an exception is thrown.
   @param[in] phi0 The value of the signed distance at the current
   configuration.
   @param[in] parameters Constraint parameters. See Parameters for details. */
  SapFrictionConeConstraint(SapConstraintJacobian<T> J, const T& phi0,
                            const Parameters& parameters);

  /* Returns the coefficient of friction for this constraint. */
  const T& mu() const { return parameters_.mu; }

  const Parameters& parameters() const { return parameters_; }

  /* Implements the projection operation. Refer to SapConstraint::Project() for
   details. */
  void Project(const Eigen::Ref<const VectorX<T>>& y,
               const Eigen::Ref<const VectorX<T>>& R,
               EigenPtr<VectorX<T>> gamma,
               MatrixX<T>* dPdy = nullptr) const final;

  /* Computes bias term. Refer to SapConstraint::CalcBiasTerm() for details. */
  VectorX<T> CalcBiasTerm(const T& time_step, const T& wi) const final;

  /* Computes the diagonal of the regularization matrix (positive diagonal) R.
   This computes R = [Rt, Rt, Rn] with:
     Rn = max(β²/(4π²)⋅wᵢ, (δt⋅(δt+tau_d)⋅k)⁻¹),
     Rt = σ⋅wᵢ
   Refer to [Castro et al., 2021] for details. */
  VectorX<T> CalcDiagonalRegularization(const T& time_step,
                                        const T& wi) const final;

 private:
  /* Private copy construction is enabled to use in the implementation of
   DoClone(). */
  SapFrictionConeConstraint(const SapFrictionConeConstraint&) = default;

  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<SapFrictionConeConstraint<T>>(
        new SapFrictionConeConstraint<T>(*this));
  }

  Parameters parameters_;
  T phi0_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
