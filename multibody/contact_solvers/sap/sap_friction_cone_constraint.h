#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

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
  tangential contact forces obeys:
    γₜ = −vₜ/Rₜ
  where vₜ is the tangential velocity and Rₜ is the regularization parameter for
  the tangential direction.
  During sliding, the friction force obeys:
    γₜ = −μγₙt̂
  where t̂ = vₜ/‖vₜ‖.
  Notice that:
    1. The friction force always opposes the tangential velocity, satisfying the
       principle of maximum dissipation.
    2. It obeys Coulomb's law of friction, i.e. ‖γₜ‖ ≤ μγₙ.

  Regularization of friction means that during stiction there might be a
  residual non-zero slip velocity, even if small and negligible for a particular
  application. [Castro et al., 2021] estimate this slip velocity vₛ to be in the
  order of vₛ ≈ σ⋅δt⋅g, where g is the acceleration of gravity (≈9.81m/s² on
  planet Earth) and σ is a dimensionless parameter used to parameterize the
  regularization introduced by the constraint in a scale independent manner.
  Typical values reside in the range σ ∈ (10⁻⁴,10⁻²).

 [Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An
   Unconstrained Convex Formulation of Compliant Contact. Available at
   https://arxiv.org/abs/2110.10107

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapFrictionConeConstraint final : public SapConstraint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapFrictionConeConstraint);

  /* Numerical parameters that define the constraint. Refer to this class's
   documention for details. */
  struct Parameters {
    /* Coefficient of friction, dimensionless. It must be non-negative. */
    T mu{0.0};
    /* Contact stiffness, in N/m. It must be non-negative. */
    T stiffness{0.0};
    /* Dissipation time scale, in seconds. It must be non-negative. */
    T dissipation_time_scale{0.0};
    // Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the contact frequency
    // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. w
    // corresponds to a diagonal approximation of the Delassuss operator for
    // each contact. See [Castro et al., 2021. §IX.A] for details.
    double beta{1.0};
    // Dimensionless parameterization of the regularization of friction. An
    // approximation for the bound on the slip velocity is vₛ ≈ σ⋅δt⋅g.
    double sigma{1.0e-3};
  };

  // @throws if the number of rows in J is different from three.
  SapFrictionConeConstraint(int clique,
                            const MatrixX<T>& J, const T& phi0, const Parameters& p);

  // @throws if the number of rows in J0 and J1 is different from three.
  SapFrictionConeConstraint(int clique0, int clique1,
                            const MatrixX<T>& J0, const MatrixX<T>& J1,
                            const T& phi0, const Parameters& p);

  const T& mu() const { return parameters_.mu; }

  void Project(const Eigen::Ref<const VectorX<T>>& y,
               const Eigen::Ref<const VectorX<T>>& R,
               EigenPtr<VectorX<T>> gamma,
               MatrixX<T>* dPdy = nullptr) const final;

  VectorX<T> CalcBiasTerm(const T& time_step, const T& wi) const final;
  VectorX<T> CalcDiagonalRegularization(const T& time_step,
                                        const T& wi) const final;

 private:
  Parameters parameters_;
  T phi0_;
  double soft_tolerance_{1.0e-7};
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
