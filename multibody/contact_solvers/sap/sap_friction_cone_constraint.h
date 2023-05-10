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

/* Structure to store data needed for SapFrictionConeConstraint computations. */
template <typename T>
struct SapFrictionConeConstraintData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapFrictionConeConstraintData);

  /* Constructs data for a SapFrictionConeConstraint.
     @param mu Friction coefficient.
     @param Rt Regularization parameter for the tangential direction.
     @param Rn Regularization parameter for the normal direction.
     @param vn_hat Bias term. */
  SapFrictionConeConstraintData(const T& mu, const T& Rt, const T& Rn,
                                const T& vn_hat);

  /* Getters for regularization R, R⁻¹,  sqrt(R) and sqrt(R)⁻¹. */
  const Vector3<T>& R() const { return parameters_.R; }
  const Vector3<T>& R_inv() const { return parameters_.R_inv; }
  const Vector3<T>& Rsqrt() const { return parameters_.Rsqrt; }
  const Vector3<T>& Rsqrt_inv() const { return parameters_.Rsqrt_inv; }

  /* Returns constraint bias v̂. */
  const Vector3<T>& v_hat() const { return parameters_.v_hat; }

  const T& mu() const { return parameters_.mu; }

  /* Returns mu_tilde = mu ⋅ sqrt(Rt / Rn). */
  const T& mu_tilde() const { return parameters_.mu_tilde; }

  /* Returns mu_hat = mu ⋅ Rt / Rn. */
  const T& mu_hat() const { return parameters_.mu_hat; }

  /* Const access. */
  const Vector3<T>& vc() const { return vc_; }
  const int& region() const { return region_; }
  const Vector3<T>& y() const { return y_; }
  const T& yr() const { return yr_; }
  const T& yn() const { return yn_; }
  const Vector2<T>& t_hat() const { return t_hat_; }
  const Vector3<T>& gamma() const { return gamma_; }
  const Matrix3<T>& dPdy() const { return dPdy_; }

  /* Mutable access. */
  Vector3<T>& mutable_vc() { return vc_; }
  int& mutable_region() { return region_; }
  Vector3<T>& mutable_y() { return y_; }
  T& mutable_yr() { return yr_; }
  T& mutable_yn() { return yn_; }
  Vector2<T>& mutable_t_hat() { return t_hat_; }
  Vector3<T>& mutable_gamma() { return gamma_; }
  Matrix3<T>& mutable_dPdy() { return dPdy_; }

 private:
  // Values stored in this struct remain const after construction.
  struct ConstParameters {
    Vector3<T> R;  // Regularization R.
    Vector3<T> R_inv;
    Vector3<T> v_hat;      // Constraint velocity bias.
    Vector3<T> Rsqrt;      // = R^{1/2}.
    Vector3<T> Rsqrt_inv;  // = R^{-1/2}.
    T mu;
    T mu_tilde;
    T mu_hat;
  };
  ConstParameters parameters_;

  Vector3<T> vc_;  // Contact velocity.

  // Indicates in what region the un-projected impulse y is:
  //  1. Stiction: yr < mu * yn
  //  2. Sliding: -mu_hat * yr < yn && yn < yn < yr/mu
  //  3. No contact: yn < -mu * yr
  int region_{-1};

  Vector3<T> y_;         // Un-projected impulse y = −R⁻¹⋅(vc−v̂)
  T yr_{NAN}, yn_{NAN};  // Radial and normal components of y, respectively.
  Vector2<T> t_hat_;     // Tangent vector.
  Vector3<T> gamma_;     // Impulse.
  Matrix3<T> dPdy_;      // Gradient of the projection γ = P(y) w.r.t. y.
};

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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapFrictionConeConstraint);

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

  /* Constructs a contact constraint for the case in which only a single clique
   is involved. E.g. contact with the world or self-contact.
   @param[in] clique The clique involved in the contact. Must be non-negative.
   @param[in] J The Jacobian, such that vc = J⋅v. It must have three rows or an
   exception is thrown.
   @param[in] phi0 The value of the signed distance at the previous time step.
   @param[in] parameters Constraint parameters. See Parameters for details. */
  SapFrictionConeConstraint(int clique, MatrixBlock<T> J, const T& phi0,
                            const Parameters& parameters);

  /* Alternative constructor for a contact constraint involving a single clique
   that takes a dense Jacobian. */
  SapFrictionConeConstraint(int clique, MatrixX<T> J, const T& phi0,
                            const Parameters& parameters)
      : SapFrictionConeConstraint(clique, MatrixBlock<T>(std::move(J)), phi0,
                                  parameters) {}

  /* Constructs a contact constraint for the case in which two cliques
   are involved.
   @param[in] clique0 First clique involved in the contact. Must be
   non-negative.
   @param[in] clique1 Second clique involved in the contact. Must be
   non-negative.
   @param[in] J0 The Jacobian w.r.t. to the first clique's generalized
   velocities. It must have three rows or an exception is thrown.
   @param[in] J1 The Jacobian w.r.t. to the second clique's generalized
   velocities. It must have three rows or an exception is thrown.
   @param[in] phi0 The value of the signed distance at the previous time step.
   @param[in] parameters Constraint parameters. See Parameters for details. */
  SapFrictionConeConstraint(int clique0, int clique1, MatrixBlock<T> J0,
                            MatrixBlock<T> J1, const T& phi0,
                            const Parameters& parameters);

  /* Alternative constructor for a contact constraint involving two cliques
   that takes a dense Jacobian. */
  SapFrictionConeConstraint(int clique0, int clique1, MatrixX<T> J0,
                            MatrixX<T> J1, const T& phi0,
                            const Parameters& parameters)
      : SapFrictionConeConstraint(
            clique0, clique1, MatrixBlock<T>(std::move(J0)),
            MatrixBlock<T>(std::move(J1)), phi0, parameters) {}

  /* Returns the coefficient of friction for this constraint. */
  const T& mu() const { return parameters_.mu; }

  const Parameters& parameters() const { return parameters_; }

  std::unique_ptr<SapConstraint<T>> Clone() const final {
    return std::make_unique<SapFrictionConeConstraint<T>>(*this);
  }

 private:
  /* Implementations to SapConstraint NVI functions. */
  std::unique_ptr<AbstractValue> DoMakeData(
      const T& time_step,
      const Eigen::Ref<const VectorX<T>>& delassus_estimation) const override;
  void DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                  AbstractValue* data) const override;
  T DoCalcCost(const AbstractValue& data) const final;
  void DoCalcImpulse(const AbstractValue& abstract_data,
                     EigenPtr<VectorX<T>> gamma) const final;
  void DoCalcCostHessian(const AbstractValue& abstract_data,
                         MatrixX<T>* G) const final;

  /* Computes the "soft norm" ‖x‖ₛ defined by ‖x‖ₛ² = ‖x‖² + ε², where ε =
   soft_tolerance. Using the soft norm we define the tangent vector as t̂ =
   γₜ/‖γₜ‖ₛ, which is well defined event for γₜ = 0. Also gradients are well
   defined and follow the same equations presented in [Castro et al., 2021]
   where regular norms are simply replaced by soft norms. */
  static T SoftNorm(const Eigen::Ref<const VectorX<T>>& x) {
    using std::sqrt;
    // TODO(amcastro-tri): consider exposing this as a parameter.
    constexpr double soft_tolerance = 1.0e-12;
    constexpr double soft_tolerance_squared = soft_tolerance * soft_tolerance;
    return sqrt(x.squaredNorm() + soft_tolerance_squared);
  };

  /* Compute in what region the un-projected impulse y is:
      1. Stiction: yr < mu * yn
      2. Sliding: -mu_hat * yr < yn && yn < yn < yr/mu
      3. No contact: yn < -mu * yr */
  static int CalcRegion(const T& mu, const T& mu_hat, const T& yr, const T& yn);

  /* Projects y into the friction cone (defined by mu) in the R norm. */
  void Project(const SapFrictionConeConstraintData<T>& data, Vector3<T>* gamma,
               Matrix3<T>* dPdy = nullptr) const;

  Parameters parameters_;
  T phi0_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
