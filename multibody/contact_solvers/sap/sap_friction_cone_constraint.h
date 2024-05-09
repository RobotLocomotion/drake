#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/contact_configuration.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint_jacobian.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* The contact constraint mode given in terms of the radial component γᵣ and
the normal component γₙ of the contact impulse γ. */
enum class ContactMode {
  /* Contact constraint is in stiction, i.e. γᵣ < μγₙ. */
  kStiction,
  /* Contact constraint is in sliding, i.e. γᵣ = μγₙ. */
  kSliding,
  /* Contact constraint is out of contact, i.e. γ = 0. */
  kNoContact
};

/* Structure to store data needed for SapFrictionConeConstraint computations. */
template <typename T>
struct SapFrictionConeConstraintData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapFrictionConeConstraintData);

  /* Constructs data for a SapFrictionConeConstraint.
     @param mu Friction coefficient.
     @param Rt Regularization parameter for the tangential direction.
     @param Rn Regularization parameter for the normal direction.
     @param vn_hat Bias term for the normal direction.

     @warning Data stored is left uninitialized to avoid the cost of an
     unnecessary initialization. */
  SapFrictionConeConstraintData(const T& mu, const T& Rt, const T& Rn,
                                const T& vn_hat);

  /* Getters for regularization R, R⁻¹,  sqrt(R) and sqrt(R)⁻¹. */
  const Vector3<T>& R() const { return parameters_.R; }
  const Vector3<T>& R_inv() const { return parameters_.R_inv; }
  const Vector3<T>& Rsqrt() const { return parameters_.Rsqrt; }
  const Vector3<T>& Rsqrt_inv() const { return parameters_.Rsqrt_inv; }
  const T& Rt() const { return parameters_.R(0); }
  const T& Rn() const { return parameters_.R(2); }

  /* Returns constraint bias v̂. */
  const Vector3<T>& v_hat() const { return parameters_.v_hat; }

  const T& mu() const { return parameters_.mu; }

  /* Returns mu_tilde = mu ⋅ sqrt(Rt / Rn). */
  const T& mu_tilde() const { return parameters_.mu_tilde; }

  /* Returns mu_hat = mu ⋅ Rt / Rn. */
  const T& mu_hat() const { return parameters_.mu_hat; }

  /* Const access. */
  const Vector3<T>& vc() const { return vc_; }
  const ContactMode& mode() const { return mode_; }
  const Vector3<T>& y() const { return y_; }
  const T& yr() const { return yr_; }
  const T& yn() const { return yn_; }
  const Vector2<T>& t_hat() const { return t_hat_; }
  const Vector3<T>& gamma() const { return gamma_; }

  /* Mutable access. */
  Vector3<T>& mutable_vc() { return vc_; }
  ContactMode& mutable_mode() { return mode_; }
  Vector3<T>& mutable_y() { return y_; }
  T& mutable_yr() { return yr_; }
  T& mutable_yn() { return yn_; }
  Vector2<T>& mutable_t_hat() { return t_hat_; }
  Vector3<T>& mutable_gamma() { return gamma_; }

 private:
  // Values stored in this struct remain const after construction.
  struct ConstParameters {
    Vector3<T> R;          // Regularization R.
    Vector3<T> R_inv;      // Inverse of regularization R⁻¹.
    Vector3<T> v_hat;      // Constraint velocity bias.
    Vector3<T> Rsqrt;      // = R^{1/2}.
    Vector3<T> Rsqrt_inv;  // = R^{-1/2}.
    T mu;
    T mu_tilde;
    T mu_hat;
  };
  ConstParameters parameters_;

  ContactMode mode_{};  // The contact mode.
  Vector3<T> vc_;       // Contact velocity.
  Vector3<T> y_;        // Un-projected impulse y = −R⁻¹⋅(vc−v̂)
  T yr_{}, yn_{};       // Radial and normal components of y, respectively.
  Vector2<T> t_hat_;    // Tangent vector.
  Vector3<T> gamma_;    // Impulse.
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
    bool operator==(const Parameters&) const = default;

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

  /* Constructor for a SAP contact constraint between two objects A and B. The
   contact occurs at point C, with positions relative to A and B specified in
   `configuration`.

   @param[in] configuration Specifies the position of the contact point C
   relative to objects A and B in contact. Objects A and B are assigned indexes
   0 and 1, respectively. See SapContraint::object().
   @param[in] J The contact Jacobian that defines the relative velocity at point
   C as v_AcBc_W = J⋅v, with v the generalized velocities.
   @param[in] parameters Constraint parameters. See Parameters for details.
   @pre J has three rows. */
  SapFrictionConeConstraint(ContactConfiguration<T> configuration,
                            SapConstraintJacobian<T> J, Parameters parameters);

  /* Returns the coefficient of friction for this constraint. */
  const T& mu() const { return parameters_.mu; }

  /* Parameters provided at construction. */
  const Parameters& parameters() const { return parameters_; }

  /* Configuration provided at construction. */
  const ContactConfiguration<T>& configuration() const {
    return configuration_;
  }

 private:
  /* Private copy construction is enabled to use in the implementation of
    DoClone(). */
  SapFrictionConeConstraint(const SapFrictionConeConstraint&) = default;

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
  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<SapFrictionConeConstraint<T>>(
        new SapFrictionConeConstraint<T>(*this));
  }
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final;

  // no-op for this constraint.
  void DoAccumulateGeneralizedImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                       EigenPtr<VectorX<T>>) const final {}

  /* Accumulates generalized forces applied by this constraint on the i-th
   object.
   @param[in] i Object index. As defined at construction i = 0 corresponds to
   object A and i = 1 corresponds to object B.
   @param[in] gamma Impulses for this constraint, of size
   num_constraint_equations().
   @param[out] F On output, the total spatial impulse applied by this constraint
   on the i-th object.
   @pre 0 ≤ i < 2.
   @pre F is not nullptr.
  */
  void DoAccumulateSpatialImpulses(int i,
                                   const Eigen::Ref<const VectorX<T>>& gamma,
                                   SpatialForce<T>* F) const final;

  /* Computes the "soft norm" ‖x‖ₛ defined by ‖x‖ₛ² = ‖x‖² + ε², where ε =
   soft_tolerance. Using the soft norm we define the tangent vector as t̂ =
   γₜ/‖γₜ‖ₛ, which is well defined event for γₜ = 0. Also gradients are well
   defined and follow the same equations presented in [Castro et al., 2021]
   where regular norms are simply replaced by soft norms. */
  static T SoftNorm(const Eigen::Ref<const VectorX<T>>& x) {
    using std::sqrt;
    // TODO(amcastro-tri): consider exposing this as a parameter.
    constexpr double soft_tolerance = 1.0e-7;
    constexpr double soft_tolerance_squared = soft_tolerance * soft_tolerance;
    return sqrt(x.squaredNorm() + soft_tolerance_squared);
  }

  /* Compute in what region the un-projected impulse y is:
      1. Stiction: yr < mu * yn,
      2. Sliding: -mu_hat * yr < yn && yn < yn < yr/mu,
      3. No contact: yn < -mu * yr.
  */
  static ContactMode CalcContactMode(const T& mu, const T& mu_hat, const T& yr,
                                     const T& yn);

  /* Performs the SAP projection γ = P(y), where the un-projected impulse y has
    already been computed into `data`.
    @pre y, yr, yn, t_hat and region are already computed into `data`. */
  void ProjectImpulse(const SapFrictionConeConstraintData<T>& data,
                      Vector3<T>* gamma) const;

  Parameters parameters_;
  ContactConfiguration<T> configuration_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
