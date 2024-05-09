#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/contact_configuration.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint_jacobian.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Constraint data created by SapHuntCrossleyConstraint::MakeData().
 See SapHuntCrossleyConstraint for details. */
template <typename T>
struct SapHuntCrossleyConstraintData {
  // Unlike the rest of the data stored in this struct, this data is not a
  // function of the constraint velocity vc. Thus it remains invariant after
  // MakeData().
  struct InvariantData {
    T dt;            // Time step.
    T n0;            // Normal impulse evaluated at previous time step.
    T epsilon_soft;  // Regularization parameter, εₛ.
  };
  InvariantData invariant_data;

  Vector3<T> vc;      // Contact velocity, defined as v_AcBc_C.
  T vn{};             // Normal component of vc, i.e. vn = vc(2).
  Vector2<T> vt;      // Tangential component of vc, i.e. vt = {vc(0), vc(1)}.
  T vt_soft{};        // Soft norm of vt, see SoftNorm().
  Vector2<T> t_soft;  // (soft) tangent vector, t_soft = vt / (vt_soft + εₛ).
  T z{};              // For Similar, z = vn - mu * vt_soft. For Lagged, z = vn.
  T nz{};             // Impulse n(z) at z.
  T Nz{};             // Antiderivative of n(z) at z, N(z).
};

/* Model approximations implemented by SapHuntCrossleyConstraint. */
enum class SapHuntCrossleyApproximation {
  kSimilar = 0,
  kLagged = 1,
};

/* Implements a convex approximation of the Hunt & Crossley model with
 regularized friction. Two convex approximations are implemented: "Similar" and
 "Lagged". Refer to [Castro et al., 2023] for further details.

 Normal Compliance:
   Given a penetration x and its rate of change ẋ, this constraint implements a
   model of linear compliance with Hunt & Crossley dissipation according to:
     γₙ/δt = k⋅x₊⋅(1+d⋅ẋ)₊
   where δt is the time step used in the formulation, k the contact stiffness
   (in N/m), d the Hunt & Crossley dissipation (in s/m) and (a)₊ = max(0, a).

   This penetration is approximated in time using x = x₀ + δt⋅ẋ. That is:
     γₙ/δt = k⋅(x₀ + δt⋅ẋ)₊⋅(1+d⋅ẋ)₊

   Note:
     For hydroelastic contact the effective penetration is defined from the
   previous time step elastic force fₑ₀ as x₀ = fₑ₀/k. Since this definition is
   not well behaved for stiffness k close to zero, this constraint implements
   instead:
    γₙ/δt = (fₑ₀ + δt⋅k⋅ẋ)₊⋅(1+d⋅ẋ)₊
   which is well defined even at k = 0.

 Regularized Friction:
   Friction is regularized according to:
     γₜ = −μ⋅γₙ⋅t̂ₛ
   where μ is the coefficient of friction and t̂ₛ is a "soft" approximation of t̂
   according to t̂ₛ = vₜ/(‖vₜ‖ₛ + εₛ), with ‖vₜ‖ₛ the "soft norm" of the
   tangential velocity vector defined as:
     ‖vₜ‖ₛ = sqrt(‖vₜ‖² + εₛ²) - εₛ
   with εₛ a regularization parameter.

   For vₜ ≠ 0 This model can also be written as:
     γₜ = −μ⋅f(‖vₜ‖/εₛ)⋅γₙ⋅t̂
   with the regularizer function f(s) = s/sqrt(1+s²). Therefore the model
   obeys the principle of maximum dissipation (friction opposes slip) and
   satisfies Coulomb's law ‖γₜ‖ ≤ μ⋅γₙ.

 Convexity:
   In general, the impulse vector γ = [γₜ, γₙ] does not lead to a convex
   formulation of contact. This constraint implements two different
   approximations: "Lagged" and "Similar". Refer to [Castro et al., 2023] for
   further details.

 The contact velocity vc = [vₜ, vₙ] ∈ ℝ³ is defined by its Jacobian J:
   vc = J⋅v
 where v is the vector of generalized velocities for the cliques involved in
 the constraint. The contact frame is defined such that vₙ is positive for
 breaking contact. Therefore, ẋ = -vₙ.

 [Castro et al., 2023] Castro A., Han X., and Masterjohn J., 2023. A Theory of
   Irrotational Contact Fields. Available online at
   https://arxiv.org/abs/2312.03908
 [Castro et al., 2022]
   Castro A., Permenter F., and Han X., 2022. An unconstrained convex
   formulation of compliant contact. IEEE Transactions on Robotics, 39(2).

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapHuntCrossleyConstraint final : public SapConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing.
    Protected copy construction is enabled for sub-classes to use in their
    implementation of DoClone(). */
  //@{
  SapHuntCrossleyConstraint& operator=(const SapHuntCrossleyConstraint&) =
      delete;
  SapHuntCrossleyConstraint(SapHuntCrossleyConstraint&&) = delete;
  SapHuntCrossleyConstraint& operator=(SapHuntCrossleyConstraint&&) = delete;
  //@}

  /* Numerical parameters that define the constraint. Refer to this class's
   documentation for details. */
  struct Parameters {
    bool operator==(const Parameters& other) const = default;

    /* Convex approximation, see [Castro et al., 2023]. */
    SapHuntCrossleyApproximation model{SapHuntCrossleyApproximation::kSimilar};
    /* Coefficient of friction μ, dimensionless. It must be non-negative. */
    T friction{0.0};
    /* Contact stiffness k, in N/m. It must be non-negative. */
    T stiffness{0.0};
    /* Parameter of Hunt & Crossley dissipation d. It must be non-negative. */
    T dissipation{0.0};
    /* Stiction tolerance vₛ, in m/s. */
    double stiction_tolerance{1.0e-4};
    /* SAP friction regularization parameter. Regularization is given by εₛ =
     max(vₛ, μ⋅σ⋅wₜ⋅n₀), where wₜ is the Delassus operator approximation for
     this constraint, σ is SAP's regularization parameter and n₀ is the impulse
     computed at the previous time step. Regularization is dominated by the SAP
     term μ⋅σ⋅wₜ⋅n₀ for cases with impacts at large time steps. When there are
     no impacts this term is 𝒪(δt) and the user specified stiction tolerance
     determines regularization. This effectively softens the approximation of
     friction during sudden transients only, leading to a better conditioned
     system of equations and improved performance. See [Castro et al., 2023] for
     details. SAP's regularization is parameterized by a single dimensionless
     parameter, estimated to be σ = 10⁻³ for a tight approximation of stiction,
     [Castro et al., 2022]. */
    double sigma{1.0e-3};
  };

  /* Constructor for a H&C contact constraint between two objects A and B. The
   contact occurs at a point C, with normal n̂ (defined to point out of A into
   B), and contact frame C.

   @param[in] configuration Specifies the kinematical configuration of the
   contact, including position of the contact point C relative to object A and
   B, signed distance, and contact frame orientation. Objects A and B are
   assigned indexes 0 and 1, respectively. See SapContraint::object().
   @param[in] J The contact Jacobian that defines the relative velocity at point
   C as v_AcBc_C = J⋅v, with v the generalized velocities of the contact
   problem.
   @param[in] parameters Constraint parameters. See Parameters for details.
   @pre J has three rows.
   @note configuration.phi is not used. */
  SapHuntCrossleyConstraint(ContactConfiguration<T> configuration,
                            SapConstraintJacobian<T> J, Parameters parameters);

  const Parameters& parameters() const { return parameters_; }
  const ContactConfiguration<T>& configuration() const {
    return configuration_;
  }

 private:
  /* Private copy construction is enabled to use in the implementation of
    DoClone(). */
  SapHuntCrossleyConstraint(const SapHuntCrossleyConstraint&) = default;

  /* Helper to compute the soft norm of vector x, defined as:
       ‖x‖ₛ = sqrt(‖x‖² + ε²) - ε. */
  static T SoftNorm(const Eigen::Ref<const VectorX<T>>& x, const T& eps) {
    using std::sqrt;
    const T x2 = x.squaredNorm();
    const T soft_norm = sqrt(x2 + eps * eps) - eps;
    return soft_norm;
  }

  // Implementations of SapConstraint NVIs.
  std::unique_ptr<AbstractValue> DoMakeData(
      const T& time_step,
      const Eigen::Ref<const VectorX<T>>& delassus_estimation) const override;
  void DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                  AbstractValue* data) const override;
  T DoCalcCost(const AbstractValue& data) const override;
  void DoCalcImpulse(const AbstractValue& abstract_data,
                     EigenPtr<VectorX<T>> gamma) const override;
  void DoCalcCostHessian(const AbstractValue& abstract_data,
                         MatrixX<T>* G) const override;
  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<SapHuntCrossleyConstraint<T>>(
        new SapHuntCrossleyConstraint<T>(*this));
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
   @pre F is not nullptr. */
  void DoAccumulateSpatialImpulses(int i,
                                   const Eigen::Ref<const VectorX<T>>& gamma,
                                   SpatialForce<T>* F) const final;

  // Computes antiderivative N(vₙ; fₑ₀) such that n(vₙ; fe0) = N'(vₙ; fₑ₀).
  // @param dt The fixed time step size.
  // @param vn Normal component of the contact velocity.
  T CalcDiscreteHuntCrossleyAntiderivative(const T& dt, const T& vn) const;

  // Computes discrete impulse function n(vₙ; fₑ₀) = N'(vₙ; fₑ₀).
  // @param dt The fixed time step size.
  // @param vn Normal component of the contact velocity.
  T CalcDiscreteHuntCrossleyImpulse(const T& dt, const T& vn) const;

  // Computes derivative n'(vₙ; fₑ₀) of the discrete impulse function.
  // @param dt The fixed time step size.
  // @param vn Normal component of the contact velocity.
  T CalcDiscreteHuntCrossleyImpulseGradient(const T& dt, const T& vn) const;

  Parameters parameters_;
  ContactConfiguration<T> configuration_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
