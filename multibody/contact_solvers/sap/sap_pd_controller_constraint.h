#pragma once

#include <algorithm>
#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Structure to store data needed for SapPdControllerConstraint computations.
 @tparam_nonsymbolic_scalar */
template <typename T>
class SapPdControllerConstraintData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapPdControllerConstraintData);

  /* Constructs data for a SapPdControllerConstraint.
    Refer to SapPdControllerConstraint's documentation for further details.
    @param Kp_eff Effective proportional gain.
    @param Kd_eff Effective derivative gain.
    @param time_step Discrete simulation time step.

    N.B. The "effective" proportional and derivative gains Kp_eff and Kd_eff are
    computed by SapPdControllerConstraint::DoMakeData() and might differ from
    the user provided values Kp and Kd when the solver is in the "near-rigid"
    regime. Refer to implementation notes in
    SapPdControllerConstraint::DoMakeData() for details. */
  SapPdControllerConstraintData(T Kp_eff, T Kd_eff, T time_step)
      : parameters_{std::move(Kp_eff), std::move(Kd_eff),
                    std::move(time_step)} {}

  // Const data after construction.
  const T& Kp_eff() const { return parameters_.Kp_eff; }
  const T& Kd_eff() const { return parameters_.Kd_eff; }
  const T& time_step() const { return parameters_.time_step; }

  // Const access.
  const T& v() const { return v_; }
  const T& cost() const { return cost_; }
  const T& impulse() const { return impulse_; }
  const T& hessian() const { return hessian_; }

  // Mutable access.
  T& mutable_v() { return v_; }
  T& mutable_cost() { return cost_; }
  T& mutable_impulse() { return impulse_; }
  T& mutable_hessian() { return hessian_; }

 private:
  // Values stored in this struct remain const after construction.
  struct ConstParameters {
    T Kp_eff{0};
    T Kd_eff{0};
    T time_step{0};
  };
  ConstParameters parameters_;

  T v_;        // Constraint velocity.
  T cost_;     // Cost, ℓ(v).
  T impulse_;  // Impulse, γ = −∂ℓ(v)/∂v.
  T hessian_;  // Hessian, G = −∂γ/∂v = ∂²ℓ(v)/∂v².
};

/* Implements a SAP constraint to model a PD controller with effort limits.

 For a joint with configuration q and velocity v, this constraint defines the
 constraint velocity as vc = v. This constraint implements a convex cost
 function such that it models the actuation force u from a PD controller with
 desired position qd and desired velocity vd:
   ũ = -Kp⋅(q − qd) - Kd⋅(v − vd) + u_ff
   u = clamp(ũ, e) = max(−e, min(e, ũ))
 where Kp and Kd are the proportional and derivative gains respectively and e is
 the effort limit e. u_ff is an external feed-forward force.

 That is, given the cost ℓ(v), this constraint models the impulse:
   γ = −∂ℓ(v)/∂v = δt⋅u
 with δt the fixed simulation time step.

 The resulting generalized impulse on dof q is γ, implemented by
 AccumulateGeneralizedImpulses().

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapPdControllerConstraint final : public SapConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing. */
  //@{
  SapPdControllerConstraint& operator=(const SapPdControllerConstraint&) =
      delete;
  SapPdControllerConstraint(SapPdControllerConstraint&&) = delete;
  SapPdControllerConstraint& operator=(SapPdControllerConstraint&&) = delete;
  //@}

  /* Model parameters that define the constraint. */
  class Parameters {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Parameters);

    // TODO(amcastro-tri): Consider extending support for both gains being zero.
    /* Constructs a valid set of parameters.
     @param Kp Proportional gain. It must be non-negative.
     @param Kd Derivative gain. It must be non-negative.
     @param effort_limit Effort limit. It must be strictly positive.

     @pre At least one of Kp and Kd must be strictly positive. That is, they
     cannot both be zero.
     @note Units will depend on the joint type on which this constraint is
     added. E.g. For a prismatic joint, Kp will be in N/m, Kd in N⋅s/m, and
     effort_limit in N. */
    Parameters(T Kp, T Kd, T effort_limit);

    const T& Kp() const { return Kp_; }
    const T& Kd() const { return Kd_; }
    const T& effort_limit() const { return effort_limit_; }

    bool operator==(const Parameters&) const = default;

   private:
    T Kp_;            // Proportional gain.
    T Kd_;            // Derivative gain.
    T effort_limit_;  // Effort limit.
  };

  /* Struct to store the current configuration of the the constraint, when it
  gets constructed. */
  struct Configuration {
    bool operator==(const Configuration&) const = default;

    /* Clique index. */
    int clique;
    /* PD controlled dof within the given clique, and value in [0, clique_nv).*/
    int clique_dof;
    /* Clique number of velocities. */
    int clique_nv;
    /* Joint current position. */
    T q0;
    /* Joint desired position. */
    T qd;
    /* Joint desired velocity. */
    T vd;
    /* Current state feedforward actuation value. */
    T u0;
  };

  /* Constructs a PD controller constraint given its configuration and model
   parameters. */
  SapPdControllerConstraint(Configuration configuration, Parameters parameters);

  const Configuration& configuration() const { return configuration_; }
  const Parameters& parameters() const { return parameters_; }

 private:
  /* Private copy construction is enabled to use in the implementation of
     DoClone(). */
  SapPdControllerConstraint(const SapPdControllerConstraint&) = default;

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
    return std::unique_ptr<SapPdControllerConstraint<T>>(
        new SapPdControllerConstraint<T>(*this));
  }
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final;
  void DoAccumulateGeneralizedImpulses(
      int c, const Eigen::Ref<const VectorX<T>>& gamma,
      EigenPtr<VectorX<T>> tau) const final;

  /* No-op for this constraint. */
  void DoAccumulateSpatialImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                   SpatialForce<T>*) const final {}

  /* Return x clamped in the interval [-e; e].
   In other words, Clamp(x, e) = max(min(x, e), -e).
   @pre e ≥ 0. */
  static T Clamp(const T& x, const T& e) {
    using std::clamp;  // max(min(x, e), -e);
    return clamp(x, -e, e);
  }

  /* Computes the derivative of the Clamp() function.
   @pre e ≥ 0.*/
  static T ClampDerivative(const T& x, const T& e) {
    if (-e <= x && x <= e) return 1.0;
    return 0.0;
  }

  /* Computes an antiderivative of the Clamp() function. In particular, this
   implementation chooses the antiderivative that is zero at x = 0.
   @pre e ≥ 0. */
  static T ClampAntiderivative(const T& x, const T& e) {
    if (x < -e) {
      return -e * (x + e / 2);
    } else if (-e <= x && x <= e) {
      return x * x / 2;
    } else {
      return e * (x - e / 2);
    }
  }

  /* Helper to make the constraint's Jacobian given the current configuration.*/
  static SapConstraintJacobian<T> MakeConstraintJacobian(Configuration c);

  Configuration configuration_;
  Parameters parameters_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
