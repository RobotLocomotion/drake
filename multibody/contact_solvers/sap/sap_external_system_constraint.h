#pragma once

#include <memory>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using multibody::SpatialForce;
using multibody::contact_solvers::internal::SapConstraint;
using multibody::contact_solvers::internal::SapConstraintJacobian;

/* Structure to store data for the external system constraint. */
template <typename T>
struct SapExternalSystemConstraintData {
  T time_step{};  // Time step for the simulation

  T v;          // Constraint velocity
  T cost{};     // Cost ℓ(v)
  T impulse{};  // Impulse γ(v) = −∂ℓ(v)/∂v.
  T hessian{};  // Hessian G = −∂γ(v)/∂v = ∂²ℓ(v)/∂v².
};

/**
 * Defines an external system constraint τ = clamp(-k v + τ₀, -e, e).
 *
 * This is a slight generalization of SapPdControllerConstraint, and is used for
 * considering external systems implicitly in the convex integrator.
 */
template <typename T>
class SapExternalSystemConstraint final : public SapConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing.
  Protected copy construction is enabled for sub-classes to use in their
  implementation of DoClone(). */
  //@{
  SapExternalSystemConstraint& operator=(const SapExternalSystemConstraint&) =
      delete;
  SapExternalSystemConstraint(SapExternalSystemConstraint&&) = delete;
  SapExternalSystemConstraint& operator=(SapExternalSystemConstraint&&) =
      delete;
  //@}

  /**
   *  Struct identifying the clique and DoF in question for this constraint.
   */
  struct Configuration {
    int clique;      // The clique index.
    int clique_nv;   // The number of generalized velocities in the clique.
    int clique_dof;  // The index of the dof in question, in [0, clique_nv]
  };

  /**
   * Construct the constraint.
   *
   * @param configuration Indices of the clique and DoF in question.
   * @param k The stiffness parameter. Should be >0.
   * @param tau0 The explicit external forces.
   * @param effort_limits The effort limits.
   */
  SapExternalSystemConstraint(Configuration configuration, const T& k,
                              const T& tau0, const T& effort_limits);

 private:
  /* Private copy construction is enabled to use in the implementation of
    DoClone(). */
  SapExternalSystemConstraint(const SapExternalSystemConstraint&) = default;

  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<SapExternalSystemConstraint<T>>(
        new SapExternalSystemConstraint<T>(*this));
  }
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final;

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
  void DoAccumulateGeneralizedImpulses(
      int c, const Eigen::Ref<const VectorX<T>>& gamma,
      EigenPtr<VectorX<T>> tau) const final;

  // no-op for this constraint
  void DoAccumulateSpatialImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                   SpatialForce<T>*) const final {};

  // Constraint Jacobian for this constraint.
  static SapConstraintJacobian<T> MakeConstraintJacobian(Configuration c);

  // Clamping helper functions from SapPdControllerConstraint
  static T Clamp(const T& x, const T& e) {
    using std::clamp;  // max(min(x, e), -e);
    return clamp(x, -e, e);
  }

  static T ClampDerivative(const T& x, const T& e) {
    if (-e <= x && x <= e) return 1.0;
    return 0.0;
  }

  static T ClampAntiderivative(const T& x, const T& e) {
    if (x < -e) {
      return -e * (x + e / 2);
    } else if (-e <= x && x <= e) {
      return x * x / 2;
    } else {
      return e * (x - e / 2);
    }
  }

  // Clique/velocity configuration
  Configuration configuration_;

  // Constraint parameters
  const T k_;     // Linearized external systems dynamics
  const T tau0_;  // Explicit external forces
  const T e_;     // Effort limit
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
