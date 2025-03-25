#pragma once

#include <memory>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using multibody::SpatialForce;
using multibody::contact_solvers::internal::SapConstraint;
using multibody::contact_solvers::internal::SapConstraintJacobian;

/**
 * Defines an external system constraint τ = τ₀ − Ãv.
 */
template <typename T>
class SapExternalSystemConstraint final : public SapConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing.
  Protected copy construction is enabled for sub-classes to use in their
  implementation of DoClone(). */
  //@{
  SapExternalSystemConstraint& operator=(const SapExternalSystemConstraint&) = delete;
  SapExternalSystemConstraint(SapExternalSystemConstraint&&) = delete;
  SapExternalSystemConstraint& operator=(SapExternalSystemConstraint&&) = delete;
  //@}

  SapExternalSystemConstraint(int clique, int nv);

 private:
  /* Private copy construction is enabled to use in the implementation of
    DoClone(). */
  SapExternalSystemConstraint(const SapExternalSystemConstraint&) = default;
  
  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<SapExternalSystemConstraint<T>>(
        new SapExternalSystemConstraint<T>(*this));
  }
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final {
    return std::unique_ptr<SapExternalSystemConstraint<double>>(
        new SapExternalSystemConstraint<double>(this->clique(0),
                                                this->num_velocities(0)));
  }
  
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

  // no-ops for this constraint
  void DoAccumulateSpatialImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                   SpatialForce<T>*) const final {};

  // Constraint Jacobian for this constraint.
  static SapConstraintJacobian<T> MakeConstraintJacobian(int clique, int nv);

};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
