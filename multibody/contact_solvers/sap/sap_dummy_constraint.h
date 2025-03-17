#pragma once

#include <memory>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using multibody::SpatialForce;
using multibody::contact_solvers::internal::SapConstraint;
using multibody::contact_solvers::internal::SapConstraintJacobian;

/* Add a proxy constraint for a single clique with one equation.
 Cost, gradient and Hessian are zero. This is useful for making sure all DoFs
 are participating when we create the problem.
 */
template <typename T>
class SapDummyConstraint final : public SapConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing.
  Protected copy construction is enabled for sub-classes to use in their
  implementation of DoClone(). */
  //@{
  SapDummyConstraint& operator=(const SapDummyConstraint&) = delete;
  SapDummyConstraint(SapDummyConstraint&&) = delete;
  SapDummyConstraint& operator=(SapDummyConstraint&&) = delete;
  //@}

  SapDummyConstraint(int clique, int nv)
      : SapConstraint<T>(MakeZeroJacobian(clique, nv), {}) {}

 private:
  /* Private copy construction is enabled to use in the implementation of
    DoClone(). */
  SapDummyConstraint(const SapDummyConstraint&) = default;

  static SapConstraintJacobian<T> MakeZeroJacobian(int clique, int nv) {
    MatrixX<T> J = MatrixX<T>::Zero(1, nv);
    return SapConstraintJacobian<T>(clique, std::move(J));
  }

  std::unique_ptr<AbstractValue> DoMakeData(
      const T&, const Eigen::Ref<const VectorX<T>>&) const override {
    int data = 0;
    return SapConstraint<T>::MoveAndMakeAbstractValue(std::move(data));
  }
  void DoCalcData(const Eigen::Ref<const VectorX<T>>&,
                  AbstractValue*) const override {}
  T DoCalcCost(const AbstractValue&) const override { return 0.0; }
  void DoCalcImpulse(const AbstractValue&,
                     EigenPtr<VectorX<T>> gamma) const override {
    *gamma = Vector1<T>::Zero();
  }
  void DoCalcCostHessian(const AbstractValue&, MatrixX<T>* G) const override {
    G->setZero();
  }

  // no-ops.
  void DoAccumulateGeneralizedImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                       EigenPtr<VectorX<T>>) const final {}
  void DoAccumulateSpatialImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                   SpatialForce<T>*) const final {};

  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<SapDummyConstraint<T>>(
        new SapDummyConstraint<T>(*this));
  }
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final {
    return std::unique_ptr<SapDummyConstraint<double>>(
        new SapDummyConstraint<double>(this->clique(0),
                                       this->num_velocities(0)));
  }
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
