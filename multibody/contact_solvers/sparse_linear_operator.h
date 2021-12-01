#pragma once

#include <string>

#include <Eigen/SparseCore>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/linear_operator.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// A LinearOperator that wraps an existing Eigen::SparseMatrix.
//
// @tparam_nonsymbolic_scalar
template <typename T>
class SparseLinearOperator final : public LinearOperator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SparseLinearOperator)

  // Constructs an operator with given `name` implementing the LinearOperator
  // interface for matrix `A`.
  // This class keeps a reference to input matrix `A` and therefore it is
  // required that it outlives this object.
  SparseLinearOperator(const std::string& name, const Eigen::SparseMatrix<T>* A)
      : LinearOperator<T>(name), A_(A) {
    DRAKE_DEMAND(A != nullptr);
  }

  ~SparseLinearOperator() = default;

  int rows() const final { return A_->rows(); }
  int cols() const final { return A_->cols(); }

 protected:
  void DoMultiply(const Eigen::Ref<const VectorX<T>>& x,
                  VectorX<T>* y) const final {
    *y = *A_ * x;
  };

  void DoMultiply(const Eigen::Ref<const Eigen::SparseVector<T>>& x,
                  Eigen::SparseVector<T>* y) const final {
    *y = *A_ * x;
  }

  void DoMultiplyByTranspose(const Eigen::Ref<const VectorX<T>>& x,
                             VectorX<T>* y) const final {
    *y = A_->transpose() * x;
  }

  void DoMultiplyByTranspose(const Eigen::Ref<const Eigen::SparseVector<T>>& x,
                             Eigen::SparseVector<T>* y) const final {
    *y = A_->transpose() * x;
  }

  // Bring LinearOperator's default DoAssembleMatrix() into scope so that
  // overrides here do not hide the other overloads.
  using LinearOperator<T>::DoAssembleMatrix;

  void DoAssembleMatrix(Eigen::SparseMatrix<T>* A) const final {
    *A = *A_;
  }

 private:
  const Eigen::SparseMatrix<T>* A_{nullptr};
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SparseLinearOperator)
