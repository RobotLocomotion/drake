#pragma once

#include <string>

#include <Eigen/SparseCore>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/solvers/linear_operator.h"

namespace drake {
namespace multibody {
namespace solvers {

/// This class allows to instantiate a LinearOperator when an explicit
/// Eigen::Sparse matrix is available.
template <typename T>
class SparseLinearOperator final : public LinearOperator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SparseLinearOperator)

  /// Constructs an operator with given `name` implementing the LinearOperator
  /// interface for matrix `A`.
  /// This class keeps a reference to input matrix `A` and therefore it is
  /// required that it outlives this object.
  SparseLinearOperator(const std::string& name, const Eigen::SparseMatrix<T>* A)
      : LinearOperator<T>(name), A_(A) {
    DRAKE_DEMAND(A != nullptr);
  }

  ~SparseLinearOperator() = default;

  int rows() const { return A_->rows(); }
  int cols() const { return A_->cols(); }

  void AssembleMatrix(Eigen::SparseMatrix<T>* A) const final {
    DRAKE_DEMAND(A != nullptr);
    *A = *A_;
  }

 protected:
  void DoMultiply(const VectorX<T>& x, VectorX<T>* y) const final {
    *y = *A_ * x;
  };

  void DoMultiply(const Eigen::SparseVector<T>& x,
                  Eigen::SparseVector<T>* y) const final {
    *y = *A_ * x;
  }

  void DoMultiplyByTranspose(const VectorX<T>& x, VectorX<T>* y) const final {
    *y = A_->transpose() * x;
  }

  void DoMultiplyByTranspose(const Eigen::SparseVector<T>& x,
                             Eigen::SparseVector<T>* y) const final {
    *y = A_->transpose() * x;
  }

 private:
  const Eigen::SparseMatrix<T>* A_{nullptr};
};

}  // namespace solvers
}  // namespace multibody
}  // namespace drake
