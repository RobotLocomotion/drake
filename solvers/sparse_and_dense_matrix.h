#pragma once

#include <mutex>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace solvers {
namespace internal {
/**
 * This class is typically used in cost and constraint class to store a sparse
 * matrix and an optional dense matrix. It also provides setter/getter for the
 * matrix.
 */
class SparseAndDenseMatrix {
 public:
  // Delete these constructors for thread safety.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SparseAndDenseMatrix)
  explicit SparseAndDenseMatrix(Eigen::SparseMatrix<double> sparse);

  explicit SparseAndDenseMatrix(const Eigen::Ref<const Eigen::MatrixXd>& dense);

  const Eigen::SparseMatrix<double>& get_as_sparse() const {
    return sparse_;
  }

  // Getter for the dense matrix. If the class is constructed from a dense
  // matrix, or GetAsDense() function has been called before, then this is a
  // no-op. Otherwise this function will internally create a dense matrix.
  const Eigen::MatrixXd& GetAsDense() const;

  ~SparseAndDenseMatrix() {}

 private:
  Eigen::SparseMatrix<double> sparse_;
  // If dense_.size() == 0, then we only use sparse_;
  mutable Eigen::MatrixXd dense_;
  mutable std::mutex mutex_;
};
}  // namespace internal
}  // namespace solvers
}  // namespace drake
