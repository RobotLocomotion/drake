#pragma once

#include <mutex>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace solvers {
namespace internal {
/*
 * This class is typically used in cost and constraint class to store a sparse
 * matrix and an optional dense matrix. It also provides setter/getter for the
 * matrix.
 */
class SparseAndDenseMatrix {
 public:
  // Delete these constructors for thread safety.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SparseAndDenseMatrix);
  explicit SparseAndDenseMatrix(const Eigen::SparseMatrix<double>& sparse);

  explicit SparseAndDenseMatrix(Eigen::MatrixXd dense);

  SparseAndDenseMatrix& operator=(const Eigen::SparseMatrix<double>& sparse);

  SparseAndDenseMatrix& operator=(Eigen::MatrixXd dense);

  ~SparseAndDenseMatrix();

  [[nodiscard]] const Eigen::SparseMatrix<double>& get_as_sparse() const {
    return sparse_;
  }

  // Getter for the dense matrix. If the class is constructed from a dense
  // matrix, or GetAsDense() function has been called before, then this is a
  // quick operation. Otherwise this function will internally create a dense
  // matrix.
  [[nodiscard]] const Eigen::MatrixXd& GetAsDense() const;

  // Returns true if all element in the matrix is finite. False otherwise.
  [[nodiscard]] bool IsFinite() const;

  // Returns true if the dense matrix has been constructed.
  [[nodiscard]] bool is_dense_constructed() const;

 private:
  Eigen::SparseMatrix<double> sparse_;
  // If dense_.size() == 0, then we only use sparse_;
  Eigen::MatrixXd dense_;
  mutable std::mutex mutex_;
};
}  // namespace internal
}  // namespace solvers
}  // namespace drake
