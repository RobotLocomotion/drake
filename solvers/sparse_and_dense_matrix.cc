#include "drake/solvers/sparse_and_dense_matrix.h"

#include <utility>

namespace drake {
namespace solvers {
namespace internal {
SparseAndDenseMatrix::SparseAndDenseMatrix(Eigen::SparseMatrix<double> sparse)
    : sparse_(std::move(sparse)), dense_(0, 0) {}

SparseAndDenseMatrix::SparseAndDenseMatrix(
    const Eigen::Ref<const Eigen::MatrixXd>& dense)
    : sparse_(dense.sparseView()), dense_(dense) {}

SparseAndDenseMatrix& SparseAndDenseMatrix::operator=(Eigen::MatrixXd dense) {
  this->dense_ = std::move(dense);
  this->sparse_ = this->dense_.sparseView();
  return *this;
}

SparseAndDenseMatrix& SparseAndDenseMatrix::operator=(
    const Eigen::SparseMatrix<double>& sparse) {
  this->sparse_ = sparse;
  this->dense_.resize(0, 0);
  return *this;
}

const Eigen::MatrixXd& SparseAndDenseMatrix::GetAsDense() const {
  if (dense_.size() != 0) {
    return dense_;
  } else {
    // Modifies the dense_ field. For thread-safety, use a std::mutex.
    std::lock_guard<std::mutex> guard(mutex_);
    dense_ = sparse_.toDense();
    return dense_;
  }
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake
