#include "drake/multibody/contact_solvers/schur_complement.h"

#include <algorithm>
#include <optional>
#include <utility>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

SchurComplement::SchurComplement(const Block3x3SparseSymmetricMatrix& A,
                                 const std::unordered_set<int>& D_indices)
    : D_indices_(D_indices.begin(), D_indices.end()) {
  /* Keep D_indices_ sorted. */
  std::sort(D_indices_.begin(), D_indices_.end());
  /* If a block index doesn't belong to the D blocks, it belongs to the C
   blocks. */
  C_indices_.reserve(A.block_cols() - D_indices.size());
  for (int j = 0; j < A.block_cols(); ++j) {
    if (D_indices.count(j) == 0) {
      C_indices_.push_back(j);
    }
  }
  const int block_cols = C_indices_.size() + D_indices_.size();
  DRAKE_DEMAND(block_cols * 3 == A.cols());
  std::optional<MatrixX<double>> S =
      solver_.FactorAndCalcSchurComplement(A, D_indices);
  if (!S) {
    throw std::runtime_error(
        "Factorization failed when computing Schur complement. Make sure the "
        "matrix is symmetric positive definite and not ill-conditioned.");
  }
  DRAKE_DEMAND(
      solver_.solver_mode() ==
      BlockSparseCholeskySolver<Matrix3<double>>::SolverMode::kFactored);
  S_ = std::move(*S);
}

VectorX<double> SchurComplement::SolveForX(
    const Eigen::Ref<const VectorX<double>>& y) const {
  DRAKE_THROW_UNLESS(y.size() == 3 * ssize(C_indices_));
  DRAKE_DEMAND(
      solver_.solver_mode() ==
      BlockSparseCholeskySolver<Matrix3<double>>::SolverMode::kFactored);
  if (D_indices_.empty()) {
    return VectorX<double>::Zero(0);
  }
  if (C_indices_.empty()) {
    return VectorX<double>::Zero(D_indices_.size());
  }

  /* Build `a`, the rhs corrsponding to variable y. */
  const VectorX<double> a = S_ * y;
  /* Build the full rhs. */
  const int block_cols = C_indices_.size() + D_indices_.size();
  VectorX<double> b(VectorX<double>::Zero(3 * block_cols));
  for (int i = 0; i < ssize(C_indices_); ++i) {
    b.segment<3>(3 * C_indices_[i]) = a.segment<3>(3 * i);
  }
  VectorX<double>& z = b;
  solver_.SolveInPlace(&z);
  /* Extract x from the z. */
  VectorX<double> x(3 * D_indices_.size());
  for (int i = 0; i < ssize(D_indices_); ++i) {
    x.segment<3>(3 * i) = z.segment<3>(3 * D_indices_[i]);
  }
  return x;
}

VectorX<double> SchurComplement::Solve(
    const Eigen::Ref<const VectorX<double>>& c) const {
  DRAKE_THROW_UNLESS(3 * (ssize(C_indices_) + ssize(D_indices_)) == c.size());
  DRAKE_DEMAND(
      solver_.solver_mode() ==
      BlockSparseCholeskySolver<Matrix3<double>>::SolverMode::kFactored);
  return solver_.Solve(c);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
