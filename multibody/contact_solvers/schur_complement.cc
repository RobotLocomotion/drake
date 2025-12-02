#include "drake/multibody/contact_solvers/schur_complement.h"

#include <algorithm>
#include <optional>
#include <utility>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

SchurComplement::SchurComplement() = default;

SchurComplement::~SchurComplement() = default;

SchurComplement::SchurComplement(const BlockSparseSymmetricMatrix3d& A,
                                 const std::unordered_set<int>& D_indices)
    : D_indices_(D_indices.begin(), D_indices.end()) {
  /* Keep D_indices_ sorted. */
  DRAKE_THROW_UNLESS(ssize(D_indices) <= A.block_cols());
  std::sort(D_indices_.begin(), D_indices_.end());
  /* If a block index doesn't belong to the D blocks, it belongs to the C
   blocks. We step through `D_indices_` to detect the gaps in order to fill in
   `C_indices`. */
  /* Starting point for each potential missing segment in D_indices_. */
  int start = 0;
  for (int D_index : D_indices_) {
    while (start < D_index) {
      C_indices_.push_back(start++);
    }
    start = D_index + 1;
  }
  /* Add the remaining indices (if any) after the last D index. */
  while (start < A.block_cols()) {
    C_indices_.push_back(start++);
  }

  const int block_cols = C_indices_.size() + D_indices_.size();
  DRAKE_DEMAND(block_cols * 3 == A.cols());
  std::optional<MatrixX<double>> S =
      A_solver_.FactorAndCalcSchurComplement(A, D_indices);
  if (!S) {
    throw std::runtime_error(
        "Factorization failed when computing Schur complement. Make sure the "
        "matrix is symmetric positive definite and not ill-conditioned.");
  }
  DRAKE_DEMAND(
      A_solver_.solver_mode() ==
      BlockSparseCholeskySolver<Matrix3<double>>::SolverMode::kFactored);
  S_ = std::move(*S);
}

/* TODO(xuchenhan-tri): In the following implementation, we construct `a`, the
 segment in the right-hand side corresponding to participating dofs with
 `a = S*y`. From `a`, we construct the full right-hand side `b`, then solve the
 entire system for `z`, and finally extract `x` from `z`. Alternatively, we
 could directly compute `x` from the equation Dx + By = 0. This would require
   1. a representation of `B`, and
   2. the ability to solve the linear system `D`.
 Both of these is avaiable, just not conveniently so:
 We had access to `A` at construction, and we could have saved a sparse
 representation of `B` and use it to efficiently compute `By`. From the
 factorization of `A`, we get a factorization of D as a by-product. So far, our
 profiling in the common case where size(D) is much smaller than size(A) shows
 that the current implementation is not a performance issue, so we have opted
 for simplicity in the implementation. */
VectorX<double> SchurComplement::SolveForX(
    const Eigen::Ref<const VectorX<double>>& y) const {
  DRAKE_THROW_UNLESS(y.size() == 3 * ssize(C_indices_));
  DRAKE_DEMAND(
      A_solver_.solver_mode() ==
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
  A_solver_.SolveInPlace(&z);
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
      A_solver_.solver_mode() ==
      BlockSparseCholeskySolver<Matrix3<double>>::SolverMode::kFactored);
  return A_solver_.Solve(c);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
