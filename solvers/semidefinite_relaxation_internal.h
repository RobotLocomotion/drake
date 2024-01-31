#pragma once

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace internal {

// Get the matrix for the map Y -> [tr(Y), y00 - ∑ᵢ₌₁ʳ⁻¹yᵢᵢ, 2y_{0i-1} when
// applied to the lower triangular part of Y as a column. This map goes from
// symmetric matrices with r - 1 rows to vectors with r rows.
Eigen::SparseMatrix<double> GetWAdjForLowerTriPsd(const int r);

// Take the sparse Kronecker product of A and B.
Eigen::SparseMatrix<double> SparseKroneckerProduct(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::SparseMatrix<double>& B);

}  // namespace internal
}  // namespace solvers
}  // namespace drake
