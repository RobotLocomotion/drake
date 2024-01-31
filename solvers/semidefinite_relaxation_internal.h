#pragma once

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace internal {

// Take the sparse Kronecker product of A and B.
Eigen::SparseMatrix<double> SparseKroneckerProduct(
    const Eigen::SparseMatrix<double>& A, const Eigen::SparseMatrix<double>& B);

// Let L and M be linear operators taking symmetric matrices to a real vectors,
// and let A and B be their matrix representation operating on the columnwise
// stacking of the lower triangular part of a symmetric matrix. This function
// outputs a matrix representation of L ⊗ M (which notably is not as simple as A
// ⊗ B).
Eigen::SparseMatrix<double>
ComputeTensorProductOfSymmetricMatrixToRealVecOperators(
    const Eigen::SparseMatrix<double>& A, const Eigen::SparseMatrix<double>& B);

// Get the matrix for the map Y -> [tr(Y), y00 - ∑ᵢ₌₁ʳ⁻¹yᵢᵢ, 2y_{0i-1} when
// applied to the lower triangular part of Y as a column. This map goes from
// symmetric matrices with r - 1 rows to vectors with r rows.
Eigen::SparseMatrix<double> GetWAdjForTril(const int r);

// Returns the map where given the lower triangular part of a matrix Y with r
// rows, returns 2 times the strictly lower triangular part (i.e. the part below
// the main diagonal) stacked as a column. This is the adjoint of the map that
// takes a vector and maps it to a skew symmetric matrix with the vector placed
// on the strictly lower triangular part of the matrix.
Eigen::SparseMatrix<double> GetSkewAdjointForLowerTri(const int r);

// Add the constraint that the matrix is Lorentz separable.
// TODO(Alexandre.Amice) Move this to mathematical_program.h
void AddMatrixIsLorentzSeparableConstraint(
    const Eigen::Ref<const Eigen::MatrixX<symbolic::Variable>>& X,
    MathematicalProgram* prog);

}  // namespace internal
}  // namespace solvers
}  // namespace drake
