#pragma once

#include "drake/math/matrix_util.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace internal {

// Take the sparse Kronecker product of A and B.
Eigen::SparseMatrix<double> SparseKroneckerProduct(
    const Eigen::SparseMatrix<double>& A, const Eigen::SparseMatrix<double>& B);

// Get the matrix representation for the map Y -> [tr(Y), y00 - ∑ᵢ₌₁ʳ⁻¹yᵢᵢ,
// 2y_{0i-1}] when applied to the lower triangular part of Y as a column. This
// map goes from symmetric matrices with r - 1 rows to a vector with r rows.
Eigen::SparseMatrix<double> GetWAdjForTril(const int r);

// Given a vector expressing an element of Sⁿ ⊗ Sᵐ, return the corresponding
// symmetric matrix of size Sⁿᵐ. Note that Sⁿ ⊗ Sᵐ is a
// subspace of Sⁿᵐ of dimension (n+1) choose 2 * (m+1) choose 2. Therefore, this
// method requires that tensor_vector.rows() be of size (n+1) choose 2 * (m+1)
// choose 2.
template <typename Derived>
drake::MatrixX<typename Derived::Scalar> ToSymmetricMatrixFromTensorVector(
    const Eigen::MatrixBase<Derived>& tensor_vector, int n, int m) {
  const int sym_elt_n = (n * (n + 1)) / 2;
  const int sym_elt_m = (m * (m + 1)) / 2;
  DRAKE_THROW_UNLESS(tensor_vector.rows() == sym_elt_n * sym_elt_m);

  // TODO(Alexandre.Amice) Make this efficient.
  drake::MatrixX<typename Derived::Scalar> symmetric_matrix(n * m, n * m);
  for (int i = 0; i < sym_elt_n; ++i) {
    Eigen::SparseVector<double> ei(sym_elt_n);
    ei.insert(i) = 1;
    Eigen::MatrixXd symmetric_matrix_i =
        math::ToSymmetricMatrixFromLowerTriangularColumns(ei.toDense());
    for (int j = 0; j < sym_elt_m; ++j) {
      Eigen::SparseVector<double> ej(sym_elt_m);
      ej.insert(j) = 1;
      Eigen::MatrixXd symmetric_matrix_j =
          math::ToSymmetricMatrixFromLowerTriangularColumns(ej.toDense());

      Eigen::SparseMatrix<double> kron = SparseKroneckerProduct(
          symmetric_matrix_i.sparseView(), symmetric_matrix_j.sparseView());
      for (int k = 0; k < kron.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(kron, k); it; ++it) {
          if (it.value() > 0) {
            symmetric_matrix(it.row(), it.col()) =
                tensor_vector(i * sym_elt_m + j);
          }
        }
      }
    }
  }
  return symmetric_matrix;
}

// TODO(Alexandre.Amice) Move these to mathematical_program.h
// Adds the constraint that the matrix X is Lorentz-positive-orthant separable
// i.e. a conic combination of tensors in ℒᵐ ⊗ R₊ⁿ, where ℒᵐ is the Lorentz cone
// of size m and R₊ⁿ is the positive orthant in n dimensions. Namely X = ∑ᵢ
// λᵢxᵢyᵢᵀ where λᵢ ≥ 0, xᵢ is in the Lorentz cone of size X.rows() and y ≥ 0
// componentwise and is of size X.cols(). This condition is equivalent to each
// column of X being in the Lorentz cone.
void AddMatrixIsLorentzByPositiveOrthantSeparableConstraint(
    const Eigen::Ref<const MatrixX<symbolic::Variable>>& X,
    MathematicalProgram* prog);

// Adds the constraint that the matrix of expressions X is
// Lorentz-positive-orthant separable i.e. a conic combination of tensors
// in ℒᵐ ⊗ R₊ⁿ, where ℒᵐ is the Lorentz cone of size m and R₊ⁿ is the positive
// orthant in n dimensions. Namely X = ∑ᵢ λᵢxᵢyᵢᵀ where λᵢ ≥ 0, xᵢ is in the
// Lorentz cone of size X.rows() and y ≥ 0 componentwise and is of size
// X.cols(). This condition is equivalent to each column of X being in the
// Lorentz cone.
void AddMatrixIsLorentzByPositiveOrthantSeparableConstraint(
    const Eigen::Ref<const MatrixX<symbolic::Expression>>& X,
    MathematicalProgram* prog);

// Adds the constraint that the matrix X is positive-orthant-Lorentz separable
// i.e. a conic combination of tensors in R₊ᵐ ⊗ ℒⁿ, where R₊ᵐ is the positive
// orthant in m dimensions and ℒⁿ is the Lorentz cone of size n. Namely X = ∑ᵢ
// λᵢxᵢyᵢᵀ where λᵢ ≥ 0, xᵢ ≥ 0 componentwise and of size X.rows() and yᵢ is in
// the Lorentz cone of size X.cols(). This condition is equivalent to each row
// of X being in the Lorentz cone.
void AddMatrixIsPositiveOrthantByLorentzSeparableConstraint(
    const Eigen::Ref<const MatrixX<symbolic::Variable>>& X,
    MathematicalProgram* prog);

// Adds the constraint that the matrix of expressions X is
// positive-orthant-Lorentz separable i.e. a conic combination of tensors
// in R₊ᵐ ⊗ ℒⁿ, where R₊ᵐ is the positive orthant in m dimensions and ℒⁿ is the
// Lorentz cone of size n. Namely X = ∑ᵢ λᵢxᵢyᵢᵀ where λᵢ ≥ 0, xᵢ ≥ 0
// componentwise and of size X.rows() and yᵢ is in the Lorentz cone of size
// X.cols(). This condition is equivalent to each row of X being in the
// Lorentz cone.
void AddMatrixIsPositiveOrthantByLorentzSeparableConstraint(
    const Eigen::Ref<const MatrixX<symbolic::Expression>>& X,
    MathematicalProgram* prog);

// Adds the constraint that the matrix X is Lorentz-Lorentz separable i.e. a
// conic combination of tensors in ℒᵐ ⊗ ℒⁿ, where ℒᵐ is the Lorentz cone of size
// m and ℒⁿ is the Lorentz cone of size n. Namely X = ∑ᵢ λᵢxᵢyᵢᵀ where λᵢ ≥ 0,
// xᵢ is in the Lorentz cone of size X.rows() and yᵢ is in the Lorentz cone of
// size X.cols().
void AddMatrixIsLorentzByLorentzSeparableConstraint(
    const Eigen::Ref<const MatrixX<symbolic::Variable>>& X,
    MathematicalProgram* prog);

// Adds the constraint that the matrix of expresssions X is Lorentz-Lorentz
// separable i.e. a conic combination of tensors in ℒᵐ ⊗ ℒⁿ, where ℒᵐ is the
// Lorentz cone of size m and ℒⁿ is the Lorentz cone of size n. Namely X = ∑ᵢ
// λᵢxᵢyᵢᵀ where λᵢ ≥ 0, xᵢ is in the Lorentz cone of size X.rows() and yᵢ is in
// the Lorentz cone of size X.cols().
void AddMatrixIsLorentzByLorentzSeparableConstraint(
    const Eigen::Ref<const MatrixX<symbolic::Expression>>& X,
    MathematicalProgram* prog);

}  // namespace internal
}  // namespace solvers
}  // namespace drake
