#pragma once

#include <map>
#include <optional>

#include "drake/math/matrix_util.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace internal {

// Validates that we can compute the semidefinite relaxation of prog.
void ValidateProgramIsSupported(const MathematicalProgram& prog);

// Checks whether the program prog has any non-convex quadratic costs or
// constraints.
bool CheckProgramHasNonConvexQuadratics(const MathematicalProgram& prog);

// Adds the initialization of the semidefinite relaxation of prog to relaxation.
// Let z represent the variables of prog. First we sort z into a vector y and
// the resorted indices are stored in variables_to_sorted_indices. The matrix X
// is set to be equal to X = [[Y, y], [yᵀ, one]]. The variable Y is a symmetric
// matrix variable and both y and Y are added as decision variables to the
// relaxation. The matrix X is constrained to be PSD. The optional group_number
// is used to name the variables in the matrix Y. We assume that the variable
// ones is already a decision variable of relaxation.
// [in/out] relaxation A pointer to a mathematical program to which the
// initialization semidefinite relaxation of prog will be added. This pointer
// cannot be null.
// [out] X The matrix X of the semidefinite relaxation of prog.
// This pointer cannot be null.
// [out] variables_to_sorted_indices A map from the decision variables z of prog
// to their index in y. The original values in variables_to_sorted_indices are
// cleared. This pointer cannot be null.
void InitializeSemidefiniteRelaxationForProg(
    const MathematicalProgram& prog, const symbolic::Variable& one,
    MathematicalProgram* relaxation, MatrixX<symbolic::Variable>* X,
    std::map<symbolic::Variable, int>* variables_to_sorted_indices,
    std::optional<int> group_number = std::nullopt);

// Iterates over the quadratic costs and constraints in prog, remove them if
// present in the relaxation, and add an equivalent linear cost or constraint on
// the semidefinite variable X. The map variables_to_sorted_indices maps the
// decision variables in prog to their index in the last column of X. [in/out]
// relaxation A pointer to a mathematical program to which the linearized costs
// and constraints are added. It cannot be null.
void DoLinearizeQuadraticCostsAndConstraints(
    const MathematicalProgram& prog, const MatrixXDecisionVariable& X,
    const std::map<symbolic::Variable, int>& variables_to_sorted_indices,
    MathematicalProgram* relaxation);

// Aggregates all the finite linear constraints in the program into a single
// expression Ay ≤ b, which can be expressed as [A, -b][y; 1] ≤ 0.
// We add the implied linear constraint [A,-b]X[A,-b]ᵀ ≤ 0 on the variable X to
// the relaxation. The map variables_to_sorted_indices maps the
// decision variables in prog to their index in the last column of X.
// [in/out] relaxation A pointer to a mathematical program to which the implied
// linear constraints are added. It cannot be null.
void DoAddImpliedLinearConstraints(
    const MathematicalProgram& prog, const MatrixXDecisionVariable& X,
    const std::map<symbolic::Variable, int>& variables_to_sorted_indices,
    MathematicalProgram* relaxation);

// For every equality constraint Ay = b in prog, adds the implied linear
// equality constraint [A, -b]X = 0 on the semidefinite relaxation variable X to
// the relaxation. The map variables_to_sorted_indices maps the decision
// variables in prog to their index in the last column of X.
// [in/out] relaxation A pointer to a mathematical program to which the implied
// linear equality constraints are added. It cannot be null.
void DoAddImpliedLinearEqualityConstraints(
    const MathematicalProgram& prog, const MatrixXDecisionVariable& X,
    const std::map<symbolic::Variable, int>& variables_to_sorted_indices,
    MathematicalProgram* relaxation);

// Takes the sparse Kronecker product of A and B.
Eigen::SparseMatrix<double> SparseKroneckerProduct(
    const Eigen::SparseMatrix<double>& A, const Eigen::SparseMatrix<double>& B);

// Gets the matrix representation for the map Y -> [tr(Y), y00 - ∑ᵢ₌₁ʳ⁻¹yᵢᵢ,
// 2y_{0i-1}] when applied to the lower triangular part of Y as a column. This
// map goes from symmetric matrices with r - 1 rows to a vector with r rows.
Eigen::SparseMatrix<double> GetWAdjForTril(const int r);

// Given a vector expressing an element of Sⁿ ⊗ Sᵐ, returns the corresponding
// symmetric matrix of size Sⁿᵐ. Note that Sⁿ ⊗ Sᵐ is a
// subspace of Sⁿᵐ of dimension (n+1) choose 2 * (m+1) choose 2. Therefore, this
// method requires that tensor_vector.rows() be of size
// ((n+1) choose 2) * ((m+1) choose 2).
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

}  // namespace internal
}  // namespace solvers
}  // namespace drake
