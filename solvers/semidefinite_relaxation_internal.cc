#include "drake/solvers/semidefinite_relaxation_internal.h"

#include <algorithm>
#include <vector>

#include "drake/common/fmt_eigen.h"
#include "drake/math/matrix_util.h"

namespace drake {
namespace solvers {
namespace internal {
using Eigen::SparseMatrix;
using Eigen::Triplet;

Eigen::SparseMatrix<double> SparseKroneckerProduct(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::SparseMatrix<double>& B) {
  Eigen::SparseMatrix<double> C(A.rows() * B.rows(), A.cols() * B.cols());
  std::vector<Eigen::Triplet<double>> C_triplets;
  C_triplets.reserve(A.nonZeros() * B.nonZeros());
  C.reserve(A.nonZeros() * B.nonZeros());
  for (int iA = 0; iA < A.outerSize(); ++iA) {
    for (SparseMatrix<double>::InnerIterator itA(A, iA); itA; ++itA) {
      for (int iB = 0; iB < B.outerSize(); ++iB) {
        for (SparseMatrix<double>::InnerIterator itB(B, iB); itB; ++itB) {
          C_triplets.emplace_back(itA.row() * B.rows() + itB.row(),
                                  itA.col() * B.cols() + itB.col(),
                                  itA.value() * itB.value());
        }
      }
    }
  }
  C.setFromTriplets(C_triplets.begin(), C_triplets.end());
  return C;
}

SparseMatrix<double> GetWAdjForTril(const int r) {
  DRAKE_DEMAND(r > 0);
  // Y is a symmetric matrix of size (r-1) hence we have (r choose 2) lower
  // triangular entries.
  const int Y_tril_size = (r * (r - 1)) / 2;

  std::vector<Triplet<double>> W_adj_triplets;
  // The map operates on the diagonal twice, and then on one of the columns
  // without the first element once.
  W_adj_triplets.reserve(2 * (r - 1) + (r - 2));

  int idx = 0;
  for (int i = 0; idx < Y_tril_size; ++i) {
    W_adj_triplets.emplace_back(0, idx, 1);
    W_adj_triplets.emplace_back(1, idx, idx > 0 ? -1 : 1);
    idx += (r - 1) - i;
  }

  for (int i = 2; i < r; ++i) {
    W_adj_triplets.emplace_back(i, i - 1, 2);
  }
  SparseMatrix<double> W_adj(r, Y_tril_size);
  W_adj.setFromTriplets(W_adj_triplets.begin(), W_adj_triplets.end());
  return W_adj;
}

namespace {

// Special case when min(X.rows(), X.cols()) ≤ 2. In this case, the lorentz cone
// is just the positive orthant in two dimensions, and therefore this is just
// the product of the positive orthant with the lorentz cone.
void AddMatrixIsLorentzSeparableConstraintSimplicialCase(
    const Eigen::Ref<const Eigen::MatrixX<symbolic::Variable>>& X,
    MathematicalProgram* prog) {
  unused(X);
  unused(prog);
  throw std::logic_error(
      "The case when min(X.rows(), X.cols()) ≤ 2 is not implemented yet.");
}

}  // namespace

void AddMatrixIsLorentzSeparableConstraint(
    const Eigen::Ref<const Eigen::MatrixX<symbolic::Variable>>& X,
    MathematicalProgram* prog) {
  if (std::min(X.rows(), X.cols()) <= 2) {
    AddMatrixIsLorentzSeparableConstraintSimplicialCase(X, prog);
    return;
  }
  const int m = X.rows();
  const int n = X.cols();
  // The lower triagular part of Y ∈ S⁽ⁿ⁻¹⁾ ⊗ S⁽ᵐ⁻¹⁾
  auto y = prog->NewContinuousVariables((n * (n - 1) * m * (m - 1)) / 4, "y");
  Eigen::MatrixX<symbolic::Variable> Y =
      ToSymmetricMatrixFromTensorVector(y, n - 1, m - 1);
  prog->AddPositiveSemidefiniteConstraint(Y);

  const Eigen::VectorX<symbolic::Variable> x =
      Eigen::Map<const Eigen::VectorX<symbolic::Variable>>(X.data(), X.size());

  const SparseMatrix<double> W_adj_n = GetWAdjForTril(n);
  const SparseMatrix<double> W_adj_m = GetWAdjForTril(m);
  // [W_adj_n ⊗ W_adj_m; - I]
  SparseMatrix<double> CoefficientMat(
      W_adj_m.rows() * W_adj_n.rows(),
      W_adj_m.cols() * W_adj_n.cols() + x.rows());
  std::vector<Triplet<double>> CoefficientMat_triplets;
  CoefficientMat_triplets.reserve(W_adj_n.nonZeros() * W_adj_m.nonZeros() +
                                  x.rows());
  // Set the left columns of CoefficientMat to W_adj_n ⊗ W_adj_m.
  for (int iA = 0; iA < W_adj_n.outerSize(); ++iA) {
    for (SparseMatrix<double>::InnerIterator itA(W_adj_n, iA); itA; ++itA) {
      for (int iB = 0; iB < W_adj_m.outerSize(); ++iB) {
        for (SparseMatrix<double>::InnerIterator itB(W_adj_m, iB); itB; ++itB) {
          CoefficientMat_triplets.emplace_back(
              itA.row() * W_adj_m.rows() + itB.row(),
              itA.col() * W_adj_m.cols() + itB.col(),
              itA.value() * itB.value());
        }
      }
    }
  }
  int col = W_adj_m.cols() * W_adj_n.cols();
  for (int i = 0; i < x.rows(); ++i) {
    CoefficientMat_triplets.emplace_back(i, col, -1);
    ++col;
  }
  CoefficientMat.setFromTriplets(CoefficientMat_triplets.begin(),
                                 CoefficientMat_triplets.end());
  VectorX<symbolic::Variable> yx(y.size() + x.size());
  yx << y, x;
  prog->AddLinearEqualityConstraint(CoefficientMat,
                                    Eigen::VectorXd::Zero(x.size()), yx);
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
