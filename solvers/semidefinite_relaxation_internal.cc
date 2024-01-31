#include "drake/solvers/semidefinite_relaxation_internal.h"

#include <iostream>

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

Eigen::SparseMatrix<double> GetSkewAdjointForLowerTri(const int r) {
  DRAKE_DEMAND(r > 0);
  const int map_num_rows = (r * (r - 1)) / 2;
  const int map_num_cols = ((r + 1) * r) / 2;
  std::vector<Triplet<double>> map_triplets;
  map_triplets.reserve(map_num_rows);
  int i = 0;
  int j = 0;
  for (int row = 0; row < r; ++row) {
    for (int col = row; col < r; ++col) {
      if (row != col) {
        map_triplets.emplace_back(i, j, -2);
        ++i;
      }
      ++j;
    }
  }
  Eigen::SparseMatrix<double> map(map_num_rows, map_num_cols);
  map.setFromTriplets(map_triplets.begin(), map_triplets.end());
  return map;
}

namespace {

// Special case when min(X.rows(), X.cols()) ≤ 2.
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
  if(std::min(X.rows(), X.cols()) <= 2) {
    AddMatrixIsLorentzSeparableConstraintSimplicialCase(X, prog);
    return;
  }
  else {
    auto Y =
  }
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake