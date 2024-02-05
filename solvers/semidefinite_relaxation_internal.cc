#include "drake/solvers/semidefinite_relaxation_internal.h"

#include <iostream>

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

Eigen::SparseMatrix<double>
ComputeTensorProductOfSymmetricMatrixToRealVecOperators(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::SparseMatrix<double>& B) {
  auto r_choose_2 = [](const int r) {
    return r * (r - 1) / 2;
  };
  auto compute_inv_triangular_number = [](const int Tr) {
    return static_cast<int>((-1 + sqrt(1 + 8 * Tr)) / 2);
  };
  const int result_symmetric_space_cols =
      compute_inv_triangular_number(A.cols()) *
      compute_inv_triangular_number(B.cols());
  const int result_cols = r_choose_2(result_symmetric_space_cols + 1);
  const int result_rows = A.rows() * B.rows();

  Eigen::SparseMatrix<double> C(result_rows, result_cols);
  std::vector<Eigen::Triplet<double>> C_triplets;

  int prod_row_idx = 0;
  int y_idx = 0;
  // Now iterate over the non-zero entries of the flattened outerproduct
  // A.col(i) * B.col(j).T for all (i,j) pairs in the operator domain space.

  for (int iA = 0; iA < A.outerSize(); ++iA) {
    for (int iB = 0; iB < B.outerSize(); ++iB) {
      for (SparseMatrix<double>::InnerIterator itA(A, iA); itA; ++itA) {
        int i = itA.col();
        for (SparseMatrix<double>::InnerIterator itB(B, iB); itB; ++itB) {
          int j = itB.col();
          y_idx = 0;
          for (int yc = 0; yc < result_symmetric_space_cols; ++yc) {
            for (int yr = yc; yr < result_symmetric_space_cols; ++yr) {
              int row = itA.row() * B.rows() + itB.row();
              int col = y_idx;
              double value = itA.value() * itB.value();
              //              std::cout << "C_triplets: " << row << ", " << col
              //              << ", " << value
              //                        << std::endl;

              C_triplets.emplace_back(row, col, value);
              ++y_idx;
            }
          }
        }
      }
      ++prod_row_idx;
    }
  }
  std::cout << "prod row = " << prod_row_idx << std::endl;
  std::cout << "y_idx= " << y_idx << std::endl;
  std::cout << fmt::format("C size = ({}, {})", C.rows(), C.cols())
            << std::endl;
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
  if (std::min(X.rows(), X.cols()) <= 2) {
    AddMatrixIsLorentzSeparableConstraintSimplicialCase(X, prog);
    return;
  } else {
    const int m = X.rows();
    const int n = X.cols();
    auto Y = prog->NewSymmetricContinuousVariables((n - 1) * (m - 1));
    prog->AddPositiveSemidefiniteConstraint(Y);

    const Eigen::VectorX<symbolic::Variable> y =
        math::ToLowerTriangularColumnsFromMatrix(Y);
    const Eigen::VectorX<symbolic::Variable> x =
        Eigen::Map<const Eigen::VectorX<symbolic::Variable>>(X.data(),
                                                             X.size());

    const SparseMatrix<double> W_adj_n = GetWAdjForTril(n);
    const SparseMatrix<double> W_adj_m = GetWAdjForTril(m);
    const SparseMatrix<double> W_adj_n_Kron_W_adj_m =
        ComputeTensorProductOfSymmetricMatrixToRealVecOperators(
            GetWAdjForTril(n), GetWAdjForTril(m));

    const SparseMatrix<double> SkewAdjoint_n_min_1 =
        GetSkewAdjointForLowerTri(n - 1);
    const SparseMatrix<double> SkewAdjoint_m_min_1 =
        GetSkewAdjointForLowerTri(m - 1);
    const SparseMatrix<double> SkewAdjoint_n_min_1_Kron_SkewAdjoint_m_min_1 =
        ComputeTensorProductOfSymmetricMatrixToRealVecOperators(
            SkewAdjoint_n_min_1, SkewAdjoint_m_min_1);

    std::cout << fmt::format("W_n = ({},{})", W_adj_n.rows(), W_adj_n.cols())
              << std::endl;
    std::cout << fmt::format("W_m = ({},{})", W_adj_m.rows(), W_adj_m.cols())
              << std::endl;
    std::cout << fmt::format("W = ({},{})", W_adj_n_Kron_W_adj_m.rows(),
                             W_adj_n_Kron_W_adj_m.cols())
              << std::endl;
    std::cout << fmt::format("A_(n-1) = ({},{})", SkewAdjoint_n_min_1.rows(),
                             SkewAdjoint_n_min_1.cols())
              << std::endl;
    std::cout << fmt::format("A_(m-1) = ({},{})", SkewAdjoint_m_min_1.rows(),
                             SkewAdjoint_m_min_1.cols())
              << std::endl;
    std::cout << fmt::format(
                     "A = ({},{})",
                     SkewAdjoint_n_min_1_Kron_SkewAdjoint_m_min_1.rows(),
                     SkewAdjoint_n_min_1_Kron_SkewAdjoint_m_min_1.cols())
              << std::endl;
    std::cout << fmt::format("y_size = {}", y.rows()) << std::endl;
    std::cout << fmt::format("x_size = {}", x.rows()) << std::endl;

    std::cout << fmt::format("W =\n{}",
                             fmt_eigen(W_adj_n_Kron_W_adj_m.toDense()))
              << std::endl;
    // TODO(Alexandre.Amice) make sure these stay as sparse
    prog->AddLinearEqualityConstraint(W_adj_n_Kron_W_adj_m * y - x,
                                      Eigen::VectorXd::Zero(x.size()));
    prog->AddLinearEqualityConstraint(
        SkewAdjoint_n_min_1_Kron_SkewAdjoint_m_min_1,
        Eigen::VectorXd::Zero(
            SkewAdjoint_n_min_1_Kron_SkewAdjoint_m_min_1.rows()),
        y);
  }
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake