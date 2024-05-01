#include "drake/solvers/semidefinite_relaxation_internal.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "drake/common/fmt_eigen.h"
#include "drake/common/symbolic/decompose.h"
#include "drake/math/matrix_util.h"

namespace drake {
namespace solvers {
namespace internal {
using Eigen::SparseMatrix;
using Eigen::Triplet;
using symbolic::Expression;
using symbolic::Variable;
using symbolic::Variables;

namespace {
const double kInf = std::numeric_limits<double>::infinity();
}  // namespace

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
template <typename T,
          typename = std::enable_if_t<std::is_same_v<T, Expression> ||
                                      std::is_same_v<T, Variable>>>
void DoAddMatrixIsLorentzByPositiveOrthantSeparableConstraint(
    const Eigen::Ref<const MatrixX<T>>& X, MathematicalProgram* prog) {
  for (int i = 0; i < X.cols(); ++i) {
    // TODO(Alexandre.Amice) AddLorentzConeConstraint only has a MatrixBase
    // version of the method not an Eigen::Ref version and so we need to make
    // this temporary copy rather than being able to call
    // prog->AddLorentzConeConstraint(X.col(i));
    const VectorX<T> x = X.col(i);
    prog->AddLorentzConeConstraint(x);
  }
}
}  //  namespace

void AddMatrixIsLorentzByPositiveOrthantSeparableConstraint(
    const Eigen::Ref<const MatrixX<Variable>>& X, MathematicalProgram* prog) {
  DoAddMatrixIsLorentzByPositiveOrthantSeparableConstraint<Variable>(X, prog);
}

void AddMatrixIsLorentzByPositiveOrthantSeparableConstraint(
    const Eigen::Ref<const MatrixX<Expression>>& X, MathematicalProgram* prog) {
  DoAddMatrixIsLorentzByPositiveOrthantSeparableConstraint<Expression>(X, prog);
}

void AddMatrixIsPositiveOrthantByLorentzSeparableConstraint(
    const Eigen::Ref<const MatrixX<Variable>>& X, MathematicalProgram* prog) {
  DoAddMatrixIsLorentzByPositiveOrthantSeparableConstraint<Variable>(
      X.transpose(), prog);
}

void AddMatrixIsPositiveOrthantByLorentzSeparableConstraint(
    const Eigen::Ref<const MatrixX<Expression>>& X, MathematicalProgram* prog) {
  DoAddMatrixIsLorentzByPositiveOrthantSeparableConstraint<Expression>(
      X.transpose(), prog);
}

namespace {
// Special case when min(X.rows(), X.cols()) ≤ 2. In this case, the Lorentz cone
// is just a linear transformation of the positive orthant (i.e. the Lorentz
// cone is simplicial). Therefore, being Lorentz-Lorentz separable is
// equivalent to being Lorentz by positive orthant separable and should be
// encoded as such.
template <typename T,
          typename = std::enable_if_t<std::is_same_v<T, Expression> ||
                                      std::is_same_v<T, Variable>>>
void DoAddMatrixIsLorentzByLorentzSeparableConstraintSimplicialCase(
    const Eigen::Ref<const Eigen::MatrixX<T>>& X, MathematicalProgram* prog) {
  DRAKE_DEMAND(X.rows() <= 2 || X.cols() <= 2);
  // In 2d, the Lorentz cone is x₀ ≥ 0 and x₀ ≥ √x₁² and so is equivalent to the
  // constraint linear constraints x₀ ≥ 0, x₀ ≥ x₁, x₀ ≥ -x₁. This can be
  // expressed as R(π/4) x ∈ ⊗ R₊ⁿ, where R(π/4) is rotation matrix by π/4
  // counterclockwise. In 1d, the Lorentz cone is just x₀ ≥ 0 so there is
  // nothing special that needs to be done.
  Eigen::MatrixX<Expression> X_expr = X.template cast<Expression>();
  const Eigen::Matrix2d R{{1 / std::sqrt(2), -1 / std::sqrt(2)},
                          {1 / std::sqrt(2), 1 / std::sqrt(2)}};
  if (X.rows() == 2) {
    X_expr = R * X_expr;
  }
  if (X.cols() == 2) {
    X_expr = X_expr * R.transpose();
  }

  if (X.rows() <= 2 && X.cols() <= 2) {
    // In this case, we have that X_expr must be
    // positive-orthant-positive-orthant separable, i.e. pointwise positive.
    prog->AddLinearConstraint(
        X_expr, Eigen::MatrixXd::Zero(X_expr.rows(), X_expr.cols()),
        kInf * Eigen::MatrixXd::Ones(X_expr.rows(), X_expr.cols()));
  } else if (X.rows() > 2) {
    AddMatrixIsLorentzByPositiveOrthantSeparableConstraint(X_expr, prog);
  } else if (X.cols() > 2) {
    AddMatrixIsPositiveOrthantByLorentzSeparableConstraint(X_expr, prog);
  } else {
    // Unreachable since X.rows() <= 2 || X.cols() <= 2.
    DRAKE_UNREACHABLE();
  }
}

// Constrain that the affine expression A * x + b, is Lorentz by Lorentz
// separable. Concretely, this means that the matrix
// Z = (A * x + b).reshaped(m,n) can be written as Z = ∑ᵢ λᵢuᵢwᵢᵀ where λᵢ ≥ 0,
// uᵢ is in the Lorentz cone of size m and w is in the Lorentz cone of size n.
void DoAddMatrixIsLorentzByLorentzSeparableConstraint(
    const SparseMatrix<double>& A, const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const VectorX<Variable>>& x, const int m, const int n,
    MathematicalProgram* prog) {
  DRAKE_DEMAND(A.rows() == n * m);
  DRAKE_DEMAND(A.cols() == x.rows());
  DRAKE_DEMAND(n > 2 || m > 2);

  // The lower triangular part of Y ∈ S⁽ⁿ⁻¹⁾ ⊗ S⁽ᵐ⁻¹⁾
  auto y = prog->NewContinuousVariables((n * (n - 1) * m * (m - 1)) / 4, "y");
  MatrixX<Variable> Y = ToSymmetricMatrixFromTensorVector(y, n - 1, m - 1);
  prog->AddPositiveSemidefiniteConstraint(Y);

  const SparseMatrix<double> W_adj_n = GetWAdjForTril(n);
  const SparseMatrix<double> W_adj_m = GetWAdjForTril(m);
  // [W_adj_n ⊗ W_adj_m; -A]
  SparseMatrix<double> CoefficientMat(
      W_adj_m.rows() * W_adj_n.rows(),
      W_adj_m.cols() * W_adj_n.cols() + A.cols());
  std::vector<Triplet<double>> CoefficientMat_triplets;
  CoefficientMat_triplets.reserve(W_adj_n.nonZeros() * W_adj_m.nonZeros() +
                                  A.nonZeros());
  // Set the left columns of CoefficientMat to W_adj_n ⊗ W_adj_m.
  for (int idx_n = 0; idx_n < W_adj_n.outerSize(); ++idx_n) {
    for (SparseMatrix<double>::InnerIterator it_n(W_adj_n, idx_n); it_n;
         ++it_n) {
      for (int idx_m = 0; idx_m < W_adj_m.outerSize(); ++idx_m) {
        for (SparseMatrix<double>::InnerIterator it_m(W_adj_m, idx_m); it_m;
             ++it_m) {
          CoefficientMat_triplets.emplace_back(
              it_n.row() * W_adj_m.rows() + it_m.row(),
              it_n.col() * W_adj_m.cols() + it_m.col(),
              it_n.value() * it_m.value());
        }
      }
    }
  }
  int col = W_adj_m.cols() * W_adj_n.cols();
  // Set the right columns of CoefficientMat to -A.
  for (int i = 0; i < A.outerSize(); ++i) {
    for (SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
      CoefficientMat_triplets.emplace_back(it.row(), it.col() + col,
                                           -it.value());
    }
  }
  CoefficientMat.setFromTriplets(CoefficientMat_triplets.begin(),
                                 CoefficientMat_triplets.end());
  VectorX<Variable> yx(y.size() + x.size());
  yx << y, x;
  prog->AddLinearEqualityConstraint(CoefficientMat, b, yx);
}

}  // namespace

void AddMatrixIsLorentzByLorentzSeparableConstraint(
    const Eigen::Ref<const MatrixX<symbolic::Variable>>& X,
    MathematicalProgram* prog) {
  if (std::min(X.rows(), X.cols()) <= 2) {
    DoAddMatrixIsLorentzByLorentzSeparableConstraintSimplicialCase(X, prog);
  } else {
    const int m = X.rows();
    const int n = X.cols();
    SparseMatrix<double> I(n * m, n * m);
    I.setIdentity();
    const Eigen::VectorX<Variable> x =
        Eigen::Map<const Eigen::VectorX<Variable>>(X.data(), X.size());
    DoAddMatrixIsLorentzByLorentzSeparableConstraint(
        I, Eigen::VectorXd::Zero(I.rows()),
        Eigen::Map<const Eigen::VectorX<Variable>>(X.data(), X.size()), m, n,
        prog);
  }
}

void AddMatrixIsLorentzByLorentzSeparableConstraint(
    const Eigen::Ref<const MatrixX<Expression>>& X, MathematicalProgram* prog) {
  if (std::min(X.rows(), X.cols()) <= 2) {
    DoAddMatrixIsLorentzByLorentzSeparableConstraintSimplicialCase(X, prog);
  } else {
    const int m = X.rows();
    const int n = X.cols();
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    VectorX<Variable> x;
    symbolic::DecomposeAffineExpressions(
        Eigen::Map<const VectorX<Expression>>(X.data(), X.size()), &A, &b, &x);
    DoAddMatrixIsLorentzByLorentzSeparableConstraint(A.sparseView(), b, x, m, n,
                                                     prog);
  }
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
