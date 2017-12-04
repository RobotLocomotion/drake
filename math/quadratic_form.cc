#include "drake/math/quadratic_form.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

namespace drake {
namespace math {
Eigen::MatrixXd DecomposePSDmatrixIntoXtransposeTimesX(
    const Eigen::Ref<const Eigen::MatrixXd>& Y, double zero_tol) {
  if (Y.rows() != Y.cols()) {
    throw std::runtime_error("Y is not square.");
  }
  if (zero_tol < 0) {
    throw std::runtime_error("zero_tol should be non-negative.");
  }
  Eigen::LLT<Eigen::MatrixXd> llt_Y(Y);
  if (llt_Y.info() == Eigen::Success) {
    return llt_Y.matrixU();
  } else {
    // TODO(hongkai.dai) Switch to use robust Choleskly decomposition instead
    // of Eigen value decomposition, when the bug in
    // http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1479 is fixed.
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_Y(Y);
    if (es_Y.info() == Eigen::Success) {
      Eigen::MatrixXd X(Y.rows(), Y.cols());
      int X_row_count = 0;
      for (int i = 0; i < es_Y.eigenvalues().rows(); ++i) {
        if (es_Y.eigenvalues()(i) < -zero_tol) {
          throw std::runtime_error("Y is not positive definite.");
        } else if (es_Y.eigenvalues()(i) > zero_tol) {
          X.row(X_row_count++) = std::sqrt(es_Y.eigenvalues()(i)) *
                                 es_Y.eigenvectors().col(i).transpose();
        }
      }
      return X.topRows(X_row_count);
    }
  }
  throw std::runtime_error("Y is not PSD.");
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> DecomposePositiveQuadraticForm(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& b, double c, double tol) {
  if (Q.rows() != Q.cols()) {
    throw std::runtime_error("Q should be a square matrix.");
  }
  if (b.rows() != Q.rows()) {
    throw std::runtime_error("b does not have the right size.");
  }
  // The quadratic form xᵀQx + bᵀx + c can also be written as
  // [x]ᵀ * [Q   b/2] * [x]
  // [1]    [b/2   c]   [1]
  // We will call the matrix in the middle as M
  Eigen::MatrixXd M(Q.rows() + 1, Q.rows() + 1);
  // clang-format on
  M << (Q + Q.transpose()) / 2, b / 2,
       b.transpose() / 2, c;
  // clang-format off

  const Eigen::MatrixXd A = DecomposePSDmatrixIntoXtransposeTimesX(M, tol);
  Eigen::MatrixXd R = A.leftCols(Q.cols());
  Eigen::VectorXd d = A.col(Q.cols());
  return std::make_pair(R, d);
}
}  // namespace math
}  // namespace drake
