#include "drake/math/quadratic_form.h"

#include <iostream>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

namespace drake {
namespace math {
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> DecomposePositiveQuadraticForm(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& b, double c, double tol) {
  if (Q.rows() != Q.cols()) {
    throw std::runtime_error("Q should be a square matrix.");
  }
  if (b.rows() != Q.rows()) {
    throw std::runtime_error("b is not in the right size.");
  }
  if (tol < 0) {
    throw std::runtime_error("tol should be non-negative.");
  }
  // The quadratic form xᵀQx + bᵀx + c can also be written as
  // [x]ᵀ * [Q   b/2] * [x]
  // [1]    [b/2   c]   [1]
  // We will call the matrix in the middle as M
  Eigen::MatrixXd M(Q.rows() + 1, Q.rows() + 1);
  M.block(0, 0, Q.rows(), Q.cols()) = (Q + Q.transpose()) / 2;
  M.block(0, Q.cols(), Q.rows(), 1) = b / 2;
  M.block(Q.rows(), 0, 1, Q.cols()) = b.transpose() / 2;
  M(Q.rows(), Q.cols()) = c;

  Eigen::MatrixXd R;
  Eigen::MatrixXd d;
  // M should be a positive semidefinite matrix, to make sure that the quadratic
  // form xᵀQx + bᵀx + c is always non-negative.
  // First do a Cholesky decomposition of M, as M = Uᵀ*U. If Cholesky fails,
  // then
  // M is not positive definite.
  Eigen::LLT<Eigen::MatrixXd> llt(M);
  if (llt.info() == Eigen::Success) {
    // U is an upper diagonal matrix.
    // U = [U1 u2]
    //     [0  u3]
    Eigen::MatrixXd U = llt.matrixU();
    R = U.block(0, 0, Q.rows() + 1, Q.cols());
    d = U.col(Q.cols());
    return std::make_pair(R, d);
  }
  // M should be positive semidefinite.
  // Ideally I should use robust Cholesky decomposition to decompose M as
  // Aᵀ * A. Unfortunately there is some bug in Eigen's LDLT code, see
  // http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1479
  // So I use Eigen value decomposition here.
  // TODO(hongkai.dai): switch to Cholesky decomposition once
  // http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1479 is fixed.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(M);
  if (es.info() == Eigen::Success && (es.eigenvalues().array() >= -tol).all()) {
    Eigen::MatrixXd eigen_vectors(es.eigenvalues().rows(), 2);
    eigen_vectors.col(0) = es.eigenvalues();
    eigen_vectors.col(1) = Eigen::VectorXd::Zero(es.eigenvalues().rows());
    Eigen::MatrixXd A = eigen_vectors.rowwise()
                            .maxCoeff()
                            .array()
                            .sqrt()
                            .matrix()
                            .asDiagonal() *
                        es.eigenvectors().transpose();
    R = A.block(0, 0, Q.rows() + 1, Q.cols());
    d = A.col(Q.cols());
    return std::make_pair(R, d);
  }
  /* Call robust Cholesky decomposition to decompose M into Aᵀ * A.
  // M should be positive semidefinite, so LDLT should be successful.
  Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
  if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
    // M = PᵀLDLᵀP
    // First write M = Aᵀ * A
    // TODO(hongkai.dai) The right bottom corner of U is 0, so we should remove
    // this block.
    Eigen::MatrixXd A =
        ldlt.matrixU() * (ldlt.transpositionsP() *
                          Eigen::MatrixXd::Identity(M.rows(), M.cols()));
    A = ldlt.vectorD().array().real().sqrt().matrix().asDiagonal() * A;
    R = A.block(0, 0, Q.rows() + 1, Q.cols());
    d = A.col(Q.cols());
    return std::make_pair(R, d);
  }*/
  throw std::runtime_error("The quadratic form is not positive semidefinite.");
}
}  // namespace math
}  // namespace drake
