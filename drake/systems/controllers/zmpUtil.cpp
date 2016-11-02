#include "drake/systems/controllers/zmpUtil.h"

#include <vector>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "drake/common/drake_assert.h"

using Eigen::Index;
using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::VectorXd;
using std::vector;

ExponentialPlusPiecewisePolynomial<double> s1Trajectory(
    const TVLQRData& sys, const PiecewisePolynomial<double>& zmp_trajectory,
    const Ref<const MatrixXd>& S) {
  size_t n = static_cast<size_t>(zmp_trajectory.getNumberOfSegments());
  int d = zmp_trajectory.getSegmentPolynomialDegree(0);
  int k = d + 1;

  for (size_t i = 1; i < n; i++) {
    DRAKE_ASSERT(zmp_trajectory.getSegmentPolynomialDegree(i) == d);
  }

  VectorXd dt(n);
  std::vector<double> breaks = zmp_trajectory.getSegmentTimes();

  for (size_t i = 0; i < n; i++) {
    dt(i) = breaks[i + 1] - breaks[i];
  }

  MatrixXd zmp_tf = zmp_trajectory.value(zmp_trajectory.getEndTime());
  PiecewisePolynomial<double> zbar_pp = zmp_trajectory - zmp_tf;

  Matrix2d R1i = sys.R1.inverse();
  MatrixXd NB = sys.N.transpose() + sys.B.transpose() * S;  // 2 x 4
  Matrix4d A2 = NB.transpose() * R1i * sys.B.transpose() - sys.A.transpose();
  MatrixXd B2 =
      2 * (sys.C.transpose() - NB.transpose() * R1i * sys.D) * sys.Qy;  // 4 x 2
  Matrix4d A2i = A2.inverse();

  MatrixXd alpha = MatrixXd::Zero(4, n);

  vector<MatrixXd> beta;
  VectorXd s1dt;

  for (size_t i = 0; i < n; i++) {
    beta.push_back(MatrixXd::Zero(4, k));
  }

  for (int j = static_cast<int>(n) - 1; j >= 0; j--) {
    auto poly_mat = zbar_pp.getPolynomialMatrix(j);
    size_t nq = poly_mat.rows();
    MatrixXd poly_coeffs = MatrixXd::Zero(nq, k);

    for (size_t x = 0; x < nq; x++) {
      auto element_coeffs = poly_mat(x).GetCoefficients();

      // Note that the number of coefficients of poly_mat(x) may not be k due to
      // erasure of zero coefficients. It should always be less than or equal to
      // k though. See #2165.
      DRAKE_ASSERT(poly_coeffs.cols() >= element_coeffs.size());
      poly_coeffs.row(x).setZero();
      for (Index i = 0; i < element_coeffs.size(); i++) {
        poly_coeffs.row(x)(i) = element_coeffs(i);
      }
    }

    beta[j].col(k - 1) = -A2i * B2 * poly_coeffs.col(k - 1);

    for (int i = k - 2; i >= 0; i--) {
      beta[j].col(i) =
          A2i * ((i + 1) * beta[j].col(i + 1) - B2 * poly_coeffs.col(i));
    }

    if (j == static_cast<int>(n) - 1) {
      s1dt = VectorXd::Zero(4);
    } else {
      s1dt = alpha.col(j + 1) + beta[j + 1].col(0);
    }

    VectorXd dtpow(k);
    for (int p = 0; p < k; p++) {
      dtpow(p) = pow(dt(j), p);
    }

    alpha.col(j) =
        (A2 * dt(j)).eval().exp().inverse() * (s1dt - beta[j] * dtpow);
  }

  vector<PiecewisePolynomial<double>::PolynomialMatrix> polynomial_matrices;
  for (size_t segment = 0; segment < n; segment++) {
    PiecewisePolynomial<double>::PolynomialMatrix polynomial_matrix(4, 1);
    for (int row = 0; row < 4; row++) {
      polynomial_matrix(row) = Polynomial<double>(beta[segment].row(row));
    }
    polynomial_matrices.push_back(polynomial_matrix);
  }

  PiecewisePolynomial<double> pp_part =
      PiecewisePolynomial<double>(polynomial_matrices, breaks);
  auto s1traj = ExponentialPlusPiecewisePolynomial<double>(Matrix4d::Identity(),
                                                           A2, alpha, pp_part);
  return s1traj;
}
