#include "zmpUtil.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;
using namespace std;

MatrixXd expm(const MatrixXd& A) {
  MatrixXd F;
  MatrixExponential<MatrixXd>(A).compute(F);
  return F;
}

ExponentialPlusPiecewisePolynomial<double> s1Trajectory(const TVLQRData &sys, const PiecewisePolynomial<double> &zmp_trajectory,const Ref<const MatrixXd> &S) {
  size_t n = static_cast<size_t>(zmp_trajectory.getNumberOfSegments());
  int d = zmp_trajectory.getSegmentPolynomialDegree(0);
  int k = d + 1;

  for (size_t i = 1; i < n; i++) {
    assert(zmp_trajectory.getSegmentPolynomialDegree(i) == d);
  }

  VectorXd dt(n);
  std::vector<double> breaks = zmp_trajectory.getSegmentTimes();

  for (size_t i = 0; i < n; i++) {
    dt(i) = breaks[i + 1] - breaks[i];
  }  
  
  MatrixXd zmp_tf = zmp_trajectory.value(zmp_trajectory.getEndTime());
  PiecewisePolynomial<double> zbar_pp = zmp_trajectory - zmp_tf;

  Matrix2d R1i = sys.R1.inverse();
  MatrixXd NB = sys.N.transpose() + sys.B.transpose() * S; //2 x 4
  Matrix4d A2 = NB.transpose() * R1i * sys.B.transpose() - sys.A.transpose();
  MatrixXd B2 = 2 * (sys.C.transpose() - NB.transpose() * R1i * sys.D) * sys.Qy; //4 x 2
  Matrix4d A2i = A2.inverse();

  MatrixXd alpha = MatrixXd::Zero(4, n);

  vector<MatrixXd> beta;
  VectorXd s1dt;
  
  for (size_t i = 0; i < n ; i++) {
    beta.push_back(MatrixXd::Zero(4, k));
  }

  for (int j = static_cast<int>(n) - 1; j >= 0; j--) { 

    auto poly_mat = zbar_pp.getPolynomialMatrix(j);
    size_t nq = poly_mat.rows();
    MatrixXd poly_coeffs = MatrixXd::Zero(nq, k);

    for (size_t x = 0; x < nq; x++) {
      poly_coeffs.row(x) = poly_mat(x).getCoefficients().transpose();
    }    
    
    beta[j].col(k - 1) = -A2i * B2 * poly_coeffs.col(k - 1);
    
    for (int i = k - 2; i >= 0; i--) {
      beta[j].col(i) = A2i * ((i+1) * beta[j].col(i + 1) - B2 * poly_coeffs.col(i));
    }
    
    if (j == n - 1) {
      s1dt = VectorXd::Zero(4);
    } else {
      s1dt = alpha.col(j+1) + beta[j + 1].col(0);
    }

    VectorXd dtpow(k);
    for (size_t p = 0; p < k; p++) { 
      dtpow(p) = pow(dt(j), static_cast<int>(p));
    }
    
    alpha.col(j) = expm(A2*dt(j)).inverse() * (s1dt - beta[j]*dtpow);
  }
  
  vector<PiecewisePolynomial<double>::PolynomialMatrix> polynomial_matrices;
  for (int segment = 0; segment < n ; segment++) {
    PiecewisePolynomial<double>::PolynomialMatrix polynomial_matrix(4, 1);
    for(int row = 0; row < 4; row++) {
      polynomial_matrix(row) = Polynomial<double>(beta[segment].row(row));
    }
    polynomial_matrices.push_back(polynomial_matrix);
  }

  PiecewisePolynomial<double> pp_part = PiecewisePolynomial<double>(polynomial_matrices, breaks);
  auto s1traj = ExponentialPlusPiecewisePolynomial<double>(Matrix4d::Identity(), A2, alpha, pp_part);
  return s1traj;
}
