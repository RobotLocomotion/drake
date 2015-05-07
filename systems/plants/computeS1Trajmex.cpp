#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "drake/ExponentialPlusPiecewisePolynomial.h"
#include "math.h"
#include <vector>

using namespace Eigen;
using namespace std;

MatrixXd expm(const MatrixXd& A) {
  MatrixXd F;
  MatrixExponential<MatrixXd>(A).compute(F);
  return F;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
  PiecewisePolynomial<double> pp; //this needs to come in as input
  double h_over_g;
  size_t n;
  size_t k;
  MatrixXd zmp_tf;
  PiecewisePolynomial<double> zbar_pp = pp - zmp_tf;
  
  Matrix4d A;
  MatrixXd B(4, 2);
  MatrixXd C(2, 4);
  Matrix4d S;
  Matrix2d D = -h_over_g * MatrixXd::Identity(2, 2);
  Matrix2d Q = MatrixXd::Identity(2, 2);
  Matrix4d Q1 = C.transpose() * Q * C;
  Matrix2d R = MatrixXd::Zero(2, 2);

  Matrix2d R1 = R + D.transpose() * Q * D;
  MatrixXd N = C.transpose() * Q * D; //4 x 2

  A << MatrixXd::Zero(2, 2), MatrixXd::Identity(2, 2), MatrixXd::Zero(2, 4);
  B << MatrixXd::Zero(2, 2), MatrixXd::Identity(2, 2);
  C << MatrixXd::Identity(2, 2), MatrixXd::Zero(2, 2);

  Matrix2d R1i = R.inverse();

  MatrixXd NB = N.transpose() + B.transpose() * S; //2 x 4
  Matrix4d A2 = NB.transpose() * R1i * B.transpose() - A.transpose();
  MatrixXd B2 = 2 * (C.transpose() - NB.transpose() * R1i * D) * Q; //4 x 2

  Matrix4d A2i = A2.inverse();

  MatrixXd alpha = MatrixXd::Zero(4, n);

  vector<MatrixXd> beta;

  for(size_t i = 0; i < n ; i++) {
    beta.push_back(MatrixXd::Zero(4, k));
  }

  VectorXd s1dt;
  VectorXd dt(n);

  for (size_t j = n - 1; j >= 0; j--) { 
    auto poly_mat = pp.getPolynomialMatrix(j);
    size_t nq = poly_mat.rows();
    MatrixXd poly_coeffs(nq, k);
    for (size_t x = 0; x < nq; x++) {
      poly_coeffs.row(x) = poly_mat(x).getCoefficients().transpose();
    }    
    beta[j].col(k - 1) = -A2i * B2 * poly_coeffs.col(k - 1);

    for (size_t i = k - 2; k >= 0; k--) {
      beta[j].col(i) = A2i * (i * beta[j].col(i + 1) - B2 * poly_coeffs.col(i));
    }

    if(j == n - 1) {
      s1dt = VectorXd::Zero(4);
    } else {
      s1dt = alpha.col(j+1) + beta[j + 1].col(0);
    }
    //alpha(:,j) = expm(A2*dt(j)) \ (s1dt - squeeze(beta(:,j,:))*(dt(j).^(0:k-1)'));
    VectorXd dtpow(k - 1);
    for(size_t p = 0; p < k; p++) { 
      dtpow(p) = pow(dt(j), p);
    }
    alpha.col(j) = expm(A2*dt(j)).inverse() * (s1dt - beta[j]*dtpow);
  }

  //ExpPlusPPTrajectory(breaks,K,A,alpha,gamma)
  //ExponentialPlusPiecewisePolynomial(Matrix4d::Identity)
  //ExponentialPlusPiecewisePolynomial(const Eigen::MatrixBase<DerivedK>& K, const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedAlpha>& alpha, const PiecewisePolynomial<CoefficientType>& piecewise_polynomial_part)
  //s1traj = ExpPlusPPTrajectory(breaks,eye(4),A2,alpha,beta);
  //build ExponentialPlusPiecewisePolynomial with alpha and beta here....

}
