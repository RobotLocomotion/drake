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

PiecewisePolynomial<double> matlabPPFormToPiecewisePolynomial(const mxArray* pp)
{
  vector<double> breaks = matlabToStdVector<double>(mxGetFieldSafe(pp, "breaks"));
  size_t num_segments = breaks.size() - 1; // l

  const mxArray* coefs_mex = mxGetFieldSafe(pp, "coefs"); // a d*l x k matrix
  const size_t* coefs_mex_dims = mxGetDimensions(coefs_mex);
  int num_coefs_mex_dims = mxGetNumberOfDimensions(coefs_mex);

  size_t number_of_elements = mxGetNumberOfElements(coefs_mex);

  const mxArray* dim_mex = mxGetFieldSafe(pp, "dim");
  int num_dims_mex = mxGetNumberOfElements(dim_mex);
  if (num_dims_mex == 0 | num_dims_mex > 2)
    throw runtime_error("case not handled"); // because PiecewisePolynomial can't currently handle it
  const int num_dims = 2;
  mwSize dims[num_dims];
  for (int i = 0; i < num_dims_mex; i++) {
    dims[i] = static_cast<mwSize>(mxGetPr(dim_mex)[i]);
  }
  for (int i = num_dims_mex; i < num_dims; i++)
    dims[i] = 1;

  size_t product_of_dimensions = dims[0]; // d
  for (int i = 1; i < num_dims; ++i) {
    product_of_dimensions *= dims[i];
  }

  size_t num_coefficients = number_of_elements / (num_segments * product_of_dimensions); // k

  vector<PiecewisePolynomial<double>::PolynomialMatrix> polynomial_matrices;
  polynomial_matrices.reserve(num_segments);
  for (mwSize segment_index = 0; segment_index < num_segments; segment_index++) {
    PiecewisePolynomial<double>::PolynomialMatrix polynomial_matrix(dims[0], dims[1]);
    for (mwSize i = 0; i < product_of_dimensions; i++) {
      VectorXd coefficients(num_coefficients);
      mwSize row = segment_index * product_of_dimensions + i;
      for (mwSize coefficient_index = 0; coefficient_index < num_coefficients; coefficient_index++) {
        mwSize sub[] = {row, num_coefficients - coefficient_index - 1}; // Matlab's reverse coefficient indexing...
        coefficients[coefficient_index] = *(mxGetPr(coefs_mex) + sub2ind(num_coefs_mex_dims, coefs_mex_dims, sub));
      }
      polynomial_matrix(i) = Polynomial<double>(coefficients);
    }
    polynomial_matrices.push_back(polynomial_matrix);
  }

  return PiecewisePolynomial<double>(polynomial_matrices, breaks);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{

  PiecewisePolynomial<double> pp; //this needs to come in as input
  pp = matlabPPFormToPiecewisePolynomial(prhs[0]);
  Map<Matrix4d> A(mxGetPrSafe(prhs[1]));
  Map<MatrixXd> B(mxGetPrSafe(prhs[2]), 4, 2);
  Map<MatrixXd> C(mxGetPrSafe(prhs[3]), 2, 4);
  Map<Matrix2d> D(mxGetPrSafe(prhs[4]));
  Map<Matrix2d> Q(mxGetPrSafe(prhs[5]));
  Map<Matrix2d> R(mxGetPrSafe(prhs[6]));
  Map<Matrix4d> Q1(mxGetPrSafe(prhs[7]));
  Map<Matrix2d> R1(mxGetPrSafe(prhs[8]));
  Map<MatrixXd> N(mxGetPrSafe(prhs[9]), 4, 2);
  Map<Matrix4d> S(mxGetPrSafe(prhs[10]));

  size_t n = static_cast<size_t>(pp.getNumberOfSegments());
  
  int d = pp.getSegmentPolynomialDegree(0);

  for (size_t i = 1; i < n; i++) {
    assert(pp.getSegmentPolynomialDegree(i) == d);
  }

  VectorXd dt(n);

  std::vector<double> breaks = pp.getSegmentTimes();

  for(size_t i = 0; i < n; i++) {
    dt(i) = breaks[i + 1] - breaks[i];
  }

  size_t k = d + 1;
  
  MatrixXd zmp_tf = pp.value(pp.getEndTime());

  
  PiecewisePolynomial<double> zbar_pp = pp - zmp_tf;

  Matrix2d R1i = R1.inverse();

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
  for (int j = n - 1; j >= 0; j--) { 
    
    auto poly_mat = zbar_pp.getPolynomialMatrix(j);
    size_t nq = poly_mat.rows();
    MatrixXd poly_coeffs = MatrixXd::Zero(nq, k);

    for (size_t x = 0; x < nq; x++) {
      poly_coeffs.row(x) = poly_mat(x).getCoefficients().transpose();
    }    
    
    beta[j].col(k - 1) = -A2i * B2 * poly_coeffs.col(k - 1); //correct
    
    for (int i = k - 2; i >= 0; i--) {
      beta[j].col(i) = A2i * ((i+1) * beta[j].col(i + 1) - B2 * poly_coeffs.col(i)); //correct
    }
    
    if(j == n - 1) {
      s1dt = VectorXd::Zero(4);
    } else {
      s1dt = alpha.col(j+1) + beta[j + 1].col(0);
    }

    VectorXd dtpow(k - 1);
    for(size_t p = 0; p < k; p++) { 
      dtpow(p) = pow(dt(j), p);
    }

    alpha.col(j) = expm(A2*dt(j)).inverse() * (s1dt - beta[j]*dtpow);
  }

  cout << alpha << endl;

  return;

  //matlabPPFormToPiecewisePolynomial

  //const mxArray* mex_breaks = mxGetFieldOrPropertySafe(array, "breaks");
  //const mxArray* mex_coefs = mxGetFieldOrPropertySafe(array, "gamma");
  //PiecewisePolynomial<double> piecewise_polynomial_part = matlabCoefsAndBreaksToPiecewisePolynomial(mex_coefs, mex_breaks, false);
  
  //ExpPlusPPTrajectory(breaks,K,A,alpha,gamma)
  //ExponentialPlusPiecewisePolynomial(Matrix4d::Identity)
  //ExponentialPlusPiecewisePolynomial(const Eigen::MatrixBase<DerivedK>& K, const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedAlpha>& alpha, const PiecewisePolynomial<CoefficientType>& piecewise_polynomial_part)
  //s1traj = ExpPlusPPTrajectory(breaks,eye(4),A2,alpha,beta);
  //build ExponentialPlusPiecewisePolynomial with alpha and beta here....

}
