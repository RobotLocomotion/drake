#include "mex.h"
#include <iostream>
#include "drakeMexUtil.h"
#include "math.h"
#include <vector>
#include "drake/zmpUtil.h"
#include <stdexcept>

using namespace Eigen;
using namespace std;

PiecewisePolynomial<double> matlabPPFormToPiecewisePolynomial(const mxArray* pp)
{
  vector<double> breaks = matlabToStdVector<double>(mxGetFieldSafe(pp, "breaks"));
  size_t num_segments = breaks.size() - 1; // l

  const mxArray* coefs_mex = mxGetFieldSafe(pp, "coefs"); // a d*l x k matrix
  const size_t* coefs_mex_dims = mxGetDimensions(coefs_mex);
  int num_coefs_mex_dims = static_cast<int>(mxGetNumberOfDimensions(coefs_mex));

  size_t number_of_elements = mxGetNumberOfElements(coefs_mex);

  const mxArray* dim_mex = mxGetFieldSafe(pp, "dim");
  int num_dims_mex = static_cast<int>(mxGetNumberOfElements(dim_mex));
  if (num_dims_mex == 0 || num_dims_mex > 2)
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

//func sig: 
//computeS1Trajmex(dZMP.pp, A, B, C, D, Q, R, Q1, R1, N, S);      

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
  auto pp = matlabPPFormToPiecewisePolynomial(prhs[0]);
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

  TVLQRData sys;
  sys.A = A;
  sys.B = B;
  sys.C = C;
  sys.D = D;
  sys.Qy = Q;
  sys.R = R;
  sys.u0 = Vector2d(0, 0);
  sys.Q1 = Q1;
  sys.R1 = R1;
  sys.N = N;

  auto s1traj = s1Trajectory(sys, pp, S);
  //do stuff with s1traj
}
