#include "mex.h"
#include <iostream>

#include "drake/drakeUtil.h"

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    const size_t A_rows = mxGetM(prhs[0]);
    const size_t A_cols = mxGetN(prhs[0]);

    const size_t B_rows = mxGetM(prhs[1]);
    const size_t B_cols = mxGetN(prhs[1]);

    const size_t Q_rows = mxGetM(prhs[2]);
    const size_t Q_cols = mxGetN(prhs[2]);

    const size_t R_rows = mxGetM(prhs[3]);
    const size_t R_cols = mxGetN(prhs[3]);

    assert(A_rows == A_cols);
    assert(Q_rows == Q_cols);
    assert(Q_rows == A_rows);
    assert(R_rows == R_cols);
    assert(B_rows == R_rows);
    assert(B_cols == A_cols);

    Map<MatrixXd> A(mxGetPr(prhs[0]), A_rows, A_cols);
    Map<MatrixXd> B(mxGetPr(prhs[1]), B_rows, B_cols);
    Map<MatrixXd> Q(mxGetPr(prhs[2]), Q_rows, Q_cols);
    Map<MatrixXd> R(mxGetPr(prhs[3]), R_rows,R_cols);

    plhs[0] = mxCreateDoubleMatrix(1, A_cols, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(A_rows, A_cols, mxREAL);

    Map<MatrixXd> K(mxGetPr(plhs[0]), 1, A_cols);
    Map<MatrixXd> S(mxGetPr(plhs[1]), A_rows, A_cols);

    lqr(A, B, Q, R, K, S);

}
