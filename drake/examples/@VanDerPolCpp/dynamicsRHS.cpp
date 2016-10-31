#include <mex.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "drake/common/polynomial.h"
#include "drake/matlab/util/drakeMexUtil.h"

using namespace Eigen;
using namespace std;
/*
 * c++ version of
 *     function xdot = dynamicsRHS(~,~,x,~)
 */

template <typename DerivedA, typename DerivedB>
void dynamicsRHS(
    const MatrixBase<DerivedA>& x,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    MatrixBase<DerivedB>& xdot) {
  xdot << x(1), -x(0) - x(1) * (x(0) * x(0) - 1);
}

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (mxIsDouble(prhs[2])) {
    Map<Vector2d> x(mxGetPrSafe(prhs[2]));
    plhs[0] = mxCreateDoubleMatrix(2, 1, mxREAL);
    Map<Vector2d> xdot(mxGetPr(plhs[0]));
    dynamicsRHS(x, xdot);
  } else if (isa(prhs[2], "msspoly")) {
    auto x = msspolyToEigen(prhs[2]);
    Matrix<Polynomiald, 2, 1> xdot;
    dynamicsRHS(x, xdot);
    //    cout << xdot << endl;
    plhs[0] = eigenToMSSPoly<2, 1>(xdot);
    //    plhs[0] = mxCreateDoubleMatrix(2, 1, mxREAL);
  } else {
    mexErrMsgIdAndTxt(
        "Drake:VanDerPolCpp:UnknownType",
        "don't know how to handle the datatype passed in for x (yet)");
  }
}
