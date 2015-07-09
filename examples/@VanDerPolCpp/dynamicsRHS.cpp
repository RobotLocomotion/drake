#include "mex.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "drakeUtil.h"
#include "Polynomial.h"

using namespace Eigen;
using namespace std;
/*
 * c++ version of
 *     function xdot = dynamicsRHS(~,~,x,~)
 */

template< int Rows, int Cols >
void msspolyToEigen(const mxArray* msspoly, Matrix<Polynomial, Rows, Cols> & poly)
{}

template< int Rows, int Cols >
mxArray* eigenToMSSPoly(Matrix<Polynomial,Rows,Cols> & poly, string name)
{
  int nrhs = 3 + num_additional_inputs;
  mxArray *plhs[1];
  mxArray **prhs;  
  prhs = new mxArray*[nrhs];  mexCallMATLABSafe(
}

template <typename DerivedA, typename DerivedB>
void dynamicsRHS(const MatrixBase<DerivedA> &x, MatrixBase<DerivedB> &xdot) 
{
  xdot << x(1), -x(0)-x(1)*(x(0)*x(0)-1);
}        
        
void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) 
{
  if (mxIsDouble(prhs[2])) {
    Map<Vector2d> x(mxGetPrSafe(prhs[2]));
    plhs[0] = mxCreateDoubleMatrix(2,1,mxREAL);
    Map<Vector2d> xdot(mxGetPr(plhs[0]));
    dynamicsRHS(x,xdot);
  } elseif (isa(prhs[2],"msspoly")) {
    Matrix < Polynomial, 2, 1 > x, xdot;
    msspolyToEigen(prhs[2],x);
    dynamicsRHS(x,xdot);
    plhs[0] = eigenToMSSPoly(xdot);
  } else {
    mexErrMsgIdAndTxt("Drake:VanDerPolCpp:UnknownType","don't know how to handle the datatype passed in for x (yet)");
  }
}