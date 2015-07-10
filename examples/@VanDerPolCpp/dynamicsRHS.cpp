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

Matrix<Polynomiald, Dynamic, Dynamic> msspolyToEigen(const mxArray* msspoly)
{
  auto dim = matlabToEigenMap(mxGetPropertySafe(msspoly,0,"dim"));
  auto sub = matlabToEigenMap(mxGetPropertySafe(msspoly,0,"sub"));
  auto var = matlabToEigenMap(mxGetPropertySafe(msspoly,0,"var"));
  auto pow = matlabToEigenMap(mxGetPropertySafe(msspoly,0,"pow"));
  auto coeff = matlabToEigenMap(mxGetPropertySafe(msspoly,0,"coeff"));
  
/*
  Matrix<Polynomiald, Dynamic, Dynamic> poly((int)dim(0),(int)dim(1));

  for (int i=0; i<sub.rows(); i++) {
    vector<VarType> vars;    
    vector<PowerType> powers;
 * for (int j=0; j<var.cols(); j++) {
      (var.cols()) = var.row(i);
     = pow.row(i);
    Polynomiald p(vars,coeff.row(i),powers);
    poly(sub(i,0),sub(i,1)) += p;
  }

  cout << poly << endl;
 */
  
  return poly;
}

template< int Rows, int Cols >
mxArray* eigenToMSSPoly(Matrix<Polynomiald,Rows,Cols> & poly)
{
  /*
  int nrhs = 3 + num_additional_inputs;
  mxArray *plhs[1];
  mxArray **prhs;  
  prhs = new mxArray*[nrhs];  mexCallMATLABSafe(
   */
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
  } else if (isa(prhs[2],"msspoly")) {
    auto x = msspolyToEigen(prhs[2]);
//    dynamicsRHS(x,xdot);
//    plhs[0] = eigenToMSSPoly(xdot);
    plhs[0] = mxCreateDoubleMatrix(2,1,mxREAL);
  } else {
    mexErrMsgIdAndTxt("Drake:VanDerPolCpp:UnknownType","don't know how to handle the datatype passed in for x (yet)");
  }
}