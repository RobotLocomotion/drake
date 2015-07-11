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
  
  assert(sub.rows()==var.rows());
  assert(sub.rows()==pow.rows());
  assert(sub.rows()==coeff.rows());
  assert(var.cols()==pow.cols());

  Matrix<Polynomiald, Dynamic, Dynamic> poly((int)dim(0),(int)dim(1));
  for (int i=0; i<sub.rows(); i++) {
    vector<Polynomiald::VarType> vars;    
    vector<Polynomiald::PowerType> powers;
    int j=0;
    while (j<var.cols() && var(i,j)>0) {
      vars.push_back(var(i,j));
      powers.push_back(pow(i,j));
      j++;
    }
    Polynomiald p(coeff(i),vars,powers);
    poly(sub(i,0)-1,sub(i,1)-1) += p;
  }

  cout << poly << endl;
  
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
    Matrix< Polynomiald, 2, 1> xdot;
    dynamicsRHS(x,xdot);
    cout << xdot << endl;
//    plhs[0] = eigenToMSSPoly(xdot);
    plhs[0] = mxCreateDoubleMatrix(2,1,mxREAL);
  } else {
    mexErrMsgIdAndTxt("Drake:VanDerPolCpp:UnknownType","don't know how to handle the datatype passed in for x (yet)");
  }
}
