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
mxArray* eigenToMSSPoly(const Matrix<Polynomiald,Rows,Cols> & poly)
{
  int num_monomials = 0, max_vars = 0;
  for (int i=0; i<poly.size(); i++) {
    auto monomials = poly(i).getMonomials();
    num_monomials += monomials.size();
    for (vector<Polynomiald::Monomial>::const_iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
      if (iter->vars.size() > max_vars)
        max_vars = iter->vars.size();
    }
  }
  
  Matrix<double,1,2> dim; dim << poly.rows(), poly.cols();
  MatrixXd sub(num_monomials,2);
  MatrixXd var = MatrixXd::Zero(num_monomials,max_vars);
  MatrixXd pow = MatrixXd::Zero(num_monomials,max_vars);
  VectorXd coeff(num_monomials);

  int index=0;
  for (int i=0; i<poly.rows(); i++) {
    for (int j=0; j<poly.cols(); j++) {
      auto monomials = poly(i,j).getMonomials();
      for (vector<Polynomiald::Monomial>::const_iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
        sub(index,0) = i+1;
        sub(index,1) = j+1;
        for (int k=0; k<iter->vars.size(); k++) {
          var(index,k)=(double)iter->vars[k];
          pow(index,k)=(double)iter->powers[k];
        }
        coeff(index) = iter->coefficient;
        index++;
      }
    }
  }

  mxArray* plhs[1];
  mxArray* prhs[5];
  prhs[0] = eigenToMatlab(dim);
  prhs[1] = eigenToMatlab(sub);
  prhs[2] = eigenToMatlab(var);
  prhs[3] = eigenToMatlab(pow);
  prhs[4] = eigenToMatlab(coeff);
  mexCallMATLABsafe(1,plhs,5,prhs,"msspoly");
  return plhs[0];
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
    plhs[0] = eigenToMSSPoly(xdot);
//    plhs[0] = mxCreateDoubleMatrix(2,1,mxREAL);
  } else {
    mexErrMsgIdAndTxt("Drake:VanDerPolCpp:UnknownType","don't know how to handle the datatype passed in for x (yet)");
  }
}
