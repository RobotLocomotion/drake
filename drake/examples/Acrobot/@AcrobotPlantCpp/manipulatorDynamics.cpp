#include "mex.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "drakeMexUtil.h"
#include <unsupported/Eigen/AutoDiff>
#include <drakeGradientUtil.h>

using namespace Eigen;
using namespace std;
/*
 * c++ version of
 *     function [H,C,B] = manipulatorDynamics(obj,q,qd)
 */

template <typename DerivedA, typename DerivedB, typename DerivedC>
void manipulatorDynamics(const mxArray* pobj, const MatrixBase<DerivedA> &q, const MatrixBase<DerivedA> &qd, MatrixBase<DerivedB> &H, MatrixBase<DerivedC> &C, MatrixBase<DerivedC> &B)
{
  // keep it readable:
  double m1  = mxGetScalar(mxGetProperty(pobj,0,"m1"));
  double m2  = mxGetScalar(mxGetProperty(pobj,0,"m2"));
  double l1  = mxGetScalar(mxGetProperty(pobj,0,"l1"));
  double g   = mxGetScalar(mxGetProperty(pobj,0,"g"));
  double lc1 = mxGetScalar(mxGetProperty(pobj,0,"lc1"));
  double lc2 = mxGetScalar(mxGetProperty(pobj,0,"lc2"));
  double b1  = mxGetScalar(mxGetProperty(pobj,0,"b1"));
  double b2  = mxGetScalar(mxGetProperty(pobj,0,"b2"));
  double I1  = mxGetScalar(mxGetProperty(pobj,0,"Ic1")) + m1*lc1*lc1;
  double I2  = mxGetScalar(mxGetProperty(pobj,0,"Ic2")) + m2*lc2*lc2;
  double m2l1lc2 = m2*l1*lc2;  // occurs often!

  auto c2 = cos(q(1));
  auto s1 = sin(q(0)), s2 = sin(q(1));
  auto s12 = sin(q(0)+q(1));

  auto h12 = I2 + m2l1lc2*c2;
  H << I1 + I2 + m2*l1*l1 + 2*m2l1lc2*c2, h12, h12, I2;

  //C = [ -2*m2l1lc2*s(2)*qd(2), -m2l1lc2*s(2)*qd(2); m2l1lc2*s(2)*qd(1), 0 ];
  //G = g*[ m1*lc1*s(1) + m2*(l1*s(1)+lc2*s12); m2*lc2*s12 ];

  C << -2*m2l1lc2*s2*qd(1)*qd(0) + -m2l1lc2*s2*qd(1)*qd(1), m2l1lc2*s2*qd(0)*qd(0);

  // add in G terms
  C(0) += g*m1*lc1*s1 + g*m2*(l1*s1+lc2*s12); C(1) += g*m2*lc2*s12;

  // damping terms
  C(0)+=b1*qd(0); C(1)+=b2*qd(1);

  B << 0.0, 1.0;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> autoDiffToValueMatrix(const Eigen::MatrixBase<Derived>& autoDiff) {
  Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> ret(autoDiff.rows(), autoDiff.cols());
  for (int i = 0; i < autoDiff.rows(); i++) {
    for (int j = 0; j < autoDiff.cols(); ++j) {
      ret(i, j) = autoDiff(i, j).value();
    }
  }
  return ret;
};

template<typename Derived>
typename Gradient<Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>, Eigen::Dynamic>::type autoDiffToGradientMatrix(
        const Eigen::MatrixBase<Derived>& autoDiff, int num_variables = Eigen::Dynamic)
{
  int num_variables_from_matrix = 0;
  for (int i = 0; i < autoDiff.size(); ++i) {
    num_variables_from_matrix = std::max(num_variables_from_matrix, static_cast<int>(autoDiff(i).derivatives().size()));
  }
  if (num_variables == Eigen::Dynamic) {
    num_variables = num_variables_from_matrix;
  }
  else if (num_variables_from_matrix != 0 && num_variables_from_matrix != num_variables) {
    std::stringstream buf;
    buf << "Input matrix has derivatives w.r.t " << num_variables_from_matrix << ", variables" << ", whereas num_variables is " << num_variables << ".\n";
    buf << "Either num_variables_from_matrix should be zero, or it should match num_variables";
    throw std::runtime_error(buf.str());
  }

  typename Gradient<Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>, Eigen::Dynamic>::type gradient(autoDiff.size(), num_variables);
  for (int row = 0; row < autoDiff.rows(); row++) {
    for (int col = 0; col < autoDiff.cols(); col++) {
      auto gradient_row = gradient.row(row + col * autoDiff.rows()).transpose();
      if (autoDiff(row, col).derivatives().size() == 0) {
        gradient_row.setZero();
      } else {
        gradient_row = autoDiff(row, col).derivatives();
      }
    }
  }
  return gradient;
}

template<typename DerivedGradient, typename DerivedAutoDiff>
void gradientMatrixToAutoDiff(const Eigen::MatrixBase<DerivedGradient>& gradient, Eigen::MatrixBase<DerivedAutoDiff>& autoDiff)
{
  int nq = gradient.cols();
  for (size_t row = 0; row < autoDiff.rows(); row++) {
    for (size_t col = 0; col < autoDiff.cols(); col++) {
      autoDiff(row, col).derivatives().resize(nq, 1);
      autoDiff(row, col).derivatives() = gradient.row(row + col * autoDiff.rows()).transpose();
    }
  }
}

template<int Rows, int Cols>
Eigen::Matrix<AutoDiffScalar<VectorXd>, Rows, Cols> taylorVarToEigen(const mxArray* taylor_var) {
  auto f = mxGetPropertySafe(taylor_var, "f");
  auto df = mxGetPropertySafe(taylor_var, "df");
  if (mxGetNumberOfElements(df) > 1)
    throw runtime_error("TaylorVars of order higher than 1 currently not supported");
  auto ret = matlabToEigenMap<Rows, Cols>(f).template cast<AutoDiffScalar<VectorXd>>().eval();
  typedef Gradient<decltype(ret), Dynamic> GradientType;
  auto gradient_matrix = matlabToEigenMap<GradientType::type::RowsAtCompileTime, GradientType::type::ColsAtCompileTime>(mxGetCell(df, 0));
  gradientMatrixToAutoDiff(gradient_matrix, ret);

  return ret;
}

template <typename Derived>
mxArray* eigenToTaylorVar(const MatrixBase<Derived>& m, int num_variables = Eigen::Dynamic)
{
  const int nrhs = 2;
  mxArray *prhs[nrhs];
  prhs[0] = eigenToMatlab(autoDiffToValueMatrix(m));
  mwSize dims[] = {1};
  prhs[1] = mxCreateCellArray(1, dims);
  mxArray *plhs[1];
  mxSetCell(prhs[1], 0, eigenToMatlab(autoDiffToGradientMatrix(m, num_variables)));
  mexCallMATLABsafe(1, plhs, nrhs, prhs, "TaylorVar");
  return plhs[0];
}

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] )
{
  const mxArray* pobj = prhs[0];

  if (mxIsDouble(prhs[1]) && mxIsDouble(prhs[2])) {
    auto q = matlabToEigenMap<2,1>(prhs[1]);
    auto qd = matlabToEigenMap<2,1>(prhs[2]);

    plhs[0] = mxCreateDoubleMatrix(2,2,mxREAL);
    Map<Matrix2d> H(mxGetPr(plhs[0]));
    plhs[1] = mxCreateDoubleMatrix(2,1,mxREAL);
    Map<Vector2d> C(mxGetPr(plhs[1]),2);
    plhs[2] = mxCreateDoubleMatrix(2,1,mxREAL);
    Map<Vector2d> B(mxGetPr(plhs[2]),2);

    manipulatorDynamics(pobj,q,qd,H,C,B);
  } else if (isa(prhs[1],"TrigPoly") && isa(prhs[2],"TrigPoly")) {
    auto q = trigPolyToEigen(prhs[1]);
    auto qd = trigPolyToEigen(prhs[2]);
    Matrix< TrigPolyd, 2, 2> H;
    Matrix< TrigPolyd, 2, 1> C;
    Matrix< TrigPolyd, 2, 1> B;

    manipulatorDynamics(pobj,q,qd,H,C,B);

    plhs[0] = eigenToTrigPoly<2,2>(H);
    plhs[1] = eigenToTrigPoly<2,1>(C);
    plhs[2] = eigenToTrigPoly<2,1>(B);
  } else if (isa(prhs[1], "TaylorVar") && isa(prhs[2], "TaylorVar")) {
    typedef AutoDiffScalar<VectorXd> Scalar;
    auto q = taylorVarToEigen<Dynamic, 1>(prhs[1]);
    auto qd = taylorVarToEigen<Dynamic, 1>(prhs[2]);
    Matrix<Scalar, 2, 2> H;
    Matrix<Scalar, 2, 1> C;
    Matrix<Scalar, 2, 1> B;

    manipulatorDynamics(pobj, q, qd, H, C, B);

    int nx = q(0).derivatives().size();
    plhs[0] = eigenToTaylorVar(H, nx);
    plhs[1] = eigenToTaylorVar(C, nx);
    plhs[2] = eigenToTaylorVar(B, nx);

  } else {
    mexErrMsgIdAndTxt("Drake:AcrobotPLantCpp:UnknownType","don't know how to handle the datatypes passed in for q and/or qd (yet)");
  }


}


