#include "mex.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "drakeMexUtil.h"
#include <drakeGradientUtil.h>

using namespace Eigen;
using namespace std;

/*
 * c++ version of
 *     function [H,C,B] = manipulatorDynamics(obj,q,qd)
 */

template<typename DerivedQ>
void manipulatorDynamics(const mxArray *pobj, const MatrixBase<DerivedQ>& q, const MatrixBase<DerivedQ>& qd,
                         mxArray **plhs)
{
  // keep it readable:
  double m1 = mxGetScalar(mxGetProperty(pobj, 0, "m1"));
  double m2 = mxGetScalar(mxGetProperty(pobj, 0, "m2"));
  double l1 = mxGetScalar(mxGetProperty(pobj, 0, "l1"));
  double g = mxGetScalar(mxGetProperty(pobj, 0, "g"));
  double lc1 = mxGetScalar(mxGetProperty(pobj, 0, "lc1"));
  double lc2 = mxGetScalar(mxGetProperty(pobj, 0, "lc2"));
  double b1 = mxGetScalar(mxGetProperty(pobj, 0, "b1"));
  double b2 = mxGetScalar(mxGetProperty(pobj, 0, "b2"));
  double I1 = mxGetScalar(mxGetProperty(pobj, 0, "Ic1")) + m1 * lc1 * lc1;
  double I2 = mxGetScalar(mxGetProperty(pobj, 0, "Ic2")) + m2 * lc2 * lc2;
  double m2l1lc2 = m2 * l1 * lc2;  // occurs often!

  auto c2 = cos(q(1));
  auto s1 = sin(q(0)), s2 = sin(q(1));
  auto s12 = sin(q(0) + q(1));

  auto h12 = I2 + m2l1lc2 * c2;

  typedef typename DerivedQ::Scalar Scalar;
  Matrix<Scalar, 2, 2> H;
  Matrix<Scalar, 2, 1> C;
  Matrix<Scalar, 2, 1> B;

  H << I1 + I2 + m2 * l1 * l1 + 2 * m2l1lc2 * c2, h12, h12, I2;

  //C = [ -2*m2l1lc2*s(2)*qd(2), -m2l1lc2*s(2)*qd(2); m2l1lc2*s(2)*qd(1), 0 ];
  //G = g*[ m1*lc1*s(1) + m2*(l1*s(1)+lc2*s12); m2*lc2*s12 ];

  C << -2 * m2l1lc2 * s2 * qd(1) * qd(0) + -m2l1lc2 * s2 * qd(1) * qd(1), m2l1lc2 * s2 * qd(0) * qd(0);

  // add in G terms
  C(0) += g * m1 * lc1 * s1 + g * m2 * (l1 * s1 + lc2 * s12);
  C(1) += g * m2 * lc2 * s12;

  // damping terms
  C(0) += b1 * qd(0);
  C(1) += b2 * qd(1);

  // input matrix
  // multiplying by q(0) just to get the size of the derivatives vector right; Matlab TaylorVar operations will complain otherwise. We could handle this case on the Matlab side instead.
  B << 0.0 * q(0), 1.0;

  plhs[0] = eigenToMatlabGeneral<2, 2>(H);
  plhs[1] = eigenToMatlabGeneral<2, 1>(C);
  plhs[2] = eigenToMatlabGeneral<2, 1>(B);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  const mxArray *pobj = prhs[0];

  if (mxIsDouble(prhs[1]) && mxIsDouble(prhs[2])) {
    auto q = matlabToEigenMap<2, 1>(prhs[1]);
    auto qd = matlabToEigenMap<2, 1>(prhs[2]);
    manipulatorDynamics(pobj, q, qd, plhs);
  } else if (isa(prhs[1], "TrigPoly") && isa(prhs[2], "TrigPoly")) {
    auto q = trigPolyToEigen(prhs[1]);
    auto qd = trigPolyToEigen(prhs[2]);
    manipulatorDynamics(pobj, q, qd, plhs);
  } else if (isa(prhs[1], "TaylorVar") && isa(prhs[2], "TaylorVar")) {
    auto q = taylorVarToEigen<Dynamic, 1>(prhs[1]);
    auto qd = taylorVarToEigen<Dynamic, 1>(prhs[2]);
    manipulatorDynamics(pobj, q, qd, plhs);
  } else {
    mexErrMsgIdAndTxt("Drake:AcrobotPLantCpp:UnknownType",
                      "don't know how to handle the datatypes passed in for q and/or qd (yet)");
  }
}


