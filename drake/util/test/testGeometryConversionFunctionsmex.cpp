#include "drakeGeometryUtil.h"
#include "drakeMexUtil.h"
#include "mex.h"
using namespace Eigen;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nrhs != 2 || nlhs != 12) {
    mexErrMsgIdAndTxt("Drake:testGeometryConversionFunctionsmex:BadInputs","Usage [omega2qd, domega2qd, omega2rpyd, domega2rpyd, ddomega2rpyd, rpyd2omega, drpyd2omega,qd2omega, dqd2omega, dq2R, drpydR, dqdR] = testGeometryConversionFunctionsmex(q, dq)");
  }

  int argnum = 0;
  Isometry3d T;
  auto q = matlabToEigen<QUAT_SIZE, 1>(prhs[argnum++]);
  auto dq = matlabToEigen<QUAT_SIZE, Eigen::Dynamic>(prhs[argnum++]);

  auto rpy = quat2rpy(q);

  Matrix<double, QUAT_SIZE, SPACE_DIMENSION> omega2qd;
  Gradient<Matrix<double, QUAT_SIZE, SPACE_DIMENSION>, QUAT_SIZE, 1>::type domega2qd;
  Matrix<double, RPY_SIZE, SPACE_DIMENSION> omega2rpyd;
  Gradient<Matrix<double, RPY_SIZE, SPACE_DIMENSION>, RPY_SIZE, 1>::type domega2rpyd;
  Gradient<Matrix<double, RPY_SIZE, SPACE_DIMENSION>, RPY_SIZE, 2>::type ddomega2rpyd;
  Matrix<double, SPACE_DIMENSION, QUAT_SIZE> qd2omega;
  Gradient<Matrix<double, SPACE_DIMENSION, QUAT_SIZE>, QUAT_SIZE, 1>::type dqd2omega;

  angularvel2quatdotMatrix(q, omega2qd, &domega2qd);
  angularvel2rpydotMatrix(rpy, omega2rpyd, &domega2rpyd, &ddomega2rpyd);
  Matrix<double, SPACE_DIMENSION, RPY_SIZE> rpyd2omega;
  Gradient<Matrix<double,SPACE_DIMENSION,RPY_SIZE>,RPY_SIZE,1>::type drpyd2omega;
  rpydot2angularvelMatrix(rpy,rpyd2omega,&drpyd2omega);
  quatdot2angularvelMatrix(q, qd2omega, &dqd2omega);
  auto R = quat2rotmat(q);
  auto dq2R = dquat2rotmat(q);
  Matrix<double, RotmatSize, Dynamic> dR = dq2R * dq;
  auto drpydR = drotmat2rpy(R, dR);
  auto dqdR = drotmat2quat(R, dR);

  int outnum = 0;
  plhs[outnum++] = eigenToMatlab(omega2qd);
  plhs[outnum++] = eigenToMatlab(domega2qd);
  plhs[outnum++] = eigenToMatlab(omega2rpyd);
  plhs[outnum++] = eigenToMatlab(domega2rpyd);
  plhs[outnum++] = eigenToMatlab(ddomega2rpyd);
  plhs[outnum++] = eigenToMatlab(rpyd2omega);
  plhs[outnum++] = eigenToMatlab(drpyd2omega);
  plhs[outnum++] = eigenToMatlab(qd2omega);
  plhs[outnum++] = eigenToMatlab(dqd2omega);
  plhs[outnum++] = eigenToMatlab(dq2R);
  plhs[outnum++] = eigenToMatlab(drpydR);
  plhs[outnum++] = eigenToMatlab(dqdR);
}
