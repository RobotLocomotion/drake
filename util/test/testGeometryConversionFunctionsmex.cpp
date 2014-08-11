#include "../drakeGeometryUtil.h"
#include "mex.h"
#include "testUtil.h"
using namespace Eigen;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nrhs != 2 || nlhs != 9) {
    mexErrMsgIdAndTxt("Drake:testGeometryGradientsmex:BadInputs","Usage [a2q, da2q, a2r, da2r, dda2r, r2a, q2a, dq2a, dq2R] = testGeometryGradientsmex(q, rpy)");
  }

  int argnum = 0;
  Isometry3d T;
  auto q = matlabToEigen<QUAT_SIZE, 1>(prhs[argnum++]);
  auto rpy = matlabToEigen<RPY_SIZE, 1>(prhs[argnum++]);

  Matrix<double, QUAT_SIZE, SPACE_DIMENSION> a2q;
  typename Gradient<Matrix<double, QUAT_SIZE, SPACE_DIMENSION>, QUAT_SIZE, 1>::type da2q;

  Matrix<double, RPY_SIZE, SPACE_DIMENSION> a2r;
  typename Gradient<Matrix<double, RPY_SIZE, SPACE_DIMENSION>, RPY_SIZE, 1>::type da2r;
  typename Gradient<Matrix<double, RPY_SIZE, SPACE_DIMENSION>, RPY_SIZE, 2>::type dda2r;

  Matrix<double, SPACE_DIMENSION, QUAT_SIZE> q2a;
  typename Gradient<Matrix<double, SPACE_DIMENSION, QUAT_SIZE>, QUAT_SIZE, 1>::type dq2a;

  angularvel2quatdotMatrix(q, a2q, &da2q);
  angularvel2rpydotMatrix(rpy, a2r, &da2r, &dda2r);
  auto r2a = rpydot2angularvelMatrix(rpy);
  quatdot2angularvelMatrix(q, q2a, &dq2a);
  auto dq2R = dquat2rotmat(q);

  int outnum = 0;
  plhs[outnum++] = eigenToMatlab(a2q);
  plhs[outnum++] = eigenToMatlab(da2q);
  plhs[outnum++] = eigenToMatlab(a2r);
  plhs[outnum++] = eigenToMatlab(da2r);
  plhs[outnum++] = eigenToMatlab(dda2r);
  plhs[outnum++] = eigenToMatlab(r2a);
  plhs[outnum++] = eigenToMatlab(q2a);
  plhs[outnum++] = eigenToMatlab(dq2a);
  plhs[outnum++] = eigenToMatlab(dq2R);
}
