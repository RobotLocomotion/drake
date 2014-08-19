#include "../drakeGeometryUtil.h"
#include "mex.h"
#include "testUtil.h"
using namespace Eigen;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nrhs != 6 || nlhs != 7) {
    mexErrMsgIdAndTxt("Drake:testGeometryGradientsmex:BadInputs","Usage [dT, dTInv, dAdT, dAdT_transpose, x_norm, dx_norm, ddx_norm] = testGeometryGradientsmex(T, S, qdot_to_v, X, dX, x)");
  }

  int argnum = 0;
  Isometry3d T;
  memcpy(T.data(),mxGetPr(prhs[argnum++]), sizeof(double)*HOMOGENEOUS_TRANSFORM_SIZE);
  auto S = matlabToEigen<TWIST_SIZE>(prhs[argnum++]);
  auto qdot_to_v = matlabToEigen<>(prhs[argnum++]);
  auto X = matlabToEigen<TWIST_SIZE>(prhs[argnum++]);
  auto dX = matlabToEigen<>(prhs[argnum++]);
  auto x = matlabToEigen<4, 1>(prhs[argnum++]);

  auto dT = dHomogTrans(T, S, qdot_to_v).eval();
  auto dTInv = dHomogTransInv(T, dT).eval();
  auto dAdT = dTransformAdjoint(T, X, dT, dX).eval();
  auto dAdT_transpose = dTransformAdjointTranspose(T, X, dT, dX).eval();

  Vector4d x_norm;
  Gradient<Vector4d, 4, 1>::type dx_norm;
  Gradient<Vector4d, 4, 2>::type ddx_norm;
  normalizeVec(x, x_norm, &dx_norm, &ddx_norm);

  int outnum = 0;
  plhs[outnum++] = eigenToMatlab(dT);
  plhs[outnum++] = eigenToMatlab(dTInv);
  plhs[outnum++] = eigenToMatlab(dAdT);
  plhs[outnum++] = eigenToMatlab(dAdT_transpose);
  plhs[outnum++] = eigenToMatlab(x_norm);
  plhs[outnum++] = eigenToMatlab(dx_norm);
  plhs[outnum++] = eigenToMatlab(ddx_norm);
}
