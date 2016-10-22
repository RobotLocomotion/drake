#include <mex.h>

#include "drake/common/constants.h"
#include "drake/common/eigen_types.h"
#include "drake/math/normalize_vector.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/matlab/util/drakeMexUtil.h"

using Eigen::Isometry3d;
using Eigen::Vector4d;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nrhs != 6 || nlhs != 7) {
    mexErrMsgIdAndTxt("Drake:testGeometryGradientsmex:BadInputs",
                      "Usage [dT, dTInv, dAdT, dAdT_transpose, x_norm, "
                      "dx_norm, ddx_norm] = testGeometryGradientsmex(T, S, "
                      "qdot_to_v, X, dX, x)");
  }

  int argnum = 0;
  Isometry3d T;
  memcpy(T.data(), mxGetPr(prhs[argnum++]),
         sizeof(double) * drake::kHomogeneousTransformSize);
  auto S = matlabToEigen<drake::kTwistSize, Eigen::Dynamic>(prhs[argnum++]);
  auto qdot_to_v =
      matlabToEigen<Eigen::Dynamic, Eigen::Dynamic>(prhs[argnum++]);
  auto X = matlabToEigen<drake::kTwistSize, Eigen::Dynamic>(prhs[argnum++]);
  auto dX = matlabToEigen<Eigen::Dynamic, Eigen::Dynamic>(prhs[argnum++]);
  auto x = matlabToEigen<4, 1>(prhs[argnum++]);

  auto dT = dHomogTrans(T, S, qdot_to_v).eval();
  auto dTInv = dHomogTransInv(T, dT).eval();
  auto dAdT = dTransformSpatialMotion(T, X, dT, dX).eval();
  auto dAdTInv_transpose = dTransformSpatialForce(T, X, dT, dX).eval();

  Vector4d x_norm;
  Gradient<Vector4d, 4, 1>::type dx_norm;
  Gradient<Vector4d, 4, 2>::type ddx_norm;
  drake::math::NormalizeVector(x, x_norm, &dx_norm, &ddx_norm);

  int outnum = 0;
  plhs[outnum++] = eigenToMatlab(dT);
  plhs[outnum++] = eigenToMatlab(dTInv);
  plhs[outnum++] = eigenToMatlab(dAdT);
  plhs[outnum++] = eigenToMatlab(dAdTInv_transpose);
  plhs[outnum++] = eigenToMatlab(x_norm);
  plhs[outnum++] = eigenToMatlab(dx_norm);
  plhs[outnum++] = eigenToMatlab(ddx_norm);
}
