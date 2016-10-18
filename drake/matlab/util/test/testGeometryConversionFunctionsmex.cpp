#include "drake/common/constants.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/gradient.h"
#include "drake/math/quaternion.h"
#include "drake/math/rotation_conversion_gradient.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/matlab/util/drakeMexUtil.h"

using namespace Eigen;

using drake::kQuaternionSize;
using drake::kRpySize;
using drake::kSpaceDimension;
using drake::math::autoDiffToValueMatrix;
using drake::math::autoDiffToGradientMatrix;
using drake::math::initializeAutoDiff;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nrhs != 2 || nlhs != 13) {
    mexErrMsgIdAndTxt(
        "Drake:testGeometryConversionFunctionsmex:BadInputs",
        "Usage [rpy,omega2qd, domega2qd, omega2rpyd, domega2rpyd, "
        "ddomega2rpyd, rpyd2omega, drpyd2omega, qd2omega, "
        "dqd2omega, dq2R, drpydR, dqdR] = "
        "testGeometryConversionFunctionsmex(q, dq)");
  }

  int argnum = 0;
  Isometry3d T;
  auto q = matlabToEigen<kQuaternionSize, 1>(prhs[argnum++]);
  auto dq = matlabToEigen<kQuaternionSize, Eigen::Dynamic>(prhs[argnum++]);

  auto rpy = drake::math::quat2rpy(q);

  Matrix<double, kQuaternionSize, kSpaceDimension> omega2qd;
  Gradient<Matrix<double, kQuaternionSize, kSpaceDimension>, kQuaternionSize,
           1>::type domega2qd;
  Matrix<double, kRpySize, kSpaceDimension> omega2rpyd;
  Gradient<Matrix<double, kRpySize, kSpaceDimension>, kRpySize, 1>::type
      domega2rpyd;
  Gradient<Matrix<double, kRpySize, kSpaceDimension>, kRpySize, 2>::type
      ddomega2rpyd;
  Matrix<double, kSpaceDimension, kQuaternionSize> qd2omega;
  Gradient<Matrix<double, kSpaceDimension, kQuaternionSize>, kQuaternionSize,
           1>::type dqd2omega;

  angularvel2quatdotMatrix(q, omega2qd, &domega2qd);
  angularvel2rpydotMatrix(rpy, omega2rpyd, &domega2rpyd, &ddomega2rpyd);
  Matrix<double, kSpaceDimension, kRpySize> rpyd2omega;
  Gradient<Matrix<double, kSpaceDimension, kRpySize>, kRpySize, 1>::type
      drpyd2omega;
  rpydot2angularvelMatrix(rpy, rpyd2omega, &drpyd2omega);

  auto qd2omega_autodiff = quatdot2angularvelMatrix(initializeAutoDiff(q));
  qd2omega = autoDiffToValueMatrix(qd2omega_autodiff);
  dqd2omega = autoDiffToGradientMatrix(qd2omega_autodiff);

  auto R = drake::math::quat2rotmat(q);
  auto dq2R = drake::math::dquat2rotmat(q);
  Matrix<double, drake::kRotmatSize, Dynamic> dR = dq2R * dq;
  auto drpydR = drake::math::drotmat2rpy(R, dR);
  auto dqdR = drake::math::drotmat2quat(R, dR);

  int outnum = 0;
  plhs[outnum++] = eigenToMatlab(rpy);
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
