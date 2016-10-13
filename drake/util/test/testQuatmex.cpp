#include <mex.h>

#include <tuple>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/gradient.h"
#include "drake/math/quaternion.h"
#include "drake/util/drakeGeometryUtil.h"

using namespace Eigen;
using namespace std;
using namespace drake;

using drake::math::autoDiffToGradientMatrix;
using drake::math::autoDiffToValueMatrix;
using drake::math::initializeAutoDiffTuple;
using drake::math::quatDiff;
using drake::math::quatDiffAxisInvar;
using drake::math::quatProduct;
using drake::math::quatRotateVec;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nrhs != 5) {
    mexErrMsgIdAndTxt("Drake:testQuatmex:BadInputs",
                      "Usage [r, dr, e, ed, quat, dquat, q3, dq3, w, dw] = "
                      "testQuatmex(q1, q2, axis, u, v)");
  }
  Vector4d q1;
  Vector4d q2;
  memcpy(q1.data(), mxGetPr(prhs[0]), sizeof(double) * 4);
  memcpy(q2.data(), mxGetPr(prhs[1]), sizeof(double) * 4);

  Vector3d axis;
  memcpy(axis.data(), mxGetPr(prhs[2]), sizeof(double) * 3);

  Vector3d u, v;
  memcpy(u.data(), mxGetPr(prhs[3]), sizeof(double) * 3);
  memcpy(v.data(), mxGetPr(prhs[4]), sizeof(double) * 3);

  {
    auto autodiff_args = initializeAutoDiffTuple(q1, q2);
    auto r_autodiff = quatDiff(get<0>(autodiff_args), get<1>(autodiff_args));
    auto r = autoDiffToValueMatrix(r_autodiff);
    auto dr = autoDiffToGradientMatrix(r_autodiff);

    plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(4, 8, mxREAL);
    memcpy(mxGetPr(plhs[0]), r.data(), sizeof(double) * 4);
    memcpy(mxGetPr(plhs[1]), dr.data(), sizeof(double) * 4 * 8);
  }

  {
    auto autodiff_args = initializeAutoDiffTuple(q1, q2, axis);
    auto e_autodiff = quatDiffAxisInvar(
        get<0>(autodiff_args), get<1>(autodiff_args), get<2>(autodiff_args));
    auto e = e_autodiff.value();
    auto de = e_autodiff.derivatives().transpose().eval();

    plhs[2] = mxCreateDoubleScalar(e);
    plhs[3] = mxCreateDoubleMatrix(1, 11, mxREAL);
    memcpy(mxGetPr(plhs[3]), de.data(), sizeof(double) * 11);
  }

  {
    auto autodiff_args = initializeAutoDiffTuple(q1, q2);
    auto q3_autodiff =
        quatProduct(get<0>(autodiff_args), get<1>(autodiff_args));
    auto q3 = autoDiffToValueMatrix(q3_autodiff);
    auto dq3 = autoDiffToGradientMatrix(q3_autodiff);

    plhs[4] = mxCreateDoubleMatrix(4, 1, mxREAL);
    plhs[5] = mxCreateDoubleMatrix(4, 8, mxREAL);
    memcpy(mxGetPr(plhs[4]), q3.data(), sizeof(double) * 4);
    memcpy(mxGetPr(plhs[5]), dq3.data(), sizeof(double) * 4 * 8);
  }

  {
    auto autodiff_args = initializeAutoDiffTuple(q1, u);
    auto w_autodiff =
        quatRotateVec(get<0>(autodiff_args), get<1>(autodiff_args));
    auto w = autoDiffToValueMatrix(w_autodiff);
    auto dw = autoDiffToGradientMatrix(w_autodiff);

    plhs[6] = mxCreateDoubleMatrix(3, 1, mxREAL);
    plhs[7] = mxCreateDoubleMatrix(3, 7, mxREAL);
    memcpy(mxGetPr(plhs[6]), w.data(), sizeof(double) * 3);
    memcpy(mxGetPr(plhs[7]), dw.data(), sizeof(double) * 3 * 7);
  }
}
