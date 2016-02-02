#include <Eigen/Core>
#include "drake/util/mexify.h"
#include "drake/util/standardMexConversions.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/makeFunction.h"
#include "drake/core/Gradient.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

pair<Vector4d, typename Gradient<Vector4d, 3>::type> expmap2quatWithGradient(const MatrixBase<Map<const Vector3d>>& expmap) {
  auto expmap_autodiff = initializeAutoDiff(expmap);
  auto quat_autodiff = expmap2quat(expmap_autodiff);
  return make_pair(autoDiffToValueMatrix(quat_autodiff), autoDiffToGradientMatrix(quat_autodiff));
}


tuple<Vector4d, typename Gradient<Vector4d, 3>::type, typename Gradient<Vector4d, 3, 2>::type> expmap2quatWithSecondDeriv(const MatrixBase<Map<const Vector3d>>& expmap) {
  auto expmap_autodiff = initializeAutoDiff(expmap);
  auto expmap_autodiff_second = initializeAutoDiff(expmap_autodiff);
  auto quat_autodiff_second = expmap2quat(expmap_autodiff_second);
  auto quat = autoDiffToValueMatrix(autoDiffToValueMatrix(quat_autodiff_second));
  auto dquat_autodiff = autoDiffToGradientMatrix(quat_autodiff_second);
  auto dquat = autoDiffToValueMatrix(dquat_autodiff);
  auto ddquat = autoDiffToGradientMatrix(dquat_autodiff);
  return make_tuple(quat, dquat, ddquat);
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nlhs == 1) {
    auto func_double = make_function(&expmap2quat<Map<const Vector3d>>);
    mexCallFunction(nlhs, plhs, nrhs, prhs, true, func_double);
  }
  else if (nlhs == 2) {
    auto func_gradient = make_function(&expmap2quatWithGradient);
    mexCallFunction(nlhs, plhs, nrhs, prhs, true, func_gradient);
  }
  else if (nlhs == 3) {
    auto func_second_deriv = make_function(&expmap2quatWithSecondDeriv);
    mexCallFunction(nlhs, plhs, nrhs, prhs, true, func_second_deriv);
  }
  else
    throw std::runtime_error("can't handle requested number of output arguments");
}
