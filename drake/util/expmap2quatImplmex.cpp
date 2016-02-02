#include <Eigen/Core>
#include "drake/util/mexify.h"
#include "drake/util/standardMexConversions.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/makeFunction.h"
#include "drake/core/Gradient.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

pair<Vector4d, typename Gradient<Vector4d, 3>::type> expmap2quatWithGradient(MatrixBase<Map<const Vector3d>>& expmap) {
  auto expmap_autodiff = initializeAutoDiff(expmap);
  auto quat_autodiff = expmap2quat(expmap_autodiff);
  return make_pair(autoDiffToValueMatrix(quat_autodiff), autoDiffToGradientMatrix(quat_autodiff));
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  auto func_double = make_function(&expmap2quat<Map<const Vector3d>>);
  auto func_gradient = make_function(&expmap2quatWithGradient);
//  auto func_second_deriv = make_function(&expmap2quatWithSecondDeriv);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_gradient); // TODO: , func_second_deriv);
}
