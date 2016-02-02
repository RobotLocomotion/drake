#include <Eigen/Core>
#include "drake/util/mexify.h"
#include "drake/util/standardMexConversions.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/makeFunction.h"
#include "drake/core/Gradient.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

pair<Vector3d, typename Gradient<Vector3d, 4>::type> quat2expmapWithGradient(MatrixBase<Map<const Vector4d>> &quat) {
  auto quat_autodiff = initializeAutoDiff(quat);
  auto expmap_autodiff = quat2expmap(quat_autodiff);
  return make_pair(autoDiffToValueMatrix(expmap_autodiff), autoDiffToGradientMatrix(expmap_autodiff));
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  auto func_double = make_function(&quat2expmap<Map<const Vector4d>>);
  auto func_gradient = make_function(&quat2expmapWithGradient);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_gradient);
}
