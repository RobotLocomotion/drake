#include <Eigen/Core>
#include "drake/util/mexify.h"
#include "drake/util/standardMexConversions.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/makeFunction.h"
#include "drake/core/Gradient.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

pair<Vector3d, typename Gradient<Vector3d, 6>::type> unwrapExpmapWithGradient(const MatrixBase<Map<const Vector3d>> &expmap1, const MatrixBase<Map<const Vector3d>> &expmap2) {
  auto args = initializeAutoDiffTuple(expmap1, expmap2);
  auto unwrapped_autodiff = unwrapExpmap(get<0>(args), get<1>(args));
  return make_pair(autoDiffToValueMatrix(unwrapped_autodiff), autoDiffToGradientMatrix(unwrapped_autodiff));
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  auto func_double = make_function(&unwrapExpmap<Map<const Vector3d>, Map<const Vector3d>>);
  auto func_gradient = make_function(&unwrapExpmapWithGradient);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_gradient);
}
