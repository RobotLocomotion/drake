#include <Eigen/Core>
#include "drake/util/mexify.h"
#include "drake/util/standardMexConversions.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/makeFunction.h"
#include "drake/core/Gradient.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

pair<Vector3d, typename Gradient<Vector3d, 6>::type> closestExpmapWithGradient(const MatrixBase<Map<const Vector3d>>& expmap1, const MatrixBase<Map<const Vector3d>>& expmap2) {
  auto args = initializeAutoDiffTuple(expmap1, expmap2);
  auto closest_autodiff = closestExpmap(get<0>(args), get<1>(args));
  return make_pair(autoDiffToValueMatrix(closest_autodiff), autoDiffToGradientMatrix(closest_autodiff));
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  auto func_double = make_function(&closestExpmap<Map<const Vector3d>, Map<const Vector3d>>);
  auto func_gradient = make_function(&closestExpmapWithGradient);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_gradient);
}
