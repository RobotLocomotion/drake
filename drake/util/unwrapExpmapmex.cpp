#include <Eigen/Core>
#include "drake/util/mexify.h"
#include "drake/util/standardMexConversions.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/makeFunction.h"
#include "drake/core/Gradient.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

// note: gradient only w.r.t. expmap2...
pair<Vector3d, typename Gradient<Vector3d, 3>::type> unwrapExpmapWithGradient(const MatrixBase<Map<const Vector3d>> &expmap1, const MatrixBase<Map<const Vector3d>> &expmap2) {
  auto args = initializeAutoDiffTuple(expmap1, expmap2);
  auto unwrapped_autodiff = unwrapExpmap(get<0>(args), get<1>(args));
  auto grad_expmap2 = autoDiffToGradientMatrix(unwrapped_autodiff).rightCols<3>();
  return make_pair(autoDiffToValueMatrix(unwrapped_autodiff), grad_expmap2);
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nlhs == 1) {
    auto func = make_function(&unwrapExpmap<Map<const Vector3d>, Map<const Vector3d>>);
    mexCallFunction(nlhs, plhs, nrhs, prhs, true, func);
  }
  else if (nlhs == 2) {
    auto func = make_function(&unwrapExpmapWithGradient);
    mexCallFunction(nlhs, plhs, nrhs, prhs, true, func);
  }
  else
    throw std::runtime_error("can't handle requested number of output arguments");
}
