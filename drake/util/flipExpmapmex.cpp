#include <Eigen/Core>
#include "drake/util/mexify.h"
#include "drake/util/standardMexConversions.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/makeFunction.h"
#include "drake/core/Gradient.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

pair<Vector3d, typename Gradient<Vector3d, 3>::type> quat2expmapWithGradient(const MatrixBase<Map<const Vector3d>> &expmap) {
  auto expmap_autodiff = initializeAutoDiff(expmap);
  auto flipped_autodiff = flipExpmap(expmap_autodiff);
  return make_pair(autoDiffToValueMatrix(flipped_autodiff), autoDiffToGradientMatrix(flipped_autodiff));
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nlhs == 1) {
    auto func = make_function(&flipExpmap<Map<const Vector3d>>);
    mexCallFunction(nlhs, plhs, nrhs, prhs, true, func);
  }
  else if (nlhs == 2) {
    auto func = make_function(&quat2expmapWithGradient);
    mexCallFunction(nlhs, plhs, nrhs, prhs, true, func);
  }
  else
    throw std::runtime_error("can't handle requested number of output arguments");
}
