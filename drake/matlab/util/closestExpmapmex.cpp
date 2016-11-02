#include <Eigen/Core>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/expmap.h"
#include "drake/math/gradient.h"
#include "drake/matlab/util/mexify.h"
#include "drake/matlab/util/standardMexConversions.h"
#include "drake/matlab/util/makeFunction.h"

using namespace std;
using namespace Eigen;
using namespace drake;

using drake::math::autoDiffToValueMatrix;
using drake::math::autoDiffToGradientMatrix;
using drake::math::initializeAutoDiff;

// note: gradient only w.r.t. expmap2...
pair<Vector3d, typename Gradient<Vector3d, 3>::type> closestExpmapWithGradient(
    const MatrixBase<Map<const Vector3d>>& expmap1,
    const MatrixBase<Map<const Vector3d>>& expmap2) {
  auto expmap2_autodiff = initializeAutoDiff(expmap2);
  auto expmap1_autodiff =
      (expmap1.cast<decltype(expmap2_autodiff)::Scalar>()).eval();
  auto closest_autodiff = drake::math::closestExpmap(expmap1_autodiff,
                                                     expmap2_autodiff);
  return make_pair(autoDiffToValueMatrix(closest_autodiff),
                   autoDiffToGradientMatrix(closest_autodiff));
}

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nlhs == 1) {
    auto func = make_function(
        &drake::math::closestExpmap<Map<const Vector3d>, Map<const Vector3d>>);
    mexCallFunction(nlhs, plhs, nrhs, prhs, true, func);
  } else if (nlhs == 2) {
    auto func = make_function(&closestExpmapWithGradient);
    mexCallFunction(nlhs, plhs, nrhs, prhs, true, func);
  } else {
    throw std::runtime_error(
        "can't handle requested number of output arguments");
  }
}
