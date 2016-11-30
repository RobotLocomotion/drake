#include <Eigen/Core>

#include "drake/math/autodiff_gradient.h"
#include "drake/math/expmap.h"
#include "drake/math/gradient.h"
#include "drake/matlab/util/makeFunction.h"
#include "drake/matlab/util/mexify.h"
#include "drake/matlab/util/standardMexConversions.h"
#include "drake/util/drakeGeometryUtil.h"

using namespace std;
using namespace Eigen;
using namespace drake;

using drake::math::initializeAutoDiff;

pair<Vector3d, typename Gradient<Vector3d, 4>::type> quat2expmapWithGradient(
    const MatrixBase<Map<const Vector4d>>& quat) {
  auto quat_autodiff = initializeAutoDiff(quat);
  auto expmap_autodiff = drake::math::quat2expmap(quat_autodiff);
  return make_pair(autoDiffToValueMatrix(expmap_autodiff),
                   autoDiffToGradientMatrix(expmap_autodiff));
}

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nlhs == 1) {
    auto func = make_function(&drake::math::quat2expmap<Map<const Vector4d>>);
    mexCallFunction(nlhs, plhs, nrhs, prhs, true, func);
  } else if (nlhs == 2) {
    auto func = make_function(&quat2expmapWithGradient);
    mexCallFunction(nlhs, plhs, nrhs, prhs, true, func);
  } else {
    throw std::runtime_error(
        "can't handle requested number of output arguments");
  }
}
