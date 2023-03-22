#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace math {

bool AreAutoDiffVecXdEqual(const Eigen::Ref<const VectorX<AutoDiffXd>>& a,
                           const Eigen::Ref<const VectorX<AutoDiffXd>>& b) {
  if (a.rows() != b.rows()) {
    return false;
  }
  if (math::ExtractValue(a) != math::ExtractValue(b)) {
    return false;
  }
  const Eigen::MatrixXd a_gradient = math::ExtractGradient(a);
  const Eigen::MatrixXd b_gradient = math::ExtractGradient(b);
  if (a_gradient.rows() != b_gradient.rows() ||
      a_gradient.cols() != b_gradient.cols()) {
    return false;
  }
  return a_gradient == b_gradient;
}

}  // namespace math
}  // namespace drake
