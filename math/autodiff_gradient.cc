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
  for (int i = 0; i < a.rows(); ++i) {
    const AutoDiffXd& a_i = internal::DirectlyReadCoefficient(a, i, 0);
    const AutoDiffXd& b_i = internal::DirectlyReadCoefficient(b, i, 0);
    if (a_i.derivatives().size() != b_i.derivatives().size()) {
      return false;
    }
    if (a_i.derivatives() != b_i.derivatives()) {
      return false;
    }
  }
  return true;
}

}  // namespace math
}  // namespace drake
