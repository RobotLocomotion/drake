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
  for (int i = 0; i < a.size(); ++i) {
    if (a(i).derivatives().size() != b(i).derivatives().size()) {
      return false;
    }
    if (a(i).derivatives() != b(i).derivatives()) {
      return false;
    }
  }
  return true;
}

}  // namespace math
}  // namespace drake
