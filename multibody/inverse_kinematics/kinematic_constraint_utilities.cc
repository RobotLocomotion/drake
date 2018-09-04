#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
bool AreAutoDiffVecXdEqual(const Eigen::Ref<const VectorX<AutoDiffXd>>& a,
                           const Eigen::Ref<const VectorX<AutoDiffXd>>& b) {
  if (a.rows() != b.rows()) {
    return false;
  }
  if (math::autoDiffToValueMatrix(a) != math::autoDiffToValueMatrix(b)) {
    return false;
  }
  const Eigen::MatrixXd a_gradient = math::autoDiffToGradientMatrix(a);
  const Eigen::MatrixXd b_gradient = math::autoDiffToGradientMatrix(b);
  if (a_gradient.rows() != b_gradient.rows() ||
      a_gradient.cols() != b_gradient.cols()) {
    return false;
  }
  return a_gradient == b_gradient;
}
// Check if the generalized positions in @p mbt_context is the same as @p q.
// If they are not the same, then reset @p mbt_context's generalized positions
// to q. Otherwise, leave @p mbt_context unchanged.
void UpdateContextConfiguration(const Eigen::Ref<const VectorX<AutoDiffXd>>& q,
                                MultibodyTreeContext<AutoDiffXd>* mbt_context) {
  if (!AreAutoDiffVecXdEqual(q, mbt_context->get_positions())) {
    mbt_context->get_mutable_positions() = q;
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
