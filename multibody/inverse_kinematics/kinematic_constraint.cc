#include "drake/multibody/inverse_kinematics/kinematic_constraint.h"

namespace drake {
namespace multibody {
namespace {
// Check if the generalized positions in @p mbt_context is the same as @p q.
// If they are not the same, then reset @p mbt_context's generalized positions
// to q. Otherwise, leave @p mbt_context unchanged.
template <typename T>
void UpdateContextConfiguration(const Eigen::Ref<const VectorX<T>>& q,
                                MultibodyTreeContext<T>* mbt_context) {
  if (mbt_context->get_positions() != q) {
    mbt_context->get_mutable_positions() = q;
  }
}
}  // namespace
PositionConstraint::PositionConstraint(
    const multibody::MultibodyTree<AutoDiffXd>& tree, FrameIndex frameB_idx,
    const Eigen::Ref<const Eigen::Vector3d>& p_BQ, FrameIndex frameA_idx,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
    MultibodyTreeContext<AutoDiffXd>* context)
    : solvers::Constraint(3, tree.num_positions(), p_AQ_lower, p_AQ_upper),
      tree_(tree),
      frameB_idx_{frameB_idx},
      frameA_idx_{frameA_idx},
      p_BQ_{p_BQ},
      context_{context} {
  DRAKE_DEMAND(context);
}

void PositionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), &y_t);
  *y = math::autoDiffToValueMatrix(y_t);
}

void PositionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                AutoDiffVecXd* y) const {
  y->resize(3);
  UpdateContextConfiguration(x, context_);
  tree_.CalcPointsPositions(*context_, tree_.get_frame(frameB_idx_),
                            p_BQ_.cast<AutoDiffXd>(),
                            tree_.get_frame(frameA_idx_), y);
}
}  // namespace multibody
}  // namespace drake
