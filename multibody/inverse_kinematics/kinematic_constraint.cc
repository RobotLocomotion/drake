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
    const multibody::MultibodyTree<AutoDiffXd>& tree,
    const FrameIndex& frameB_idx, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    const FrameIndex& frameA_idx,
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

OrientationConstraint::OrientationConstraint(
    const MultibodyTree<AutoDiffXd>& tree, const FrameIndex& frameA_idx,
    const FrameIndex& frameB_idx, double angle_bound,
    MultibodyTreeContext<AutoDiffXd>* context)
    : solvers::Constraint(1, tree.num_positions(),
                          Vector1d(2 * std::cos(angle_bound) - 1), Vector1d(1)),
      tree_{tree},
      frameA_idx_{frameA_idx},
      frameB_idx_{frameB_idx},
      context_(context) {
  DRAKE_DEMAND(context);
  if (angle_bound < 0) {
    throw std::logic_error(
        "OrientationConstraint: angle_bound should be non-negative.\n");
  }
}

void OrientationConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                   Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), &y_t);
  *y = math::autoDiffToValueMatrix(y_t);
}

void OrientationConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                   AutoDiffVecXd* y) const {
  y->resize(1);
  UpdateContextConfiguration(x, context_);
  (*y)(0) = tree_
                .CalcRelativeTransform(*context_, tree_.get_frame(frameA_idx_),
                                       tree_.get_frame(frameB_idx_))
                .linear()
                .trace();
}
}  // namespace multibody
}  // namespace drake
