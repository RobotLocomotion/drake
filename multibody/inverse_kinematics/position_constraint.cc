#include "drake/multibody/inverse_kinematics/position_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
PositionConstraint::PositionConstraint(
    const multibody::MultibodyTree<AutoDiffXd>& tree,
    const FrameIndex& frameB_idx, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    const FrameIndex& frameA_idx,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
    MultibodyTreeContext<AutoDiffXd>* context)
    : solvers::Constraint(3, tree.num_positions(), p_AQ_lower, p_AQ_upper),
      tree_(tree),
      frameB_{tree_.get_frame(frameB_idx)},
      frameA_{tree_.get_frame(frameA_idx)},
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
  tree_.CalcPointsPositions(*context_, frameB_, p_BQ_.cast<AutoDiffXd>(),
                            frameA_, y);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
