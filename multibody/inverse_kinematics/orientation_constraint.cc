#include "drake/multibody/inverse_kinematics/orientation_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
OrientationConstraint::OrientationConstraint(
    const MultibodyTree<AutoDiffXd>& tree, const FrameIndex& frameA_idx,
    const FrameIndex& frameB_idx, double angle_bound,
    MultibodyTreeContext<AutoDiffXd>* context)
    : solvers::Constraint(1, tree.num_positions(),
                          Vector1d(2 * std::cos(angle_bound) + 1), Vector1d(3)),
      tree_{tree},
      frameA_{tree.get_frame(frameA_idx)},
      frameB_{tree.get_frame(frameB_idx)},
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
  (*y)(0) =
      tree_.CalcRelativeTransform(*context_, frameA_, frameB_).linear().trace();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
