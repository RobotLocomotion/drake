#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
AngleBetweenVectorsConstraint::AngleBetweenVectorsConstraint(
    const MultibodyTree<AutoDiffXd>& tree, const FrameIndex& frameA_idx,
    const Eigen::Ref<const Eigen::Vector3d>& n_A, const FrameIndex& frameB_idx,
    const Eigen::Ref<const Eigen::Vector3d>& n_B, double angle_lower,
    double angle_upper, MultibodyTreeContext<AutoDiffXd>* context)
    : solvers::Constraint(1, tree.num_positions(),
                          Vector1d(std::cos(angle_upper)),
                          Vector1d(std::cos(angle_lower))),
      tree_(tree),
      frameA_(tree_.get_frame(frameA_idx)),
      n_A_A_(NormalizeVector(n_A)),
      frameB_(tree_.get_frame(frameB_idx)),
      n_B_B_(NormalizeVector(n_B)),
      context_(context) {
  if (!(angle_lower >= 0 && angle_upper >= angle_lower &&
        angle_upper <= M_PI)) {
    throw std::logic_error(
        "AngleBetweenVectorsConstraint: should satisfy 0 <= angle_lower <= "
        "angle_upper <= pi");
  }
}

void AngleBetweenVectorsConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), &y_t);
  *y = math::autoDiffToValueMatrix(y_t);
}

void AngleBetweenVectorsConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  y->resize(1);
  UpdateContextConfiguration(x, context_);
  (*y)(0) = n_A_A_.dot(
      tree_.CalcRelativeTransform(*context_, frameA_, frameB_).linear() *
      n_B_B_);
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
