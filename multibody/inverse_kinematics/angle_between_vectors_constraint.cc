#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
AngleBetweenVectorsConstraint::AngleBetweenVectorsConstraint(
    const MultibodyTree<AutoDiffXd>& tree, FrameIndex frameA_idx,
    const Eigen::Ref<const Eigen::Vector3d>& na_A, FrameIndex frameB_idx,
    const Eigen::Ref<const Eigen::Vector3d>& nb_B, double angle_lower,
    double angle_upper, MultibodyTreeContext<AutoDiffXd>* context)
    : solvers::Constraint(1, tree.num_positions(),
                          Vector1d(std::cos(angle_upper)),
                          Vector1d(std::cos(angle_lower))),
      tree_(tree),
      frameA_(tree_.get_frame(frameA_idx)),
      na_unit_A_(NormalizeVector(na_A)),
      frameB_(tree_.get_frame(frameB_idx)),
      nb_unit_B_(NormalizeVector(nb_B)),
      context_(context) {
  // TODO(hongkai.dai): use MultibodyTree<double> and LeafContext<double> when
  // MBT provides the API for computing analytical Jacobian.
  if (!(angle_lower >= 0 && angle_upper >= angle_lower &&
        angle_upper <= M_PI)) {
    throw std::invalid_argument(
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
  (*y)(0) = na_unit_A_.dot(
      tree_.CalcRelativeTransform(*context_, frameA_, frameB_).linear() *
      nb_unit_B_);
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
