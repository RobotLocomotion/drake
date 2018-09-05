#include "drake/multibody/inverse_kinematics/gaze_target_constraint.h"

#include <limits>

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
GazeTargetConstraint::GazeTargetConstraint(
    const MultibodyTree<AutoDiffXd>& tree, const FrameIndex& frameA_idx,
    const Eigen::Ref<const Eigen::Vector3d>& p_AS,
    const Eigen::Ref<const Eigen::Vector3d>& n_A, const FrameIndex& frameB_idx,
    const Eigen::Ref<const Eigen::Vector3d>& p_BT, double cone_half_angle,
    MultibodyTreeContext<AutoDiffXd>* context)
    : solvers::Constraint(
          2, tree.num_positions(), Eigen::Vector2d::Zero(),
          Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity())),
      tree_{tree},
      frameA_{tree_.get_frame(frameA_idx)},
      p_AS_{p_AS},
      n_A_{NormalizeVector(n_A)},
      frameB_{tree_.get_frame(frameB_idx)},
      p_BT_{p_BT.cast<AutoDiffXd>()},
      cone_half_angle_{cone_half_angle},
      cos_cone_half_angle_{std::cos(cone_half_angle_)},
      context_{context} {
  // TODO(hongkai.dai): use MultibodyTree<double> and LeafContext<double> when
  // MBT provides the API for computing analytical Jacobian.
  if (cone_half_angle < 0 || cone_half_angle > M_PI) {
    throw std::invalid_argument(
        "GazeTargetConstraint: cone_half_angle should be within [0, pi]");
  }
}

void GazeTargetConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                  Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), &y_t);
  *y = math::autoDiffToValueMatrix(y_t);
}

void GazeTargetConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                  AutoDiffVecXd* y) const {
  y->resize(2);
  UpdateContextConfiguration(x, context_);
  // position of target point T measured and expressed in A frame.
  Vector3<AutoDiffXd> p_AT;
  tree_.CalcPointsPositions(*context_, frameB_, p_BT_.cast<AutoDiffXd>(),
                            frameA_, &p_AT);
  const Vector3<AutoDiffXd> p_ST_A = p_AT - p_AS_;
  (*y)(0) = p_ST_A.dot(n_A_);
  (*y)(1) = (*y)(0) * (*y)(0) -
            std::pow(cos_cone_half_angle_, 2) * p_ST_A.dot(p_ST_A);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
