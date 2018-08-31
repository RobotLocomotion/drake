#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

namespace drake {
namespace multibody {
InverseKinematics::InverseKinematics(const MultibodyTree<AutoDiffXd>& tree)
    : tree_(tree),
      context_(tree_.CreateDefaultContext()),
      q_(NewContinuousVariables(tree_.num_positions(), "q")) {
        // Add joint limit constraint here.
}

solvers::Binding<PositionConstraint> InverseKinematics::AddPositionConstraint(
    const FrameIndex& frameB_idx, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    const FrameIndex& frameA_idx,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper) {
  auto constraint = std::make_shared<PositionConstraint>(
      tree_, frameB_idx, p_BQ, frameA_idx, p_AQ_lower, p_AQ_upper,
      get_mutable_context());
  AddConstraint(constraint, q_);
  return solvers::Binding<PositionConstraint>(constraint, q_);
}
}  // namespace multibody
}  // namespace drake
