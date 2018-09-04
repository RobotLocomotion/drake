#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

namespace drake {
namespace multibody {
InverseKinematics::InverseKinematics(const MultibodyTree<double>& tree)
    : tree_(tree.ToAutoDiffXd()),
      context_(tree_->CreateDefaultContext()),
      q_(NewContinuousVariables(tree_->num_positions(), "q")) {
  // Add joint limit constraint here.
}

solvers::Binding<PositionConstraint> InverseKinematics::AddPositionConstraint(
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    const Frame<double>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper) {
  auto constraint = std::make_shared<PositionConstraint>(
      *tree_, frameB.index(), p_BQ, frameA.index(), p_AQ_lower, p_AQ_upper,
      get_mutable_context());
  AddConstraint(constraint, q_);
  return solvers::Binding<PositionConstraint>(constraint, q_);
}

solvers::Binding<OrientationConstraint>
InverseKinematics::AddOrientationConstraint(const Frame<double>& frameA,
                                            const Frame<double>& frameB,
                                            double angle_bound) {
  auto constraint = std::make_shared<OrientationConstraint>(
      *tree_, frameA.index(), frameB.index(), angle_bound,
      get_mutable_context());
  AddConstraint(constraint, q_);
  return solvers::Binding<OrientationConstraint>(constraint, q_);
}

solvers::Binding<GazeTargetConstraint>
InverseKinematics::AddGazeTargetConstraint(
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& p_AS,
    const Eigen::Ref<const Eigen::Vector3d>& n_A, const Frame<double>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BT, double cone_half_angle) {
  auto constraint = std::make_shared<GazeTargetConstraint>(
      *tree_, frameA.index(), p_AS, n_A, frameB.index(), p_BT, cone_half_angle,
      get_mutable_context());
  AddConstraint(constraint, q_);
  return solvers::Binding<GazeTargetConstraint>(constraint, q_);
}

solvers::Binding<AngleBetweenVectorsConstraint>
InverseKinematics::AddAngleBetweenVectorsConstraint(
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& n_A,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& n_B,
    double angle_lower, double angle_upper) {
  auto constraint = std::make_shared<AngleBetweenVectorsConstraint>(
      *tree_, frameA.index(), n_A, frameB.index(), n_B, angle_lower,
      angle_upper, get_mutable_context());
  AddConstraint(constraint, q_);
  return solvers::Binding<AngleBetweenVectorsConstraint>(constraint, q_);
}
}  // namespace multibody
}  // namespace drake
