#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"
#include "drake/multibody/inverse_kinematics/gaze_target_constraint.h"
#include "drake/multibody/inverse_kinematics/orientation_constraint.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"

namespace drake {
namespace multibody {
InverseKinematics::InverseKinematics(const MultibodyTree<double>& tree)
    : tree_(tree.ToAutoDiffXd()),
      context_(tree_->CreateDefaultContext()),
      q_(NewContinuousVariables(tree_->num_positions(), "q")) {
  // Add joint limit constraint here.
}

solvers::Binding<solvers::Constraint> InverseKinematics::AddPositionConstraint(
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    const Frame<double>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper) {
  auto constraint = std::make_shared<internal::PositionConstraint>(
      *tree_, frameB.index(), p_BQ, frameA.index(), p_AQ_lower, p_AQ_upper,
      get_mutable_context());
  AddConstraint(constraint, q_);
  return solvers::Binding<solvers::Constraint>(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddOrientationConstraint(const Frame<double>& frameA,
                                            const Frame<double>& frameB,
                                            double angle_bound) {
  auto constraint = std::make_shared<internal::OrientationConstraint>(
      *tree_, frameA.index(), frameB.index(), angle_bound,
      get_mutable_context());
  AddConstraint(constraint, q_);
  return solvers::Binding<solvers::Constraint>(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddGazeTargetConstraint(
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& p_AS,
    const Eigen::Ref<const Eigen::Vector3d>& n_A, const Frame<double>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BT, double cone_half_angle) {
  auto constraint = std::make_shared<internal::GazeTargetConstraint>(
      *tree_, frameA.index(), p_AS, n_A, frameB.index(), p_BT, cone_half_angle,
      get_mutable_context());
  AddConstraint(constraint, q_);
  return solvers::Binding<solvers::Constraint>(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddAngleBetweenVectorsConstraint(
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& n_A,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& n_B,
    double angle_lower, double angle_upper) {
  auto constraint = std::make_shared<internal::AngleBetweenVectorsConstraint>(
      *tree_, frameA.index(), n_A, frameB.index(), n_B, angle_lower,
      angle_upper, get_mutable_context());
  AddConstraint(constraint, q_);
  return solvers::Binding<solvers::Constraint>(constraint, q_);
}
}  // namespace multibody
}  // namespace drake
