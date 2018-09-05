#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"
#include "drake/multibody/inverse_kinematics/gaze_target_constraint.h"
#include "drake/multibody/inverse_kinematics/orientation_constraint.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"

namespace drake {
namespace multibody {
InverseKinematics::InverseKinematics(
    const multibody_plant::MultibodyPlant<double>& plant)
    : prog_{new solvers::MathematicalProgram()},
      plant_(plant),
      tree_(plant_.model().ToAutoDiffXd()),
      context_(tree_->CreateDefaultContext()),
      q_(prog_->NewContinuousVariables(plant_.num_positions(), "q")) {
  // TODO(hongkai.dai) Add joint limit constraint here.
  // TODO(hongkai.dai) Add other position constraints, such as unit length
  // quaternion constraint here.
}

solvers::Binding<solvers::Constraint> InverseKinematics::AddPositionConstraint(
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    const Frame<double>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper) {
  auto constraint = std::make_shared<internal::PositionConstraint>(
      *tree_, frameB.index(), p_BQ, frameA.index(), p_AQ_lower, p_AQ_upper,
      get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddOrientationConstraint(const Frame<double>& frameA,
                                            const Frame<double>& frameB,
                                            double angle_bound) {
  auto constraint = std::make_shared<internal::OrientationConstraint>(
      *tree_, frameA.index(), frameB.index(), angle_bound,
      get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddGazeTargetConstraint(
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& p_AS,
    const Eigen::Ref<const Eigen::Vector3d>& n_A, const Frame<double>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BT, double cone_half_angle) {
  auto constraint = std::make_shared<internal::GazeTargetConstraint>(
      *tree_, frameA.index(), p_AS, n_A, frameB.index(), p_BT, cone_half_angle,
      get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddAngleBetweenVectorsConstraint(
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& na_A,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& nb_B,
    double angle_lower, double angle_upper) {
  auto constraint = std::make_shared<internal::AngleBetweenVectorsConstraint>(
      *tree_, frameA.index(), na_A, frameB.index(), nb_B, angle_lower,
      angle_upper, get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}
}  // namespace multibody
}  // namespace drake
