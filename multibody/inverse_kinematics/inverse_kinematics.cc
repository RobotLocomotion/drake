#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

#include <limits>

#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"
#include "drake/multibody/inverse_kinematics/gaze_target_constraint.h"
#include "drake/multibody/inverse_kinematics/minimal_distance_constraint.h"
#include "drake/multibody/inverse_kinematics/orientation_constraint.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"

namespace drake {
namespace multibody {
using multibody_plant::MultibodyPlant;
InverseKinematics::InverseKinematics(const MultibodyPlant<double>& plant)
    : prog_{new solvers::MathematicalProgram()},
      owned_diagram_{nullptr},
      diagram_{nullptr},
      owned_plant_{plant.ToAutoDiffXd()},
      plant_autodiff_{
          dynamic_cast<MultibodyPlant<AutoDiffXd>*>(owned_plant_.get())},
      owned_context_(owned_plant_->CreateDefaultContext()),
      context_(owned_context_.get()),
      q_(prog_->NewContinuousVariables(plant.num_positions(), "q")) {
  AddDefaultJointLimitConstraint();
  // TODO(hongkai.dai) Add other position constraints, such as unit length
  // quaternion constraint here.
}

/**
 * Given that @p diagram_double contains @p plant, and @p diagram_autodiff is
 * transmogrofied from @p diagram_double, returns the pointer to
 * MultibodyPlant<AutoDiffXd> in the transmogrofied diagram_autodiff.
 */
const multibody_plant::MultibodyPlant<AutoDiffXd>* FindMultibodyPlantInDiagram(
    const multibody_plant::MultibodyPlant<double>& plant,
    const systems::Diagram<AutoDiffXd>& diagram_autodiff) {
  return dynamic_cast<const MultibodyPlant<AutoDiffXd>*>(
      &(diagram_autodiff.GetSubsystemByName(plant.get_name())));
}

InverseKinematics::InverseKinematics(
    const systems::Diagram<double>& diagram,
    const multibody_plant::MultibodyPlant<double>& plant,
    systems::Context<double>*)
    : prog_{new solvers::MathematicalProgram()},
      owned_diagram_{diagram.ToAutoDiffXd()},
      diagram_{dynamic_cast<const systems::Diagram<AutoDiffXd>* const>(
          owned_diagram_.get())},
      owned_plant_{nullptr},
      plant_autodiff_{FindMultibodyPlantInDiagram(plant, *diagram_)},
      owned_context_{owned_diagram_->CreateDefaultContext()},
      context_{&(diagram_->GetMutableSubsystemContext(*plant_autodiff_,
                                                      owned_context_.get()))},
      q_(prog_->NewContinuousVariables(plant.num_positions(), "q")) {
  AddDefaultJointLimitConstraint();
  // TODO(hongkai.dai) Add other position constraints, such as unit length
  // quaternion constraint here.
}

void InverseKinematics::AddDefaultJointLimitConstraint() {
  // Initialize the lower and upper bounds to -inf/inf. A free floating body
  // does not increment `num_joints()` (A single free floating body has
  // num_joints() = 0), but has 7 generalized positions for each free floating
  // body. The initialization below guarantees proper bounds on the
  // generalized positions for the free floating body.
  Eigen::VectorXd q_lower = Eigen::VectorXd::Constant(
      tree().num_positions(), -std::numeric_limits<double>::infinity());
  Eigen::VectorXd q_upper = Eigen::VectorXd::Constant(
      tree().num_positions(), std::numeric_limits<double>::infinity());
  for (JointIndex i{0}; i < tree().num_joints(); ++i) {
    const auto& joint = tree().get_joint(i);
    q_lower.segment(joint.position_start(), joint.num_positions()) =
        joint.lower_limits();
    q_upper.segment(joint.position_start(), joint.num_positions()) =
        joint.upper_limits();
  }
  prog_->AddBoundingBoxConstraint(q_lower, q_upper, q_);
}

solvers::Binding<solvers::Constraint> InverseKinematics::AddPositionConstraint(
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    const Frame<double>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper) {
  auto constraint = std::make_shared<internal::PositionConstraint>(
      tree(), frameB.index(), p_BQ, frameA.index(), p_AQ_lower, p_AQ_upper,
      get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddOrientationConstraint(
    const Frame<double>& frameAbar, const math::RotationMatrix<double>& R_AbarA,
    const Frame<double>& frameBbar, const math::RotationMatrix<double>& R_BbarB,
    double angle_bound) {
  auto constraint = std::make_shared<internal::OrientationConstraint>(
      tree(), frameAbar.index(), R_AbarA, frameBbar.index(), R_BbarB,
      angle_bound, get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddGazeTargetConstraint(
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& p_AS,
    const Eigen::Ref<const Eigen::Vector3d>& n_A, const Frame<double>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BT, double cone_half_angle) {
  auto constraint = std::make_shared<internal::GazeTargetConstraint>(
      tree(), frameA.index(), p_AS, n_A, frameB.index(), p_BT, cone_half_angle,
      get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddAngleBetweenVectorsConstraint(
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& na_A,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& nb_B,
    double angle_lower, double angle_upper) {
  auto constraint = std::make_shared<internal::AngleBetweenVectorsConstraint>(
      tree(), frameA.index(), na_A, frameB.index(), nb_B, angle_lower,
      angle_upper, get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddMinimalDistanceConstraint(double minimal_distance) {
  auto constraint = std::make_shared<internal::MinimalDistanceConstraint>(
      *plant_autodiff_, minimal_distance, context_);
  return prog_->AddConstraint(constraint, q_);
}
}  // namespace multibody
}  // namespace drake
