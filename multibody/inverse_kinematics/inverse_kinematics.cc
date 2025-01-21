#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

#include <limits>
#include <utility>

#include "drake/multibody/inverse_kinematics/add_multibody_plant_constraints.h"
#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"
#include "drake/multibody/inverse_kinematics/angle_between_vectors_cost.h"
#include "drake/multibody/inverse_kinematics/distance_constraint.h"
#include "drake/multibody/inverse_kinematics/gaze_target_constraint.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_upper_bound_constraint.h"
#include "drake/multibody/inverse_kinematics/orientation_constraint.h"
#include "drake/multibody/inverse_kinematics/orientation_cost.h"
#include "drake/multibody/inverse_kinematics/point_to_line_distance_constraint.h"
#include "drake/multibody/inverse_kinematics/point_to_point_distance_constraint.h"
#include "drake/multibody/inverse_kinematics/polyhedron_constraint.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/multibody/inverse_kinematics/position_cost.h"
#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"

namespace drake {
namespace multibody {

InverseKinematics::InverseKinematics(const MultibodyPlant<double>& plant,
                                     bool with_joint_limits)
    : InverseKinematics(plant, plant.CreateDefaultContext(), nullptr,
                        with_joint_limits) {}

InverseKinematics::InverseKinematics(const MultibodyPlant<double>& plant,
                                     systems::Context<double>* plant_context,
                                     bool with_joint_limits)
    : InverseKinematics(plant, nullptr, plant_context, with_joint_limits) {}

InverseKinematics::InverseKinematics(
    const MultibodyPlant<double>& plant,
    std::unique_ptr<systems::Context<double>> owned_context,
    systems::Context<double>* plant_context, bool with_joint_limits)
    : prog_(std::make_unique<solvers::MathematicalProgram>()),
      plant_(plant),
      owned_context_(std::move(owned_context)),
      context_(owned_context_ ? owned_context_.get() : plant_context),
      q_(prog_->NewContinuousVariables(plant.num_positions(), "q")) {
  if (owned_context_ != nullptr) {
    // The public constructor must not pass both pointers when calling us.
    DRAKE_DEMAND(plant_context == nullptr);
  } else {
    // The user calling the `plant_context` overload should not pass nullptr.
    DRAKE_THROW_UNLESS(plant_context != nullptr);
  }
  DRAKE_DEMAND(context_ != nullptr);  // Sanity check.

  AddMultibodyPlantConstraints(
      std::shared_ptr<const MultibodyPlant<double>>(
          /* managed object = */ std::shared_ptr<void>{},
          /* stored pointer = */ &plant_),
      q_, prog_.get(), context_);

  if (!with_joint_limits) {
    // Remove only the joint limit constraint.
    const auto& bbox_bindings = prog_->bounding_box_constraints();
    bool removed_joint_limits = false;
    for (const auto& binding : bbox_bindings) {
      if (binding.evaluator()->get_description() == "Joint limits") {
        prog_->RemoveConstraint(binding);
        removed_joint_limits = true;
        break;
      }
    }
    DRAKE_DEMAND(removed_joint_limits);
  }
}

solvers::Binding<solvers::Constraint> InverseKinematics::AddPositionConstraint(
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    const Frame<double>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper) {
  auto constraint = std::make_shared<PositionConstraint>(
      &plant_, frameA, p_AQ_lower, p_AQ_upper, frameB, p_BQ,
      get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Constraint> InverseKinematics::AddPositionConstraint(
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    const Frame<double>& frameAbar,
    const std::optional<math::RigidTransformd>& X_AbarA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper) {
  auto constraint = std::make_shared<PositionConstraint>(
      &plant_, frameAbar, X_AbarA, p_AQ_lower, p_AQ_upper, frameB, p_BQ,
      get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Cost> InverseKinematics::AddPositionCost(
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& p_AP,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    const Eigen::Ref<const Eigen::Matrix3d>& C) {
  auto cost = std::make_shared<PositionCost>(&plant_, frameA, p_AP, frameB,
                                             p_BQ, C, get_mutable_context());
  return prog_->AddCost(cost, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddOrientationConstraint(
    const Frame<double>& frameAbar, const math::RotationMatrix<double>& R_AbarA,
    const Frame<double>& frameBbar, const math::RotationMatrix<double>& R_BbarB,
    double angle_bound) {
  auto constraint = std::make_shared<OrientationConstraint>(
      &plant_, frameAbar, R_AbarA, frameBbar, R_BbarB, angle_bound,
      get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Cost> InverseKinematics::AddOrientationCost(
    const Frame<double>& frameAbar, const math::RotationMatrix<double>& R_AbarA,
    const Frame<double>& frameBbar, const math::RotationMatrix<double>& R_BbarB,
    double c) {
  auto cost =
      std::make_shared<OrientationCost>(&plant_, frameAbar, R_AbarA, frameBbar,
                                        R_BbarB, c, get_mutable_context());
  return prog_->AddCost(cost, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddGazeTargetConstraint(
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& p_AS,
    const Eigen::Ref<const Eigen::Vector3d>& n_A, const Frame<double>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BT, double cone_half_angle) {
  auto constraint = std::make_shared<GazeTargetConstraint>(
      &plant_, frameA, p_AS, n_A, frameB, p_BT, cone_half_angle,
      get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddAngleBetweenVectorsConstraint(
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& na_A,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& nb_B,
    double angle_lower, double angle_upper) {
  auto constraint = std::make_shared<AngleBetweenVectorsConstraint>(
      &plant_, frameA, na_A, frameB, nb_B, angle_lower, angle_upper,
      get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Cost> InverseKinematics::AddAngleBetweenVectorsCost(
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& na_A,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& nb_B,
    double c) {
  auto cost = std::make_shared<AngleBetweenVectorsCost>(
      &plant_, frameA, na_A, frameB, nb_B, c, get_mutable_context());
  return prog_->AddCost(cost, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddMinimumDistanceLowerBoundConstraint(
    double bound, double influence_distance_offset) {
  auto constraint = std::shared_ptr<MinimumDistanceLowerBoundConstraint>(
      new MinimumDistanceLowerBoundConstraint(&plant_, bound,
                                              get_mutable_context(), {},
                                              influence_distance_offset));
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddMinimumDistanceUpperBoundConstraint(
    double bound, double influence_distance_offset) {
  auto constraint = std::shared_ptr<MinimumDistanceUpperBoundConstraint>(
      new MinimumDistanceUpperBoundConstraint(&plant_, bound,
                                              get_mutable_context(),
                                              influence_distance_offset, {}));
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Constraint> InverseKinematics::AddDistanceConstraint(
    const SortedPair<geometry::GeometryId>& geometry_pair,
    double distance_lower, double distance_upper) {
  auto constraint = std::make_shared<DistanceConstraint>(
      &plant_, geometry_pair, get_mutable_context(), distance_lower,
      distance_upper);
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddPointToPointDistanceConstraint(
    const Frame<double>& frame1,
    const Eigen::Ref<const Eigen::Vector3d>& p_B1P1,
    const Frame<double>& frame2,
    const Eigen::Ref<const Eigen::Vector3d>& p_B2P2, double distance_lower,
    double distance_upper) {
  auto constraint = std::make_shared<PointToPointDistanceConstraint>(
      &plant_, frame1, p_B1P1, frame2, p_B2P2, distance_lower, distance_upper,
      get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddPointToLineDistanceConstraint(
    const Frame<double>& frame_point,
    const Eigen::Ref<const Eigen::Vector3d>& p_B1P,
    const Frame<double>& frame_line,
    const Eigen::Ref<const Eigen::Vector3d>& p_B2Q,
    const Eigen::Ref<const Eigen::Vector3d>& n_B2, double distance_lower,
    double distance_upper) {
  auto constraint = std::make_shared<PointToLineDistanceConstraint>(
      &plant_, frame_point, p_B1P, frame_line, p_B2Q, n_B2, distance_lower,
      distance_upper, get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}

solvers::Binding<solvers::Constraint>
InverseKinematics::AddPolyhedronConstraint(
    const Frame<double>& frameF, const Frame<double>& frameG,
    const Eigen::Ref<const Eigen::Matrix3Xd>& p_GP,
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  auto constraint = std::make_shared<PolyhedronConstraint>(
      &plant_, frameF, frameG, p_GP, A, b, get_mutable_context());
  return prog_->AddConstraint(constraint, q_);
}
}  // namespace multibody
}  // namespace drake
