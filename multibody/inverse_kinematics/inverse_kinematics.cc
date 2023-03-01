#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

#include <limits>

#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"
#include "drake/multibody/inverse_kinematics/angle_between_vectors_cost.h"
#include "drake/multibody/inverse_kinematics/distance_constraint.h"
#include "drake/multibody/inverse_kinematics/gaze_target_constraint.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"
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
    : prog_{new solvers::MathematicalProgram()},
      plant_(plant),
      owned_context_(plant_.CreateDefaultContext()),
      context_(owned_context_.get()),
      q_(prog_->NewContinuousVariables(plant_.num_positions(), "q")) {
  if (with_joint_limits) {
    prog_->AddBoundingBoxConstraint(plant.GetPositionLowerLimits(),
                                    plant.GetPositionUpperLimits(), q_);
  }
  AddUnitQuaternionConstraintOnPlant(plant, q_, prog_.get());
}

InverseKinematics::InverseKinematics(const MultibodyPlant<double>& plant,
                                     systems::Context<double>* plant_context,
                                     bool with_joint_limits)
    : prog_{new solvers::MathematicalProgram()},
      plant_(plant),
      owned_context_(nullptr),
      context_(plant_context),
      q_(prog_->NewContinuousVariables(plant.num_positions(), "q")) {
  DRAKE_DEMAND(plant_context != nullptr);
  if (with_joint_limits) {
    prog_->AddBoundingBoxConstraint(plant.GetPositionLowerLimits(),
                                    plant.GetPositionUpperLimits(), q_);
  }
  AddUnitQuaternionConstraintOnPlant(plant, q_, prog_.get());
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
InverseKinematics::AddMinimumDistanceConstraint(
    double minimum_distance, double influence_distance_offset) {
  auto constraint =
      std::shared_ptr<MinimumDistanceConstraint>(new MinimumDistanceConstraint(
          &plant_, minimum_distance, get_mutable_context(), {},
          influence_distance_offset));
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
