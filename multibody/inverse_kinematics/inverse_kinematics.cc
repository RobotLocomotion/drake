#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

#include <limits>
#include <utility>

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

constexpr double kInf = std::numeric_limits<double>::infinity();

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

  // We're about to add constraints for position limits (if requested), locked
  // joints, and quaternions (unit norm). When a quaternion is locked, we'll use
  // the lock constraint instead of the unit norm constraint.

  // Obey the joint limits (if requested).
  const int nq = plant.num_positions();
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  if (with_joint_limits) {
    lb = plant.GetPositionLowerLimits();
    ub = plant.GetPositionUpperLimits();
  } else {
    lb = Eigen::VectorXd::Constant(nq, -kInf);
    ub = Eigen::VectorXd::Constant(nq, +kInf);
  }

  // Obey joint locking. Joint locking trumps joint limits.
  const Eigen::VectorXd current_positions = plant.GetPositions(context());
  VectorX<bool> is_locked = VectorX<bool>::Constant(nq, false);
  for (JointIndex i{0}; i < plant.num_joints(); ++i) {
    const Joint<double>& joint = plant.get_joint(i);
    if (joint.is_locked(context())) {
      const int start = joint.position_start();
      const int size = joint.num_positions();
      lb.segment(start, size) = current_positions.segment(start, size);
      ub.segment(start, size) = current_positions.segment(start, size);
      is_locked.segment(start, size).array() = true;
    }
  }

  // Add the unit quaternion constraints.
  for (BodyIndex i{0}; i < plant.num_bodies(); ++i) {
    const RigidBody<double>& body = plant.get_body(i);
    if (body.has_quaternion_dofs()) {
      const int start = body.floating_positions_start();
      constexpr int size = 4;
      if (is_locked.segment<size>(start).any()) {
        // Sanity check the MultibodyTree invariant.
        DRAKE_DEMAND(is_locked.segment<size>(start).all());
        // Lock to the normalized value, in lieu of a unit norm constraint.
        const Eigen::Vector4d quat =
            current_positions.segment<size>(start).normalized();
        lb.segment<size>(start) = quat;
        ub.segment<size>(start) = quat;
        prog_->SetInitialGuess(q_.segment<size>(start), quat);
      } else {
        prog_->AddConstraint(solvers::Binding<solvers::Constraint>(
            std::make_shared<UnitQuaternionConstraint>(),
            q_.segment<size>(start)));
        // Set a non-zero initial guess to help avoid singularities.
        prog_->SetInitialGuess(q_.segment<size>(start),
                              Eigen::Vector4d{1, 0, 0, 0});
      }
    }
  }

  // Now we can finally add the bbox constraint for joint limits and locking.
  prog_->AddBoundingBoxConstraint(lb, ub, q_);
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
