#include "drake/multibody/inverse_kinematics/add_multibody_plant_constraints.h"

#include <memory>

#include "drake/multibody/inverse_kinematics/orientation_constraint.h"
#include "drake/multibody/inverse_kinematics/point_to_point_distance_constraint.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"

namespace drake {
namespace multibody {

using solvers::Binding;
using solvers::Constraint;

std::vector<Binding<Constraint>> AddMultibodyPlantConstraints(
    const MultibodyPlant<double>& plant,
    const solvers::VectorXDecisionVariable& q,
    solvers::MathematicalProgram* prog,
    systems::Context<double>* plant_context) {
  DRAKE_THROW_UNLESS(prog != nullptr);
  if (plant_context) {
    plant.ValidateContext(*plant_context);
  }
  // AddMultibodyPlantConstraints only supports coupler, ball, distance, and
  // weld.
  int num_supported_constraints =
      plant.num_coupler_constraints() + plant.num_ball_constraints() +
      plant.num_distance_constraints() + plant.num_weld_constraints();
  if (num_supported_constraints != plant.num_constraints()) {
    // TODO(russt): It would be better to say something specific about the
    // unidentified constraint here, but the constraint API for MultibodyPlant
    // does not have that surface area (yet).
    throw std::runtime_error(fmt::format(
        "plant.num_constraints() = {}, but only AddMultibodyPlantConstraints() "
        "only captured {} of them. This method currently supports Coupler, "
        "Distance, Ball, and Weld multibody constraints. If you're seeing "
        "this, MultibodyPlant must have gained a new constraint type that is "
        "not supported here yet (but likely should be)",
        plant.num_constraints(), num_supported_constraints));
  }
  std::vector<Binding<Constraint>> bindings;

  // Joint limits.
  const int nq = plant.num_positions();
  Eigen::VectorXd lb = plant.GetPositionLowerLimits();
  Eigen::VectorXd ub = plant.GetPositionUpperLimits();

  // Joint locking.
  VectorX<bool> is_locked = VectorX<bool>::Constant(nq, false);
  Eigen::VectorXd current_positions(nq);
  if (plant_context) {
    current_positions = plant.GetPositions(*plant_context);
    for (JointIndex i : plant.GetJointIndices()) {
      const Joint<double>& joint = plant.get_joint(i);
      if (joint.is_locked(*plant_context)) {
        const int start = joint.position_start();
        const int size = joint.num_positions();
        lb.segment(start, size) = current_positions.segment(start, size);
        ub.segment(start, size) = current_positions.segment(start, size);
        is_locked.segment(start, size).array() = true;
      }
    }
  }

  // Add the unit quaternion constraints.
  for (BodyIndex i{0}; i < plant.num_bodies(); ++i) {
    const RigidBody<double>& body = plant.get_body(i);
    if (body.has_quaternion_dofs()) {
      const int start = body.floating_positions_start();
      constexpr int kSize = 4;
      if (plant_context && is_locked.segment<kSize>(start).any()) {
        // Sanity check the MultibodyTree invariant.
        DRAKE_DEMAND(is_locked.segment<kSize>(start).all());
        // Lock to the normalized value, in lieu of a unit norm constraint.
        const Eigen::Vector4d quat =
            current_positions.segment<kSize>(start).normalized();
        lb.segment<kSize>(start) = quat;
        ub.segment<kSize>(start) = quat;
        prog->SetInitialGuess(q.segment<kSize>(start), quat);
      } else {
        // TODO(russt): Probably the joint limits should be [-1, 1] coming
        // right out of the MultibodyPlant.
        lb.segment<kSize>(start) = -Eigen::Vector4d::Ones();
        ub.segment<kSize>(start) = Eigen::Vector4d::Ones();
        bindings
            .emplace_back(
                prog->AddConstraint(solvers::Binding<solvers::Constraint>(
                    std::make_shared<UnitQuaternionConstraint>(),
                    q.segment<kSize>(start))))
            .evaluator()
            ->set_description(fmt::format(
                "Unit quaternion constraint for body {}", body.name()));
        prog->SetInitialGuess(q.segment<kSize>(start),
                              Eigen::Vector4d{1, 0, 0, 0});
      }
    }
  }
  bindings.emplace_back(prog->AddBoundingBoxConstraint(lb, ub, q))
      .evaluator()
      ->set_description("Joint limits");

  for (const auto& [id, spec] : plant.get_coupler_constraint_specs()) {
    const int q0_index = plant.get_joint(spec.joint0_index).position_start();
    const int q1_index = plant.get_joint(spec.joint1_index).position_start();
    bindings
        .emplace_back(prog->AddLinearEqualityConstraint(
            q[q0_index] == spec.gear_ratio * q[q1_index] + spec.offset))
        .evaluator()
        ->set_description(
            fmt::format("Coupler constraint for joint {} and joint {}",
                        spec.joint0_index, spec.joint1_index));
  }
  for (const auto& [id, spec] : plant.get_distance_constraint_specs()) {
    DRAKE_THROW_UNLESS(plant_context != nullptr);
    // d(q) == d₀.
    bindings
        .emplace_back(prog->AddConstraint(
            std::make_shared<PointToPointDistanceConstraint>(
                &plant, plant.get_body(spec.body_A).body_frame(), spec.p_AP,
                plant.get_body(spec.body_B).body_frame(), spec.p_BQ,
                spec.distance, spec.distance, plant_context),
            q))
        .evaluator()
        ->set_description(
            fmt::format("Distance constraint between body {} and body {}",
                        spec.body_A, spec.body_B));
  }
  for (const auto& [id, spec] : plant.get_ball_constraint_specs()) {
    DRAKE_THROW_UNLESS(plant_context != nullptr);
    bindings
        .emplace_back(prog->AddConstraint(
            std::make_shared<PositionConstraint>(
                &plant, plant.get_body(spec.body_A).body_frame(), spec.p_AP,
                spec.p_AP, plant.get_body(spec.body_B).body_frame(), *spec.p_BQ,
                plant_context),
            q))
        .evaluator()
        ->set_description(
            fmt::format("Ball constraint between body {} and body {}",
                        spec.body_A, spec.body_B));
  }
  for (const auto& [id, spec] : plant.get_weld_constraint_specs()) {
    DRAKE_THROW_UNLESS(plant_context != nullptr);
    // TODO(russt): Consider implementing a WeldConstraint.
    bindings
        .emplace_back(prog->AddConstraint(
            std::make_shared<PositionConstraint>(
                &plant, plant.get_body(spec.body_A).body_frame(),
                spec.X_AP.translation(), spec.X_AP.translation(),
                plant.get_body(spec.body_B).body_frame(),
                spec.X_BQ.translation(), plant_context),
            q))
        .evaluator()
        ->set_description(
            fmt::format("Weld position constraint between body {} and body {}",
                        spec.body_A, spec.body_B));
    bindings
        .emplace_back(prog->AddConstraint(
            std::make_shared<OrientationConstraint>(
                &plant, plant.get_body(spec.body_A).body_frame(),
                spec.X_AP.rotation(), plant.get_body(spec.body_B).body_frame(),
                spec.X_BQ.rotation(), 0.0, plant_context),
            q))
        .evaluator()
        ->set_description(fmt::format(
            "Weld orientation constraint between body {} and body {}",
            spec.body_A, spec.body_B));
  }
  return bindings;
}

}  // namespace multibody
}  // namespace drake
