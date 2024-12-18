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
  std::vector<Binding<Constraint>> bindings =
      AddUnitQuaternionConstraintOnPlant(plant, q, prog);
  int num_multibody_constraints = 0;
  for (const auto& [id, spec] : plant.get_coupler_constraint_specs()) {
    bindings.emplace_back(prog->AddLinearEqualityConstraint(
        q[spec.joint0_index] ==
        spec.gear_ratio * q[spec.joint1_index] + spec.offset));
    ++num_multibody_constraints;
  }
  for (const auto& [id, spec] : plant.get_distance_constraint_specs()) {
    DRAKE_THROW_UNLESS(plant_context != nullptr);
    // d(q) == d₀.
    bindings.emplace_back(prog->AddConstraint(
        std::make_shared<PointToPointDistanceConstraint>(
            &plant, plant.get_body(spec.body_A).body_frame(), spec.p_AP,
            plant.get_body(spec.body_B).body_frame(), spec.p_BQ, spec.distance,
            spec.distance, plant_context),
        q));
    ++num_multibody_constraints;
  }
  for (const auto& [id, spec] : plant.get_ball_constraint_specs()) {
    DRAKE_THROW_UNLESS(plant_context != nullptr);
    bindings.emplace_back(prog->AddConstraint(
        std::make_shared<PositionConstraint>(
            &plant, plant.get_body(spec.body_A).body_frame(), spec.p_AP,
            spec.p_AP, plant.get_body(spec.body_B).body_frame(), *spec.p_BQ,
            plant_context),
        q));
    ++num_multibody_constraints;
  }
  for (const auto& [id, spec] : plant.get_weld_constraint_specs()) {
    DRAKE_THROW_UNLESS(plant_context != nullptr);
    // TODO(russt): Consider implementing a WeldConstraint.
    bindings.emplace_back(prog->AddConstraint(
        std::make_shared<PositionConstraint>(
            &plant, plant.get_body(spec.body_A).body_frame(),
            spec.X_AP.translation(), spec.X_AP.translation(),
            plant.get_body(spec.body_B).body_frame(), spec.X_BQ.translation(),
            plant_context),
        q));
    bindings.emplace_back(prog->AddConstraint(
        std::make_shared<OrientationConstraint>(
            &plant, plant.get_body(spec.body_A).body_frame(),
            spec.X_AP.rotation(), plant.get_body(spec.body_B).body_frame(),
            spec.X_BQ.rotation(), 0.0, plant_context),
        q));
    ++num_multibody_constraints;
  }
  if (num_multibody_constraints != plant.num_constraints()) {
    // TODO(russt): It would be better to say something specific about the
    // unidentified constraint here, but the constraint API for MultibodyPlant
    // does not have that surface area (yet).
    log()->warn(
        "plant.num_constraints() = {}, but only AddMultibodyPlantConstraints() "
        "only captured {} of them. This method currently supports Coupler, "
        "Distance, Ball, and Weld multibody constraints. If you're seeing "
        "this, MultibodyPlant may have gained a new constraint type that is "
        "not supported here yet (but perhaps should be)",
        plant.num_constraints(), num_multibody_constraints);
  }
  return bindings;
}

}  // namespace multibody
}  // namespace drake
