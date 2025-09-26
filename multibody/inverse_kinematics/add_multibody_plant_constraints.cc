#include "drake/multibody/inverse_kinematics/add_multibody_plant_constraints.h"

#include <utility>

#include "drake/multibody/inverse_kinematics/orientation_constraint.h"
#include "drake/multibody/inverse_kinematics/point_to_point_distance_constraint.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"

namespace drake {
namespace multibody {

using solvers::Binding;
using solvers::Constraint;

namespace {
// Given a binding that requires the plant to be kept alive, modifies the
// binding to incorporate a shared_ptr to the given plant.
// TODO(jwnimmer-tri) Ideally the constraints themselves would accept a
// shared_ptr to the plant instead of a raw pointer.
void AddPlantLifetimeToBinding(
    const std::shared_ptr<const MultibodyPlant<double>>& plant,
    Binding<Constraint>* binding) {
  DRAKE_DEMAND(binding != nullptr);
  // Obtain the binding's constraint as it exists now.
  const std::shared_ptr<Constraint> old_constraint_shared =
      binding->evaluator();
  Constraint* old_constraint_raw = old_constraint_shared.get();
  // For the binding to remain valid, we need to ensure that both the MbP and
  // the constraint live at least as long as the binding. We can accomplish that
  // by keeping each of their shared_ptrs alive (as a pair, for convenience).
  std::shared_ptr<void> keep_alives =
      std::make_shared<std::pair<std::shared_ptr<const MultibodyPlant<double>>,
                                 std::shared_ptr<Constraint>>>(
          plant, old_constraint_shared);
  // Create a shared_ptr whose `get()` still points to the constraint, but whose
  // control block now keeps alive *both* the the plant and the constraint.
  std::shared_ptr<Constraint> new_constraint_shared(
      /* managed object = */ std::move(keep_alives),
      /* stored pointer = */ old_constraint_raw);
  // Replace the binding's constraint with our new, fatter one.
  *binding = Binding<Constraint>(std::move(new_constraint_shared),
                                 binding->variables());
}
}  // namespace

std::vector<Binding<Constraint>> AddMultibodyPlantConstraints(
    const std::shared_ptr<const MultibodyPlant<double>>& plant,
    const solvers::VectorXDecisionVariable& q,
    solvers::MathematicalProgram* prog,
    systems::Context<double>* plant_context) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(prog != nullptr);
  const MultibodyPlant<double>& plant_ref = *plant;
  if (plant_context) {
    plant_ref.ValidateContext(*plant_context);
  }
  // AddMultibodyPlantConstraints only supports coupler, ball, distance, and
  // weld.
  int num_supported_constraints =
      plant_ref.num_coupler_constraints() + plant_ref.num_ball_constraints() +
      plant_ref.num_distance_constraints() + plant_ref.num_weld_constraints();
  if (num_supported_constraints != plant_ref.num_constraints()) {
    // TODO(russt): It would be better to say something specific about the
    // unidentified constraint here, but the constraint API for MultibodyPlant
    // does not have that surface area (yet).
    throw std::runtime_error(fmt::format(
        "plant.num_constraints() = {}, but only AddMultibodyPlantConstraints() "
        "only captured {} of them. This method currently supports Coupler, "
        "Distance, Ball, and Weld multibody constraints. If you're seeing "
        "this, MultibodyPlant must have gained a new constraint type that is "
        "not supported here yet (but likely should be)",
        plant_ref.num_constraints(), num_supported_constraints));
  }
  std::vector<Binding<Constraint>> bindings;

  // Joint limits.
  const int nq = plant_ref.num_positions();
  Eigen::VectorXd lb = plant_ref.GetPositionLowerLimits();
  Eigen::VectorXd ub = plant_ref.GetPositionUpperLimits();

  // Joint locking.
  VectorX<bool> is_locked = VectorX<bool>::Constant(nq, false);
  Eigen::VectorXd current_positions(nq);
  if (plant_context) {
    current_positions = plant_ref.GetPositions(*plant_context);
    for (JointIndex i : plant_ref.GetJointIndices()) {
      const Joint<double>& joint = plant_ref.get_joint(i);
      if (joint.is_locked(*plant_context)) {
        const int start = joint.position_start();
        const int size = joint.num_positions();
        lb.segment(start, size) = current_positions.segment(start, size);
        ub.segment(start, size) = current_positions.segment(start, size);
        is_locked.segment(start, size).array() = true;
        prog->SetInitialGuess(q.segment(start, size),
                              current_positions.segment(start, size));
      }
    }
  }

  // Add the unit quaternion constraints.
  for (JointIndex joint_index : plant_ref.GetJointIndices()) {
    const Joint<double>& joint = plant->get_joint(joint_index);
    if (joint.type_name() == QuaternionFloatingJoint<double>::kTypeName) {
      const int start = joint.position_start();
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
                "Unit quaternion constraint for joint {}", joint.name()));
        prog->SetInitialGuess(q.segment<kSize>(start),
                              Eigen::Vector4d{1, 0, 0, 0});
      }
    }
  }
  bindings.emplace_back(prog->AddBoundingBoxConstraint(lb, ub, q))
      .evaluator()
      ->set_description("Joint limits");

  for (const auto& [id, spec] : plant_ref.get_coupler_constraint_specs()) {
    const int q0_index =
        plant_ref.get_joint(spec.joint0_index).position_start();
    const int q1_index =
        plant_ref.get_joint(spec.joint1_index).position_start();
    bindings
        .emplace_back(prog->AddLinearEqualityConstraint(
            q[q0_index] == spec.gear_ratio * q[q1_index] + spec.offset))
        .evaluator()
        ->set_description(
            fmt::format("Coupler constraint for joint {} and joint {}",
                        spec.joint0_index, spec.joint1_index));
  }

  if (plant_context != nullptr) {
    for (const auto& [id, params] :
         plant_ref.GetDistanceConstraintParams(*plant_context)) {
      // d(q) == d₀.
      bindings
          .emplace_back(prog->AddConstraint(
              std::make_shared<PointToPointDistanceConstraint>(
                  &plant_ref, plant_ref.get_body(params.bodyA()).body_frame(),
                  params.p_AP(),
                  plant_ref.get_body(params.bodyB()).body_frame(),
                  params.p_BQ(), params.distance(), params.distance(),
                  plant_context),
              q))
          .evaluator()
          ->set_description(
              fmt::format("Distance constraint between body {} and body {}",
                          params.bodyA(), params.bodyB()));
      AddPlantLifetimeToBinding(plant, &bindings.back());
    }
  }
  for (const auto& [id, spec] : plant_ref.get_ball_constraint_specs()) {
    DRAKE_THROW_UNLESS(plant_context != nullptr);
    bindings
        .emplace_back(prog->AddConstraint(
            std::make_shared<PositionConstraint>(
                &plant_ref, plant_ref.get_body(spec.body_A).body_frame(),
                spec.p_AP, spec.p_AP,
                plant_ref.get_body(spec.body_B).body_frame(), *spec.p_BQ,
                plant_context),
            q))
        .evaluator()
        ->set_description(
            fmt::format("Ball constraint between body {} and body {}",
                        spec.body_A, spec.body_B));
    AddPlantLifetimeToBinding(plant, &bindings.back());
  }
  for (const auto& [id, spec] : plant_ref.get_weld_constraint_specs()) {
    DRAKE_THROW_UNLESS(plant_context != nullptr);
    // TODO(russt): Consider implementing a WeldConstraint.
    bindings
        .emplace_back(prog->AddConstraint(
            std::make_shared<PositionConstraint>(
                &plant_ref, plant_ref.get_body(spec.body_A).body_frame(),
                spec.X_AP.translation(), spec.X_AP.translation(),
                plant_ref.get_body(spec.body_B).body_frame(),
                spec.X_BQ.translation(), plant_context),
            q))
        .evaluator()
        ->set_description(
            fmt::format("Weld position constraint between body {} and body {}",
                        spec.body_A, spec.body_B));
    AddPlantLifetimeToBinding(plant, &bindings.back());
    bindings
        .emplace_back(prog->AddConstraint(
            std::make_shared<OrientationConstraint>(
                &plant_ref, plant_ref.get_body(spec.body_A).body_frame(),
                spec.X_AP.rotation(),
                plant_ref.get_body(spec.body_B).body_frame(),
                spec.X_BQ.rotation(), 0.0, plant_context),
            q))
        .evaluator()
        ->set_description(fmt::format(
            "Weld orientation constraint between body {} and body {}",
            spec.body_A, spec.body_B));
    AddPlantLifetimeToBinding(plant, &bindings.back());
  }
  return bindings;
}

}  // namespace multibody
}  // namespace drake
