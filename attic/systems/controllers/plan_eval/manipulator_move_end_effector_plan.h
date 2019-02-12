#pragma once

#include <string>

#include "drake/systems/controllers/plan_eval/generic_plan.h"

namespace drake {
namespace systems {
namespace controllers {
namespace plan_eval {

/**
 * A concrete plan that sets up a Cartesian tracking objective for the end
 * effector assuming no contacts. The joint space tracking objective is set
 * to track the measured joint position when Initialize() is called.
 * Note that joint position tracking can be turned off by setting the
 * position gains to zero in the parameters passed to UpdateQpInput.
 */
template <typename T>
class ManipulatorMoveEndEffectorPlan : public GenericPlan<T> {
 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ManipulatorMoveEndEffectorPlan)

 public:
  ManipulatorMoveEndEffectorPlan() {}

  static constexpr const char* kEndEffectorAliasGroupName = "flange";

 protected:
  /**
   * A desired stationary Cartesian spline is set up for the body whose alias
   * in @p alias_groups is kEndEffectorAliasGroupName.
   */
  void InitializeGenericPlanDerived(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
          robot_status,
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups) override;

  /**
   * This function assumes that the bytes in @p plan encodes a valid
   * lcmt_manipulator_plan_move_end_effector plan, from which the desired
   * poses (x_i) for the end effector and timing (t_i) are extracted and used
   * to construct a Cartesian spline. Let t_now be the time in @p robot_stauts.
   * If t_0 = 0 (first time stamp in @p plan is zero), the spline is
   * constructed with
   * MakeCubicLinearWithEndLinearVelocity(t_i + t_now, x_i, 0, 0), which starts
   * immediately from x_0 regardless of what's the current planned desired
   * pose and velocity. This introduces a discontinuity in the interpolated
   * desired position and velocity when interpolating, which can cause a larger
   * discontinuity (due to gains) in output and instability on robot.
   * If t_0 > 0, the spline is constructed with
   * MakeCubicLinearWithEndLinearVelocity({0, t_i} + t_now,
   * {x_d_now, x_i}, xd_d_now, 0),
   * where x_d_now and xd_d_now are the current desired pose and velocity of
   * the end effector. This results in a smoother transition to the new plan.
   *
   * @throws std::bad_cast if @p plan is not of type
   * lcmt_manipulator_plan_move_end_effector;
   */
  void HandlePlanGenericPlanDerived(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
          robot_stauts,
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups,
      const AbstractValue& plan) override;

 private:
  GenericPlan<T>* CloneGenericPlanDerived() const override;

  void ModifyPlanGenericPlanDerived(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&,
      const systems::controllers::qp_inverse_dynamics::ParamSet&,
      const RigidBodyTreeAliasGroups<T>&) override {}

  void UpdateQpInputGenericPlanDerived(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&,
      const systems::controllers::qp_inverse_dynamics::ParamSet&,
      const RigidBodyTreeAliasGroups<T>&,
      systems::controllers::qp_inverse_dynamics::QpInput*) const override {}
};

}  // namespace plan_eval
}  // namespace controllers
}  // namespace systems
}  // namespace drake
