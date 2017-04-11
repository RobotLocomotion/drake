#pragma once

#include <string>

#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/generic_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A concrete plan that sets up a Cartesian tracking objective for the end
 * effector assuming no contacts. The joint space tracking objective is set
 * to track the joint position when this plan is first initialized to.
 * Note that joint position tracking can be turned off by setting the
 * position gains to zero in the parameters passed to UpdateQpInput.
 */
template <typename T>
class ManipulatorMoveEndEffectorPlan : public GenericPlan<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManipulatorMoveEndEffectorPlan<T>)

  ManipulatorMoveEndEffectorPlan() {}

  static constexpr char* kEndEffectorAliasGroupName = "flange";

 protected:
  /**
   * A desired stationary Cartesian spline is set up for the body whose alias
   * in @p alias_groups is kEndEffectorAliasGroupName.
   */
  void InitializeGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  /**
   * This function assumes that the bytes in @p message_bytes encodes a valid
   * lcmt_manipulator_plan_move_end_effector message, from which the desired
   * poses (x_i) for the end effector and timing (t_i) are extracted and used
   * to construct a Cartesian spline. Let t_now be the time in @p robot_stauts.
   * If t_0 = 0 (first time stamp in @p message_bytes is zero), the spline is
   * constructed with
   * MakeCubicLinearWithEndLinearVelocity(t_i + t_now, x_i, 0, 0), which starts
   * immediately from x_0 regardless of what's the current planned desired
   * pose and velocity. This introduces a discontinuity in the interpolated
   * desired position and velocity when interpolating, which can cause large
   * discontinuity in output and instability on robot.
   * If t_0 > 0, the spline is constructed with
   * MakeCubicLinearWithEndLinearVelocity({0, t_i} + t_now,
   * {x_d_now, x_i}, xd_d_now, 0),
   * where x_d_now and xd_d_now are the current desired pose and velocity of
   * the end effector. This results in a smoother transition to the new plan.
   */
  void HandlePlanMessageGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length);

 private:
  GenericPlan<T>* CloneGenericPlanDerived() const;

  void ModifyPlanGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {}

  void UpdateQpInputGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      QpInput* qp_input) const {}
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
