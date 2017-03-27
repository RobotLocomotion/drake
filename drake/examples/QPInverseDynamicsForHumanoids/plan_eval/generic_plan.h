#pragma once

#include <memory>
#include <set>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/param_parser.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/rigid_body_tree_alias_groups.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/plan_eval_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * This class represents a behavior, which conceptually serves as a bridge
 * between a high level planner (e.g. A* planner) and a low level
 * controller (e.g. PID controller). It is responsible for generating high
 * frequency / dense commands that are compatible with the low level controller
 * from the behavioral inputs. Here is a concrete example: A motion planner
 * generates a list of desired joint configurations (behavioral inputs) to be
 * tracked. The low level controller is a PID controller which takes joint
 * position and velocity set points and outputs joint torques. A simple behavior
 * can use splines to represent smooth motions through those desired
 * configurations, from which dense position and velocity set points are
 * interpolated and sent to the PID controller at high rate.
 *
 * For this class, the low level controller of choice is QPController. So the
 * output of this class is an QpInput. The behavioral level inputs are assumed
 * to be passed as Lcm messages. Because there is no common base class for Lcm
 * messages, raw bytes are passed in instead. Derived classes are responsible
 * for decoding those into proper messages types.
 */
template <typename T>
class GenericPlan {
 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GenericPlan)

 public:

  GenericPlan() {}

  /**
   * Returns a unique pointer to a copy of this instance. Derived classes need
   * to implement CloneGenericPlanDerived() to copy custom internal fields.
   */
  std::unique_ptr<GenericPlan<T>> Clone() const;

  virtual ~GenericPlan() {}

  /**
   * Initializes this plan. This function sets a plan to main the current joint
   * position in @p robot_status. It assumes no contacts and no Cartesian
   * tracking objectives are present. Derived classes can implement custom
   * behaviors or override these in InitializeGenericPlanDerived().
   * @param robot_status Current status of the robot.
   * @param paramset Parameters.
   * @param alias_groups Topological information of the robot.
   */
  void Initialize(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  /**
   * Control logic should be implemented in this function. This function is
   * intended to be called in a main loop. All internal state mutations (e.g.
   * state machine transition, reference trajectory generation, etc) should
   * be done here. Derived classes need to implement custom behaviors in
   * ExecutePlanGenericPlanDerived().
   * @param robot_status Current status of the robot.
   * @param paramset Parameters.
   * @param alias_groups Topological information of the robot.
   */
  void ExecutePlan(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  /**
   * Handles a discrete command in the form of a Lcm message (e.g. footstep
   * plan for a walking controller or a sequence of joint angles for a
   * manipulator). Derived classes need to implement custom behaviors in
   * HandlePlanMessageGenericPlanDerived().
   * @param robot_status Current status of the robot.
   * @param paramset Parameters.
   * @param alias_groups Topological information of the robot.
   * @param message_bytes Pointer to the raw message message. Derived classes
   * need to decode this explicitly.
   * @param message_length Number of bytes in the message.
   */
  void HandlePlanMessage(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length);

  /**
   * Updates @p qp_input given the current state of the plan and measured robot
   * state in @p robot_status. Specifically, this function performs the
   * following:
   * <pre>
   * 1. Generates all ContactInformation for each body that are planned to be in
   * contact.
   * 2. For all degrees of freedom, desired accelerations are computed by:
   * `vd_d = kp * (q* - q) + kd * (v* - v) + vd*`, where `q*, v*` and `vd*` are
   * interpolated from the planned trajectory, `q, v` are from @p robot_status,
   * and kp and kd are looked up from @p paramset.
   * 3. For all bodies that have a Cartesian tracking objective, desired
   * Cartesian accelerations are computed by:
   * `xd_d = kp * (x* - x) + kd * (xd* - xd) + xdd*`, where `x*, xd*, xdd*` are
   * interpolated from the planned trajectories, `x, xd` are from
   * @p robot_status, and kp and kd are looked up from @p paramset.
   * </pre>
   * Derived classes can implement custom behaviors or override these in
   * UpdateQpInputGenericPlanDerived().
   * @param robot_status Current status of the robot.
   * @param paramset Parameters.
   * @param alias_groups Topological information of the robot.
   * @param[out] qp_input Output for QpInput.
   */
  void UpdateQpInput(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      QpInput* qp_input) const;

  /**
   * Returns the current planned contact state.
   */
  const ContactState& get_contact_state() const { return contact_state_; }

  /**
   * Returns a map of all Cartesian trajectories.
   */
  const std::unordered_map<const RigidBody<T>*,
                           PiecewiseCartesianTrajectory<T>>&
  get_body_trajectories() const {
    return body_trajectories_;
  }

  /**
   * Returns the Cartesian trajectory for @p body.
   */
  const PiecewiseCartesianTrajectory<T>& get_body_trajectory(
      const RigidBody<T>* body) const {
    return body_trajectories_.at(body);
  }

  /**
   * Returns trajectory for all degrees of freedom.
   */
  const PiecewiseCubicTrajectory<T>& get_dof_trajectory() const {
    return dof_trajectory_;
  }

  /**
   * Returns true if there is a Cartesian trajectory for @p body.
   */
  const bool has_body_trajectory(const RigidBody<T>* body) const {
    return body_trajectories_.find(body) != body_trajectories_.end();
  }

 protected:
  /**
   * Custom fields can be cloned here.
   */
  virtual GenericPlan<T>* CloneGenericPlanDerived() const = 0;

  /**
   * Custom initialization can be implemented here.
   */
  virtual void InitializeGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) = 0;

  /**
   * Custom state mutation can be implemented here.
   */
  virtual void ExecutePlanGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) = 0;

  /**
   * Custom message handling can be implemented here.
   */
  virtual void HandlePlanMessageGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length) = 0;

  /**
   * Custom QpInput updates can be implemented here.
   */
  virtual void UpdateQpInputGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      QpInput* qp_input) const = 0;

  /**
   * Sets the planned contact state to @p contact_state.
   */
  void UpdateContactState(const ContactState& contact_state) {
    contact_state_ = contact_state;
  }

  /**
   * Set a Cartesian trajectory for @p body. Replaces the existing one if
   * it exists.
   */
  void set_body_trajectory(const RigidBody<T>* body,
                           const PiecewiseCartesianTrajectory<T>& traj) {
    auto it = body_trajectories_.find(body);
    if (it != body_trajectories_.end()) {
      body_trajectories_.erase(it);
    }
    body_trajectories_.emplace(body, traj);
  }

  /**
   * Removes a Cartesian trajectory for @p body.
   */
  const void remove_body_trajectory(const RigidBody<T>* body) {
    body_trajectories_.erase(body);
  }

  /**
   * Sets dof trajectory to @p traj.
   */
  void set_dof_trajectory(const PiecewiseCubicTrajectory<T>& traj) {
    dof_trajectory_ = traj;
  }

 private:
  // Planned set of bodies that are in contact.
  ContactState contact_state_;
  // Trajectory for all dof.
  PiecewiseCubicTrajectory<T> dof_trajectory_;
  // Trajectories for all bodies that have Cartesian tracking objectives.
  std::unordered_map<const RigidBody<T>*, PiecewiseCartesianTrajectory<T>>
      body_trajectories_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
