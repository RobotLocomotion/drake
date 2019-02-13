#pragma once

#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/value.h"
#include "drake/manipulation/util/trajectory_utils.h"
#include "drake/multibody/rigid_body_tree_alias_groups.h"
#include "drake/systems/controllers/qp_inverse_dynamics/param_parser.h"
#include "drake/systems/controllers/qp_inverse_dynamics/qp_inverse_dynamics_common.h"
#include "drake/systems/controllers/qp_inverse_dynamics/robot_kinematic_state.h"

namespace drake {
namespace systems {
namespace controllers {
namespace plan_eval {

/**
 * ContactState is intended to represent a set of bodies that are in contact.
 * Detailed information such as contact points, contact wrench are not included
 * here.
 */
typedef std::unordered_set<const RigidBody<double>*> ContactState;

/**
 * This class represents a plan interpretor, which conceptually serves as a
 * bridge between a high level planner (e.g. A* planner) and a low level
 * controller (e.g. PID controller). It is responsible for generating high
 * frequency / dense commands that are compatible with the low level controller
 * from the behavioral inputs. One main advantage for this separation is that
 * the planner does not need to worry about any stabilization or realtime
 * requirements associated with the hardware. This class also provides an
 * interface for simple reactive behaviors such as "move arm in a straight line
 * until the hand touches something", which requires little computation but
 * high rate feedback.
 * Here is a concrete example: suppose the robot is a position and velocity
 * controlled manipulator arm, and a motion planner is used to generate a
 * sequence of joint configurations for the robot to follow. A simple
 * implementation can use splines to represent smooth motions through
 * those desired configurations, from which dense position and velocity set
 * points are interpolated and sent to the PID controller at high rate.
 *
 * For this class, the low level controller of choice is QPController. So the
 * output of this class is an QpInput. The behavioral level inputs are type
 * erased and stored in AbstractValue. Derived classes are responsible
 * for recovering the proper plan type.
 */
template <typename T>
class GenericPlan {
 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GenericPlan)

 public:
  /**
   * Returns a unique pointer to a copy of this instance. Derived classes need
   * to implement CloneGenericPlanDerived() to copy custom internal fields.
   */
  std::unique_ptr<GenericPlan<T>> Clone() const;

  virtual ~GenericPlan() {}

  /**
   * Initializes this plan. This function sets a dummy plan to main the current
   * joint configuration in @p robot_status. It assumes no contacts and no
   * Cartesian tracking objectives are present. Derived classes can implement
   * custom behaviors or override these in InitializeGenericPlanDerived().
   * @param robot_status Current status of the robot.
   * @param paramset Parameters.
   * @param alias_groups Topological information of the robot.
   *
   * @throws std::logic_error if the RigidBodyTree reference in @p robot_status
   * or @p paramset or @p alias_groups is incompatible.
   */
  void Initialize(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
          robot_status,
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups);

  /**
   * Control logic should be implemented in this function. This function is
   * intended to be called in a main loop. All internal state mutations (e.g.
   * state machine transition, reference trajectory generation, etc) should
   * be done here. Derived classes need to implement custom behaviors in
   * ModifyPlanGenericPlanDerived().
   * @param robot_status Current status of the robot.
   * @param paramset Parameters.
   * @param alias_groups Topological information of the robot.
   *
   * @throws std::logic_error if the RigidBodyTree reference in @p robot_status
   * or @p paramset or @p alias_groups is incompatible.
   */
  void ModifyPlan(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
          robot_status,
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups) {
    // Checks parameters and throw if they are incompatible.
    CheckCompatibilityAndThrow(robot_status.get_robot(), paramset,
                               alias_groups);
    ModifyPlanGenericPlanDerived(robot_status, paramset, alias_groups);
  }

  /**
   * Handles a discrete plan (e.g. footstep plan for a walking controller or
   * a sequence of joint angles for a manipulator). Derived classes need to
   * implement custom behaviors in HandlePlanGenericPlanDerived().
   * @param robot_status Current status of the robot.
   * @param paramset Parameters.
   * @param alias_groups Topological information of the robot.
   * @param plan AbstractValue that contains the plan.
   *
   * @throws std::logic_error if the RigidBodyTree reference in @p robot_status
   * or @p paramset or @p alias_groups is incompatible.
   */
  void HandlePlan(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
          robot_status,
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups,
      const AbstractValue& plan) {
    // Checks parameters and throw if they are incompatible.
    CheckCompatibilityAndThrow(robot_status.get_robot(), paramset,
                               alias_groups);
    HandlePlanGenericPlanDerived(robot_status, paramset, alias_groups, plan);
  }

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
   *
   * @throws std::logic_error if the RigidBodyTree reference in @p robot_status
   * or @p paramset or @p alias_groups is incompatible.
   */
  void UpdateQpInput(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
          robot_status,
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups,
      systems::controllers::qp_inverse_dynamics::QpInput* qp_input) const;

  /**
   * Returns the current planned contact state.
   */
  const ContactState& get_planned_contact_state() const {
    return contact_state_;
  }

  /**
   * Returns a map of all Cartesian trajectories.
   */
  const std::unordered_map<const RigidBody<T>*,
                           manipulation::PiecewiseCartesianTrajectory<T>>&
  get_body_trajectories() const {
    return body_trajectories_;
  }

  /**
   * Returns the Cartesian trajectory for @p body.
   */
  const manipulation::PiecewiseCartesianTrajectory<T>& get_body_trajectory(
      const RigidBody<T>* body) const {
    return body_trajectories_.at(body);
  }

  /**
   * Returns trajectory for all degrees of freedom.
   */
  const manipulation::PiecewiseCubicTrajectory<T>& get_dof_trajectory() const {
    return dof_trajectory_;
  }

  /**
   * Returns true if there is a Cartesian trajectory for @p body.
   */
  bool has_body_trajectory(const RigidBody<T>* body) const {
    return body_trajectories_.find(body) != body_trajectories_.end();
  }

  /**
   * Returns true if @p robot is compatible. The default implementation always
   * returns true. Derived class must override to implement meaningful checks.
   */
  virtual bool IsRigidBodyTreeCompatible(const RigidBodyTree<T>& robot) const {
    unused(robot);
    return true;
  }

  /**
   * Returns true if @p alias_groups is compatible. The default implementation
   * always returns true. Derived class must override to implement meaningful
   * checks.
   */
  virtual bool IsRigidBodyTreeAliasGroupsCompatible(
      const RigidBodyTreeAliasGroups<T>& alias_groups) const {
    unused(alias_groups);
    return true;
  }

  /**
   * Returns true if @p paramset is compatible. The default implementation
   * always returns true. Derived class must override to implement meaningful
   * checks.
   */
  virtual bool IsParamSetCompatible(
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset)
      const {
    unused(paramset);
    return true;
  }

 protected:
  /**
   * It is convenient to separate allocation with initialization because
   * the later commonly depends on extra information such as measured robot
   * state.
   */
  GenericPlan() {}

  /**
   * Custom fields can be cloned here.
   */
  virtual GenericPlan<T>* CloneGenericPlanDerived() const = 0;

  /**
   * Custom initialization can be implemented here.
   */
  virtual void InitializeGenericPlanDerived(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
          robot_status,
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups) = 0;

  /**
   * Custom state mutation can be implemented here.
   */
  virtual void ModifyPlanGenericPlanDerived(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
          robot_status,
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups) = 0;

  /**
   * Custom plan handling can be implemented here.
   */
  virtual void HandlePlanGenericPlanDerived(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
          robot_status,
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups,
      const AbstractValue& plan) = 0;

  /**
   * Custom QpInput updates can be implemented here.
   */
  virtual void UpdateQpInputGenericPlanDerived(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
          robot_status,
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups,
      systems::controllers::qp_inverse_dynamics::QpInput* qp_input) const = 0;

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
  void set_body_trajectory(
      const RigidBody<T>* body,
      const manipulation::PiecewiseCartesianTrajectory<T>& traj) {
    auto it = body_trajectories_.find(body);
    if (it != body_trajectories_.end()) {
      body_trajectories_.erase(it);
    }
    body_trajectories_.emplace(body, traj);
  }

  /**
   * Removes a Cartesian trajectory for @p body.
   */
  void remove_body_trajectory(const RigidBody<T>* body) {
    body_trajectories_.erase(body);
  }

  /**
   * Sets dof trajectory to @p traj.
   */
  void set_dof_trajectory(
      const manipulation::PiecewiseCubicTrajectory<T>& traj) {
    dof_trajectory_ = traj;
  }

  /**
   * Checks @p robot, @p paramset and @p alias_groups's compatibility, throws
   * std::logic_error if any is incompatible.
   */
  void CheckCompatibilityAndThrow(
      const RigidBodyTree<T>& robot,
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups) const {
    if (!IsRigidBodyTreeCompatible(robot)) {
      throw std::logic_error("Robot model is incompatible with the plan.");
    }
    if (!IsRigidBodyTreeAliasGroupsCompatible(alias_groups)) {
      throw std::logic_error("Alias_groups is incompatible with the plan.");
    }
    if (!IsParamSetCompatible(paramset)) {
      throw std::logic_error("ParamSet is incompatible with the plan.");
    }
  }

 private:
  // Planned set of bodies that are in contact.
  ContactState contact_state_;
  // Trajectory for all dof.
  manipulation::PiecewiseCubicTrajectory<T> dof_trajectory_;
  // Trajectories for all bodies that have Cartesian tracking objectives.
  std::unordered_map<const RigidBody<T>*,
                     manipulation::PiecewiseCartesianTrajectory<T>>
      body_trajectories_;
};

}  // namespace plan_eval
}  // namespace controllers
}  // namespace systems
}  // namespace drake
