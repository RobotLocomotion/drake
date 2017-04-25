#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/generic_plan.h"

#include <vector>

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
std::unique_ptr<GenericPlan<T>> GenericPlan<T>::Clone() const {
  return std::unique_ptr<GenericPlan<T>>(CloneGenericPlanDerived());
}

template <typename T>
void GenericPlan<T>::Initialize(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  // Sets contact states sequence to empty from t = 0 to forever.
  ContactState empty;
  UpdateContactState(empty);

  // Clears all tracked body trajectories.
  body_trajectories_.clear();

  // Initializes all dof trajectory to hold at the measured.
  // When interpolating, time is clipped to the bounds, so the end time doesn't
  // matter for a zoh trajectory.
  const std::vector<T> times = {robot_status.time(), robot_status.time() + 1};
  const MatrixX<T> q_d = robot_status.position();
  this->set_dof_trajectory(PiecewiseCubicTrajectory<T>(
      PiecewisePolynomial<T>::ZeroOrderHold(times, {q_d, q_d})));

  // Calls custom initialization.
  InitializeGenericPlanDerived(robot_status, paramset, alias_groups);
}

template <typename T>
void GenericPlan<T>::HandlePlanMessage(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const void* message_bytes, int message_length) {
  // Calls custom handler.
  HandlePlanMessageGenericPlanDerived(robot_status, paramset, alias_groups,
                                      message_bytes, message_length);
}

template <typename T>
void GenericPlan<T>::ModifyPlan(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  // Runs derived class' plan.
  ModifyPlanGenericPlanDerived(robot_status, paramset, alias_groups);
}

template <typename T>
void GenericPlan<T>::UpdateQpInput(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    QpInput* qp_input) const {
  // Gets all bodies that are in contact.
  const ContactState& contact_state = get_planned_contact_state();
  const std::vector<const RigidBody<T>*> contact_bodies(contact_state.begin(),
                                                        contact_state.end());

  // Gets all bodies that have a Cartesian tracking objective.
  std::vector<const RigidBody<T>*> tracked_bodies;
  for (const auto& tracked_body_pair : body_trajectories_) {
    tracked_bodies.push_back(tracked_body_pair.first);
  }

  // Sets up weights and modes.
  *qp_input =
      paramset.MakeQpInput(contact_bodies, tracked_bodies, alias_groups);

  // Generates desired acceleration for tracked bodies.
  const T interp_time = robot_status.time();
  for (const auto& body_motion_pair : body_trajectories_) {
    const RigidBody<T>* body = body_motion_pair.first;
    const PiecewiseCartesianTrajectory<T>& traj = body_motion_pair.second;

    Vector6<T> kp, kd;
    paramset.LookupDesiredBodyMotionGains(*body, &kp, &kd);

    CartesianSetpoint<T> tracker(traj.get_pose(interp_time),
                                 traj.get_velocity(interp_time),
                                 traj.get_acceleration(interp_time), kp, kd);

    Isometry3<T> pose = robot_status.robot().CalcBodyPoseInWorldFrame(
        robot_status.cache(), *body);
    Vector6<T> velocity =
        robot_status.robot().CalcBodySpatialVelocityInWorldFrame(
            robot_status.cache(), *body);
    qp_input->mutable_desired_body_motions()
        .at(body->get_name())
        .mutable_values() = tracker.ComputeTargetAcceleration(pose, velocity);
  }

  // Generates desired acceleration for all dof.
  VectorX<T> kp, kd;
  paramset.LookupDesiredDofMotionGains(&kp, &kd);
  const PiecewiseCubicTrajectory<T>& dof_traj = dof_trajectory_;
  VectorSetpoint<T> tracker(dof_traj.get_position(interp_time),
                            dof_traj.get_velocity(interp_time),
                            dof_traj.get_acceleration(interp_time), kp, kd);

  qp_input->mutable_desired_dof_motions().mutable_values() =
      tracker.ComputeTargetAcceleration(robot_status.position(),
                                        robot_status.velocity());

  // Calls custom update.
  UpdateQpInputGenericPlanDerived(robot_status, paramset, alias_groups,
                                  qp_input);
}

template class GenericPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
