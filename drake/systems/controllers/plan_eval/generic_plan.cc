#include "drake/systems/controllers/plan_eval/generic_plan.h"

#include <vector>

#include "drake/systems/controllers/setpoint.h"

namespace drake {
namespace systems {
namespace controllers {
namespace plan_eval {

template <typename T>
std::unique_ptr<GenericPlan<T>> GenericPlan<T>::Clone() const {
  return std::unique_ptr<GenericPlan<T>>(CloneGenericPlanDerived());
}

template <typename T>
void GenericPlan<T>::Initialize(
    const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
        robot_status,
    const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
    const RigidBodyTreeAliasGroups<T>& alias_groups) {
  // Checks parameters and throw if they are incompatible.
  CheckCompatibilityAndThrow(robot_status.get_robot(), paramset, alias_groups);

  // Sets contact states sequence to empty from t = 0 to forever.
  ContactState empty;
  UpdateContactState(empty);

  // Clears all tracked body trajectories.
  body_trajectories_.clear();

  // Initializes all dof trajectory to hold at the measured.
  // When interpolating, time is clipped to the bounds, so the end time doesn't
  // matter for a zoh trajectory.
  const std::vector<T> times = {robot_status.get_time(),
                                robot_status.get_time() + 1};
  const MatrixX<T> q_d = robot_status.get_cache().getQ();
  this->set_dof_trajectory(manipulation::PiecewiseCubicTrajectory<T>(
      PiecewisePolynomial<T>::ZeroOrderHold(times, {q_d, q_d})));

  // Calls custom initialization.
  InitializeGenericPlanDerived(robot_status, paramset, alias_groups);
}

template <typename T>
void GenericPlan<T>::UpdateQpInput(
    const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
        robot_status,
    const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
    const RigidBodyTreeAliasGroups<T>& alias_groups,
    systems::controllers::qp_inverse_dynamics::QpInput* qp_input) const {
  // Checks parameters and throw if they are incompatible.
  CheckCompatibilityAndThrow(robot_status.get_robot(), paramset, alias_groups);

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
  const T interp_time = robot_status.get_time();
  for (const auto& body_motion_pair : body_trajectories_) {
    const RigidBody<T>* body = body_motion_pair.first;
    const manipulation::PiecewiseCartesianTrajectory<T>& traj =
        body_motion_pair.second;

    Vector6<T> kp, kd;
    paramset.LookupDesiredBodyMotionGains(*body, &kp, &kd);

    systems::controllers::CartesianSetpoint<T> tracker(
        traj.get_pose(interp_time), traj.get_velocity(interp_time),
        traj.get_acceleration(interp_time), kp, kd);

    Isometry3<T> pose = robot_status.get_robot().CalcBodyPoseInWorldFrame(
        robot_status.get_cache(), *body);
    Vector6<T> velocity =
        robot_status.get_robot().CalcBodySpatialVelocityInWorldFrame(
            robot_status.get_cache(), *body);
    qp_input->mutable_desired_body_motions()
        .at(body->get_name())
        .mutable_values() = tracker.ComputeTargetAcceleration(pose, velocity);
  }

  // Generates desired acceleration for all dof.
  VectorX<T> kp, kd;
  paramset.LookupDesiredDofMotionGains(&kp, &kd);
  const manipulation::PiecewiseCubicTrajectory<T>& dof_traj = dof_trajectory_;
  systems::controllers::VectorSetpoint<T> tracker(
      dof_traj.get_position(interp_time), dof_traj.get_velocity(interp_time),
      dof_traj.get_acceleration(interp_time), kp, kd);

  qp_input->mutable_desired_dof_motions().mutable_values() =
      tracker.ComputeTargetAcceleration(robot_status.get_cache().getQ(),
                                        robot_status.get_cache().getV());

  // Calls custom update.
  UpdateQpInputGenericPlanDerived(robot_status, paramset, alias_groups,
                                  qp_input);
}

template class GenericPlan<double>;

}  // namespace plan_eval
}  // namespace controllers
}  // namespace systems
}  // namespace drake
