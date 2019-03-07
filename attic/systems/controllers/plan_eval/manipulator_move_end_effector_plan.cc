#include "drake/systems/controllers/plan_eval/manipulator_move_end_effector_plan.h"

#include <vector>

#include "drake/common/unused.h"
#include "drake/lcmt_manipulator_plan_move_end_effector.hpp"
#include "drake/manipulation/util/bot_core_lcm_encode_decode.h"

namespace drake {
namespace systems {
namespace controllers {
namespace plan_eval {

using systems::controllers::qp_inverse_dynamics::ParamSet;
using systems::controllers::qp_inverse_dynamics::RobotKinematicState;

template <typename T>
void ManipulatorMoveEndEffectorPlan<T>::InitializeGenericPlanDerived(
    const RobotKinematicState<T>& robot_status,
    const ParamSet& paramset,
    const RigidBodyTreeAliasGroups<T>& alias_groups) {
  unused(paramset);  // TODO(jwnimmer-tri) This seems bad.

  // Knots are constant, the second time doesn't matter.
  const std::vector<T> times = {robot_status.get_time(),
                                robot_status.get_time() + 1};
  const RigidBody<T>* ee_body =
      alias_groups.get_body(kEndEffectorAliasGroupName);
  Isometry3<T> ee_pose = robot_status.get_robot().CalcBodyPoseInWorldFrame(
      robot_status.get_cache(), *ee_body);

  manipulation::PiecewiseCartesianTrajectory<T> ee_traj =
      manipulation::PiecewiseCartesianTrajectory<
          T>::MakeCubicLinearWithEndLinearVelocity(times, {ee_pose, ee_pose},
                                                   Vector3<double>::Zero(),
                                                   Vector3<double>::Zero());
  this->set_body_trajectory(ee_body, ee_traj);
}

template <typename T>
void ManipulatorMoveEndEffectorPlan<T>::HandlePlanGenericPlanDerived(
    const RobotKinematicState<T>& robot_status,
    const ParamSet& paramset,
    const RigidBodyTreeAliasGroups<T>& alias_groups,
    const AbstractValue& plan) {
  unused(paramset);  // TODO(jwnimmer-tri) This seems bad.

  const auto& msg =
      plan.get_value<lcmt_manipulator_plan_move_end_effector>();

  // TODO(siyuan): should do better error handling wrt bad plan plan.
  DRAKE_DEMAND(static_cast<size_t>(msg.num_steps) == msg.utimes.size() &&
               static_cast<size_t>(msg.num_steps) == msg.poses.size());
  DRAKE_DEMAND(msg.num_steps >= 0);

  if (msg.num_steps == 0) return;

  std::vector<T> times;
  std::vector<Isometry3<T>> poses;
  Vector3<double> vel0 = Vector3<double>::Zero();

  const RigidBody<T>* ee_body =
      alias_groups.get_body(kEndEffectorAliasGroupName);

  // If the first keyframe does not start immediately (its time > 0), we start
  // from the current desired pose.
  if (msg.utimes.front() > 0) {
    times.push_back(robot_status.get_time());
    poses.push_back(
        this->get_body_trajectory(ee_body).get_pose(robot_status.get_time()));
    vel0 = this->get_body_trajectory(ee_body)
               .get_velocity(robot_status.get_time())
               .template tail<3>();
  }

  for (int i = 0; i < msg.num_steps; i++) {
    times.push_back(robot_status.get_time() +
                    static_cast<T>(msg.utimes[i]) / 1e6);
    poses.push_back(DecodePose(msg.poses[i]));
  }

  // TODO(siyuan): use msg.order_of_interpolation.
  manipulation::PiecewiseCartesianTrajectory<T> ee_traj =
      manipulation::PiecewiseCartesianTrajectory<
          T>::MakeCubicLinearWithEndLinearVelocity(times, poses, vel0,
                                                   Vector3<double>::Zero());

  this->set_body_trajectory(ee_body, ee_traj);
}

template <typename T>
GenericPlan<T>* ManipulatorMoveEndEffectorPlan<T>::CloneGenericPlanDerived()
    const {
  return new ManipulatorMoveEndEffectorPlan(*this);
}

template class ManipulatorMoveEndEffectorPlan<double>;

}  // namespace plan_eval
}  // namespace controllers
}  // namespace systems
}  // namespace drake
