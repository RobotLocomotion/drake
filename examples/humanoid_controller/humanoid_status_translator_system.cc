#include "drake/examples/humanoid_controller/humanoid_status_translator_system.h"

#include "drake/examples/humanoid_controller/humanoid_status.h"

namespace drake {
namespace examples {
namespace humanoid_controller {

using systems::controllers::qp_inverse_dynamics::RobotKinematicState;

RobotStateMsgToHumanoidStatusSystem::RobotStateMsgToHumanoidStatusSystem(
    const RigidBodyTree<double>* robot, const std::string& alias_group_path)
    : robot_(*robot), translator_(*robot) {
  RigidBodyTreeAliasGroups<double> alias_groups(&robot_);
  alias_groups.LoadFromFile(alias_group_path);
  default_output_ = std::make_unique<HumanoidStatus>(&robot_, alias_groups);

  DeclareAbstractInputPort(
      systems::kUseDefaultName,
      Value<bot_core::robot_state_t>{});
  DeclareAbstractOutputPort<RobotStateMsgToHumanoidStatusSystem,
                            RobotKinematicState<double>>(
      *default_output_,
      &RobotStateMsgToHumanoidStatusSystem::CalcHumanoidStatus);
}

void RobotStateMsgToHumanoidStatusSystem::CalcHumanoidStatus(
    const systems::Context<double>& context,
    RobotKinematicState<double>* output) const {
  const bot_core::robot_state_t* msg =
      EvalInputValue<bot_core::robot_state_t>(context, 0);

  VectorX<double> pos(robot_.get_num_positions());
  VectorX<double> vel(robot_.get_num_velocities());
  VectorX<double> joint_torque(robot_.get_num_actuators());
  Vector6<double> l_foot_wrench, r_foot_wrench;

  translator_.DecodeMessageKinematics(*msg, pos, vel);
  translator_.DecodeMessageTorque(*msg, joint_torque);
  const double time = static_cast<double>(msg->utime) / 1e6;

  // TODO(siyuan): move this to translator.
  l_foot_wrench.setZero();
  r_foot_wrench.setZero();
  l_foot_wrench[0] = msg->force_torque.l_foot_torque_x;
  l_foot_wrench[1] = msg->force_torque.l_foot_torque_y;
  l_foot_wrench[5] = msg->force_torque.l_foot_force_z;
  r_foot_wrench[0] = msg->force_torque.r_foot_torque_x;
  r_foot_wrench[1] = msg->force_torque.r_foot_torque_y;
  r_foot_wrench[5] = msg->force_torque.r_foot_force_z;

  HumanoidStatus* status = dynamic_cast<HumanoidStatus*>(output);
  DRAKE_DEMAND(status != nullptr);
  status->Update(time, pos, vel, joint_torque, l_foot_wrench, r_foot_wrench);
}

}  // namespace humanoid_controller
}  // namespace examples
}  // namespace drake
