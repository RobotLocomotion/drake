#include "drake/examples/QPInverseDynamicsForHumanoids/system/humanoid_status_translator_system.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

using systems::controllers::qp_inverse_dynamics::RobotKinematicState;

HumanoidStatusTranslatorSystem::HumanoidStatusTranslatorSystem(
    const RigidBodyTree<double>* robot, const std::string& alias_group_path)
    : robot_(*robot) {
  RigidBodyTreeAliasGroups<double> alias_groups(&robot_);
  alias_groups.LoadFromFile(alias_group_path);
  default_output_.reset(new HumanoidStatus(&robot_, alias_groups));

  // Defer creation of the output port to derived classes where the appropriate
  // calculator method is known.
}

StateToHumanoidStatusSystem::StateToHumanoidStatusSystem(
    const RigidBodyTree<double>* robot, const std::string& path)
    : HumanoidStatusTranslatorSystem(robot, path) {
  const int kDim = robot->get_num_positions() + robot->get_num_velocities();
  input_port_index_state_ =
      DeclareInputPort(systems::kVectorValued, kDim).get_index();

  set_output_port_index_humanoid_status(
      DeclareAbstractOutputPort<StateToHumanoidStatusSystem,
                                RobotKinematicState<double>>(
          get_default_output(),
          &StateToHumanoidStatusSystem::CalcHumanoidStatus)
          .get_index());
}

void StateToHumanoidStatusSystem::CalcHumanoidStatus(
    const systems::Context<double>& context,
    RobotKinematicState<double>* output) const {
  const VectorX<double> x =
      EvalEigenVectorInput(context, input_port_index_state_);

  RobotKinematicState<double>& humanoid_status = *output;

  const int kPosDim = get_robot().get_num_positions();
  const int kVelDim = get_robot().get_num_velocities();
  humanoid_status.UpdateKinematics(context.get_time(), x.head(kPosDim),
                                   x.tail(kVelDim));
}

RobotStateMsgToHumanoidStatusSystem::RobotStateMsgToHumanoidStatusSystem(
    const RigidBodyTree<double>* robot, const std::string& alias_group_path)
    : HumanoidStatusTranslatorSystem(robot, alias_group_path),
      translator_(*robot) {
  input_port_index_lcm_msg_ = DeclareAbstractInputPort().get_index();
  set_output_port_index_humanoid_status(
      DeclareAbstractOutputPort<RobotStateMsgToHumanoidStatusSystem,
                                RobotKinematicState<double>>(
          get_default_output(),
          &RobotStateMsgToHumanoidStatusSystem::CalcHumanoidStatus)
          .get_index());
}

void RobotStateMsgToHumanoidStatusSystem::CalcHumanoidStatus(
    const systems::Context<double>& context,
    RobotKinematicState<double>* output) const {
  const bot_core::robot_state_t* msg = EvalInputValue<bot_core::robot_state_t>(
      context, input_port_index_lcm_msg_);

  HumanoidStatus* humanoid_status = dynamic_cast<HumanoidStatus*>(output);
  DRAKE_DEMAND(humanoid_status != nullptr);

  const RigidBodyTree<double>& robot = humanoid_status->get_robot();
  VectorX<double> pos(robot.get_num_positions());
  VectorX<double> vel(robot.get_num_velocities());
  VectorX<double> joint_torque(robot.get_num_actuators());
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

  humanoid_status->Update(time, pos, vel, joint_torque, l_foot_wrench,
                          r_foot_wrench);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
