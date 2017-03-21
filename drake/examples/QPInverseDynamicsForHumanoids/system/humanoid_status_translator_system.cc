#include "drake/examples/QPInverseDynamicsForHumanoids/system/humanoid_status_translator_system.h"

#include "bot_core/robot_state_t.hpp"

#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

HumanoidStatusTranslatorSystem::HumanoidStatusTranslatorSystem(
    const RigidBodyTree<double>& robot, const std::string& alias_group_path)
    : robot_(robot), alias_group_path_(alias_group_path) {
  output_port_index_humanoid_status_ = DeclareAbstractOutputPort().get_index();
}

std::unique_ptr<systems::AbstractValue>
HumanoidStatusTranslatorSystem::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() == output_port_index_humanoid_status_);

  param_parsers::RigidBodyTreeAliasGroups<double> alias_groups(robot_);
  alias_groups.LoadFromFile(alias_group_path_);

  return systems::AbstractValue::Make<HumanoidStatus>(
      HumanoidStatus(robot_, alias_groups));
}

StateToHumanoidStatusSystem::StateToHumanoidStatusSystem(
    const RigidBodyTree<double>& robot, const std::string& path)
    : HumanoidStatusTranslatorSystem(robot, path) {
  const int kDim = robot.get_num_positions() + robot.get_num_velocities();
  input_port_index_state_ =
      DeclareInputPort(systems::kVectorValued, kDim).get_index();
}

void StateToHumanoidStatusSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  const VectorX<double> x =
      EvalEigenVectorInput(context, input_port_index_state_);

  HumanoidStatus& humanoid_status =
      output->GetMutableData(get_output_port_index_humanoid_status())
          ->GetMutableValue<HumanoidStatus>();

  const int kPosDim = get_robot().get_num_positions();
  const int kVelDim = get_robot().get_num_velocities();
  humanoid_status.UpdateKinematics(context.get_time(), x.head(kPosDim),
                                   x.tail(kVelDim));
}

RobotStateMsgToHumanoidStatusSystem::RobotStateMsgToHumanoidStatusSystem(
    const RigidBodyTree<double>& robot, const std::string& alias_group_path)
    : HumanoidStatusTranslatorSystem(robot, alias_group_path) {
  input_port_index_lcm_msg_ = DeclareAbstractInputPort().get_index();
}

void RobotStateMsgToHumanoidStatusSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  const bot_core::robot_state_t* msg = EvalInputValue<bot_core::robot_state_t>(
      context, input_port_index_lcm_msg_);

  HumanoidStatus& humanoid_status =
      output->GetMutableData(get_output_port_index_humanoid_status())
          ->GetMutableValue<HumanoidStatus>();

  VectorX<double> pos(humanoid_status.position().size());
  VectorX<double> vel(humanoid_status.velocity().size());
  VectorX<double> joint_torque(humanoid_status.joint_torque().size());
  Vector6<double> l_foot_wrench, r_foot_wrench;
  double time;

  DecodeRobotStateLcmMsg(*msg, humanoid_status.name_to_position_index(), &time,
                         &pos, &vel, &joint_torque, &l_foot_wrench,
                         &r_foot_wrench);

  humanoid_status.Update(time, pos, vel, joint_torque, l_foot_wrench,
                         r_foot_wrench);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
