#include "drake/manipulation/kuka_iiwa/iiwa_command_sender.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

IiwaCommandSender::IiwaCommandSender(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareInputPort(
      "position", systems::kVectorValued, num_joints_);
  this->DeclareInputPort(
      "torque", systems::kVectorValued, num_joints_);
  this->DeclareAbstractOutputPort(
      "lcmt_iiwa_command", &IiwaCommandSender::CalcOutput);
}

using InPort = systems::InputPort<double>;
const InPort& IiwaCommandSender::get_position_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}
const InPort& IiwaCommandSender::get_torque_input_port() const {
  return LeafSystem<double>::get_input_port(1);
}
const systems::OutputPort<double>& IiwaCommandSender::get_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}

void IiwaCommandSender::CalcOutput(
    const systems::Context<double>& context, lcmt_iiwa_command* output) const {
  const auto* const position_in = this->EvalVectorInput(context, 0);
  const auto* const torque_in = this->EvalVectorInput(context, 1);
  // Check that inputs are properly connected.
  DRAKE_THROW_UNLESS(position_in);
  const int num_torques = torque_in ? num_joints_ : 0;

  lcmt_iiwa_command& command = *output;
  command.utime = context.get_time() * 1e6;
  command.num_joints = num_joints_;
  command.joint_position.resize(num_joints_);
  for (int i = 0; i < num_joints_; ++i) {
    command.joint_position[i] = (*position_in)[i];
  }
  command.num_torques = num_torques;
  command.joint_torque.resize(num_torques);
  for (int i = 0; i < num_torques; ++i) {
    command.joint_torque[i] = (*torque_in)[i];
  }
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
