#include "drake/manipulation/kinova_jaco/jaco_command_sender.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {

JacoCommandSender::JacoCommandSender(int num_joints, int num_fingers)
    : num_joints_(num_joints),
      num_fingers_(num_fingers) {
  this->DeclareInputPort(
      "state", systems::kVectorValued, (num_joints_ + num_fingers_) * 2);
  this->DeclareAbstractOutputPort(
      "lcmt_jaco_command", &JacoCommandSender::CalcOutput);
}

const systems::InputPort<double>& JacoCommandSender::get_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}
const systems::OutputPort<double>& JacoCommandSender::get_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}

void JacoCommandSender::CalcOutput(
    const systems::Context<double>& context, lcmt_jaco_command* output) const {
  const auto& state = get_input_port().Eval(context);

  lcmt_jaco_command& command = *output;
  command.utime = context.get_time() * 1e6;

  command.num_joints = num_joints_;
  command.joint_position.resize(num_joints_);
  command.joint_velocity.resize(num_joints_);
  for (int i = 0; i < num_joints_; ++i) {
    command.joint_position[i] = state(i);
    command.joint_velocity[i] = state(i + num_joints_ + num_fingers_);
  }

  command.num_fingers = num_fingers_;
  command.finger_position.resize(num_fingers_);
  command.finger_velocity.resize(num_fingers_);
  for (int i = 0; i < num_fingers_; ++i) {
    command.finger_position[i] = state(i + num_joints_) * kFingerUrdfToSdk;
    command.finger_velocity[i] =
        state.tail(num_fingers_)(i) * kFingerUrdfToSdk;
  }
}

}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
