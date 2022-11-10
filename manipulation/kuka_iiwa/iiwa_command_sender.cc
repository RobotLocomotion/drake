#include "drake/manipulation/kuka_iiwa/iiwa_command_sender.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

IiwaCommandSender::IiwaCommandSender(int num_joints, int control_mode)
    : num_joints_(num_joints), control_mode_(control_mode) {
  DRAKE_DEMAND(
      control_mode_ >= kIiwaPositionMode
      && control_mode_ <= (kIiwaPositionMode | kIiwaTorqueMode));
  if (control_mode_ & kIiwaPositionMode) {
    position_input_port_ = &this->DeclareInputPort(
        "position", systems::kVectorValued, num_joints_);
  }
  if (control_mode & kIiwaTorqueMode) {
    torque_input_port_ = &this->DeclareInputPort(
        "torque", systems::kVectorValued, num_joints_);
  }
  time_input_port_ = &this->DeclareInputPort(
      "time", systems::kVectorValued, 1);
  this->DeclareAbstractOutputPort(
      "lcmt_iiwa_command", &IiwaCommandSender::CalcOutput);
}

IiwaCommandSender::~IiwaCommandSender() = default;

using InPort = systems::InputPort<double>;
const InPort& IiwaCommandSender::get_position_input_port() const {
  DRAKE_THROW_UNLESS(control_mode_ & kIiwaPositionMode);
  DRAKE_DEMAND(position_input_port_ != nullptr);
  return *position_input_port_;
}

const InPort& IiwaCommandSender::get_torque_input_port() const {
  DRAKE_THROW_UNLESS(control_mode_ & kIiwaTorqueMode);
  DRAKE_DEMAND(torque_input_port_ != nullptr);
  return *torque_input_port_;
}

const InPort& IiwaCommandSender::get_time_input_port() const {
  DRAKE_DEMAND(time_input_port_ != nullptr);
  return *time_input_port_;
}

void IiwaCommandSender::CalcOutput(
    const systems::Context<double>& context, lcmt_iiwa_command* output) const {
  const double message_time =
      get_time_input_port().HasValue(context)
          ? get_time_input_port().Eval(context)[0]
          : context.get_time();

  const bool has_position = control_mode_ & kIiwaPositionMode;
  bool has_torque = false;
  if (control_mode_ == kIiwaTorqueMode) {
    has_torque = true;
  } else if (control_mode_ & kIiwaTorqueMode) {
    has_torque = get_torque_input_port().HasValue(context);
  }

  const int num_position = has_position ? num_joints_ : 0;
  const int num_torques = has_torque ? num_joints_ : 0;

  lcmt_iiwa_command& command = *output;
  command.utime = message_time * 1e6;

  command.num_joints = num_position;
  command.joint_position.resize(num_position);
  if (has_position) {
    const auto& position = get_position_input_port().Eval(context);
    for (int i = 0; i < num_joints_; ++i) {
      command.joint_position[i] = position[i];
    }
  }

  command.num_torques = num_torques;
  command.joint_torque.resize(num_torques);
  if (has_torque) {
    const auto& torque = get_torque_input_port().Eval(context);
    for (int i = 0; i < num_torques; ++i) {
      command.joint_torque[i] = torque[i];
    }
  }
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
