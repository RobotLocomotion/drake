#include "drake/manipulation/franka_panda/panda_command_sender.h"

#include <vector>

#include <fmt/format.h>

#include "drake/lcmt_panda_status.hpp"
#include "drake/manipulation/franka_panda/panda_constants.h"

namespace drake {
namespace manipulation {
namespace franka_panda {

using drake::lcmt_panda_command;
using drake::lcmt_panda_status;
using drake::systems::Context;
using drake::systems::kVectorValued;

PandaCommandSender::PandaCommandSender(int num_joints,
                                       PandaControlMode control_mode)
    : num_joints_(num_joints), control_mode_(control_mode) {
  PandaControlMode remaining = control_mode_;
  if ((control_mode_ & PandaControlModes::kPosition) !=
      PandaControlModes::kNone) {
    remaining &= ~PandaControlModes::kPosition;
    position_input_port_ =
        &this->DeclareInputPort("position", kVectorValued, num_joints_);
  }
  if ((control_mode_ & PandaControlModes::kVelocity) !=
      PandaControlModes::kNone) {
    remaining &= ~PandaControlModes::kVelocity;
    velocity_input_port_ =
        &this->DeclareInputPort("velocity", kVectorValued, num_joints_);
  }
  if ((control_mode_ & PandaControlModes::kTorque) !=
      PandaControlModes::kNone) {
    remaining &= ~PandaControlModes::kTorque;
    torque_input_port_ =
        &this->DeclareInputPort("torque", kVectorValued, num_joints_);
  }
  if (remaining != PandaControlModes::kNone) {
    throw std::logic_error(fmt::format("Invalid control_mode bits set: 0x{:x}",
                                       to_int(remaining)));
  }
  this->DeclareAbstractOutputPort("lcmt_panda_command",
                                  &PandaCommandSender::CalcOutput);
}

PandaCommandSender::~PandaCommandSender() = default;

using InPort = drake::systems::InputPort<double>;

const InPort& PandaCommandSender::get_position_input_port() const {
  if (position_input_port_ == nullptr) {
    throw std::logic_error(
        "Invalid call to PandaCommandSender::get_position_input_port when"
        " control_mode does not involve position");
  }
  return *position_input_port_;
}

const InPort& PandaCommandSender::get_velocity_input_port() const {
  if (velocity_input_port_ == nullptr) {
    throw std::logic_error(
        "Invalid call to PandaCommandSender::get_velocity_input_port when"
        " control_mode does not involve velocity");
  }
  return *velocity_input_port_;
}

const InPort& PandaCommandSender::get_torque_input_port() const {
  if (torque_input_port_ == nullptr) {
    throw std::logic_error(
        "Invalid call to PandaCommandSender::get_torque_input_port when"
        " control_mode does not involve torque");
  }
  return *torque_input_port_;
}

namespace {
void CopyInputPortToMessage(const Context<double>& context, const InPort* port,
                            int32_t* size_out,
                            std::vector<double>* values_out) {
  DRAKE_DEMAND(size_out != nullptr);
  DRAKE_DEMAND(values_out != nullptr);
  if (port == nullptr) {
    *size_out = 0;
    values_out->clear();
  } else {
    const auto& values_in = port->Eval(context);
    const int size_in = values_in.size();
    *size_out = size_in;
    values_out->resize(size_in);
    for (int i = 0; i < size_in; ++i) {
      (*values_out)[i] = values_in[i];
    }
  }
}
}  // namespace

void PandaCommandSender::CalcOutput(const Context<double>& context,
                                    lcmt_panda_command* output) const {
  lcmt_panda_command& command = *output;
  command.utime = context.get_time() * 1e6;
  command.control_mode_expected = to_int(control_mode_);
  CopyInputPortToMessage(context, position_input_port_,
                         &command.num_joint_position, &command.joint_position);
  CopyInputPortToMessage(context, velocity_input_port_,
                         &command.num_joint_velocity, &command.joint_velocity);
  CopyInputPortToMessage(context, torque_input_port_, &command.num_joint_torque,
                         &command.joint_torque);
}

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
