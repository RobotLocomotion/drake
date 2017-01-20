#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using systems::Context;
using systems::SystemOutput;

static const int kNumJoints = 7;

IiwaCommandReceiver::IiwaCommandReceiver() {
  this->DeclareAbstractInputPort();
  this->DeclareOutputPort(systems::kVectorValued, kNumJoints);
}

void IiwaCommandReceiver::DoCalcOutput(const Context<double>& context,
                                       SystemOutput<double>* output) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->GetValue<lcmt_iiwa_command>();
  auto output_vec = this->GetMutableOutputVector(output, 0);

  // If we're using a default constructed message (haven't received
  // a command yet), just return zeros.
  if (command.num_joints == 0) {
    output_vec.fill(0);
  } else {
    DRAKE_ASSERT(command.num_joints == kNumJoints);
    for (int i = 0; i < command.num_joints; ++i) {
      output_vec(i) = command.joint_position[i];
    }
  }

  // TODO(sam.creasey) Support torque control.
  DRAKE_ASSERT(command.num_torques == 0);
}

IiwaStatusSender::IiwaStatusSender() {
  this->DeclareInputPort(systems::kVectorValued, kNumJoints);
  this->DeclareInputPort(systems::kVectorValued, kNumJoints * 2);
  this->DeclareAbstractOutputPort();
}

std::unique_ptr<systems::SystemOutput<double>>
IiwaStatusSender::AllocateOutput(
    const systems::Context<double>& context) const {
  auto output = std::make_unique<systems::LeafSystemOutput<double>>();
  lcmt_iiwa_status msg{};
  msg.num_joints = kNumJoints;
  msg.joint_position_measured.resize(msg.num_joints, 0);
  msg.joint_position_commanded.resize(msg.num_joints, 0);
  msg.joint_position_ipo.resize(msg.num_joints, 0);
  msg.joint_torque_measured.resize(msg.num_joints, 0);
  msg.joint_torque_commanded.resize(msg.num_joints, 0);
  msg.joint_torque_external.resize(msg.num_joints, 0);

  output->get_mutable_ports()->emplace_back(
      std::make_unique<systems::OutputPort>(
          std::make_unique<systems::Value<lcmt_iiwa_status>>(msg)));
  return std::unique_ptr<SystemOutput<double>>(output.release());
}

void IiwaStatusSender::DoCalcOutput(
    const Context<double>& context, SystemOutput<double>* output) const {
  systems::AbstractValue* mutable_data = output->GetMutableData(0);
  lcmt_iiwa_status& status =
      mutable_data->GetMutableValue<lcmt_iiwa_status>();

  status.utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* command =
      this->EvalVectorInput(context, 0);
  const systems::BasicVector<double>* state =
      this->EvalVectorInput(context, 1);
  for (int i = 0; i < kNumJoints; ++i) {
    status.joint_position_measured[i] = state->GetAtIndex(i);
    status.joint_position_commanded[i] = command->GetAtIndex(i);
  }
}


}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
