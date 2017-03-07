#include "drake/examples/Acrobot/acrobot_lcm.h"
#include "drake/lcmt_acrobot_u.hpp"
#include "drake/lcmt_acrobot_x.hpp"

namespace drake {
namespace examples {
namespace acrobot {

using systems::Context;
using systems::SystemOutput;

static const int kNumJoints = 2;

/*--------------------------------------------------------------------------*/
// methods implementation for AcrobotStateReceiver.

AcrobotStateReceiver::AcrobotStateReceiver() {
  this->DeclareAbstractInputPort();
  this->DeclareOutputPort(systems::kVectorValued, kNumJoints * 2);
}

void AcrobotStateReceiver::DoCalcOutput(const Context<double>& context,
                                        SystemOutput<double>* output) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& state = input->GetValue<lcmt_acrobot_x>();
  auto output_vec = this->GetMutableOutputVector(output, 0);

  output_vec(0) = state.theta1;
  output_vec(1) = state.theta2;
  output_vec(2) = state.theta1Dot;
  output_vec(3) = state.theta2Dot;
}

std::unique_ptr<systems::BasicVector<double>>
AcrobotStateReceiver::AllocateOutputVector(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  DRAKE_THROW_UNLESS(descriptor.size() == 4);
  return std::make_unique<AcrobotStateVector<double>>();
}

/*--------------------------------------------------------------------------*/
// methods implementation for AcrobotCommandSender.

AcrobotCommandSender::AcrobotCommandSender() {
  this->DeclareInputPort(systems::kVectorValued, 1);
  this->DeclareAbstractOutputPort();
}

std::unique_ptr<systems::AbstractValue>
AcrobotCommandSender::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  lcmt_acrobot_u msg{};
  msg.tau = 0;
  return std::make_unique<systems::Value<lcmt_acrobot_u>>(msg);
}

void AcrobotCommandSender::DoCalcOutput(const Context<double>& context,
                                        SystemOutput<double>* output) const {
  systems::AbstractValue* mutable_data = output->GetMutableData(0);
  lcmt_acrobot_u& status = mutable_data->GetMutableValue<lcmt_acrobot_u>();

  const systems::BasicVector<double>* command =
      this->EvalVectorInput(context, 0);
  status.tau = command->GetAtIndex(0);
}

/*--------------------------------------------------------------------------*/
// methods implementation for AcrobotCommandReceiver

AcrobotCommandReceiver::AcrobotCommandReceiver() {
  this->DeclareAbstractInputPort();
  this->DeclareOutputPort(systems::kVectorValued, 1);
}

void AcrobotCommandReceiver::DoCalcOutput(const Context<double>& context,
                                          SystemOutput<double>* output) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->GetValue<lcmt_acrobot_u>();
  auto output_vec = this->GetMutableOutputVector(output, 0);

  output_vec(0) = command.tau;
}

/*--------------------------------------------------------------------------*/
// methods implementation for AcrobotStateSender

AcrobotStateSender::AcrobotStateSender() {
  this->DeclareInputPort(systems::kVectorValued, kNumJoints * 2);
  this->DeclareAbstractOutputPort();
}

std::unique_ptr<systems::AbstractValue>
AcrobotStateSender::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  auto output = std::make_unique<systems::LeafSystemOutput<double>>();
  lcmt_acrobot_x msg{};
  msg.theta1 = 0;
  msg.theta2 = 0;
  msg.theta1Dot = 0;
  msg.theta2Dot = 0;
  return std::make_unique<systems::Value<lcmt_acrobot_x>>(msg);
}

void AcrobotStateSender::DoCalcOutput(const Context<double>& context,
                                      SystemOutput<double>* output) const {
  systems::AbstractValue* mutable_data = output->GetMutableData(0);
  lcmt_acrobot_x& status = mutable_data->GetMutableValue<lcmt_acrobot_x>();

  const systems::BasicVector<double>* state = this->EvalVectorInput(context, 0);
  status.theta1 = state->GetAtIndex(0);
  status.theta2 = state->GetAtIndex(1);
  status.theta1Dot = state->GetAtIndex(2);
  status.theta2Dot = state->GetAtIndex(3);
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
