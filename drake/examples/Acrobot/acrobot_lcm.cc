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
  this->DeclareVectorOutputPort(&AcrobotStateReceiver::CopyStateOut);
}

void AcrobotStateReceiver::CopyStateOut(
    const Context<double>& context, AcrobotStateVector<double>* output) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& state = input->GetValue<lcmt_acrobot_x>();
  auto output_vec = output->get_mutable_value();

  output_vec(0) = state.theta1;
  output_vec(1) = state.theta2;
  output_vec(2) = state.theta1Dot;
  output_vec(3) = state.theta2Dot;
}

/*--------------------------------------------------------------------------*/
// methods implementation for AcrobotCommandSender.

AcrobotCommandSender::AcrobotCommandSender() {
  this->DeclareInputPort(systems::kVectorValued, 1);
  this->DeclareAbstractOutputPort(&AcrobotCommandSender::OutputCommand);
}

void AcrobotCommandSender::OutputCommand(const Context<double>& context,
                                         lcmt_acrobot_u* status) const {
  const systems::BasicVector<double>* command =
      this->EvalVectorInput(context, 0);
  status->tau = command->GetAtIndex(0);
}

/*--------------------------------------------------------------------------*/
// methods implementation for AcrobotCommandReceiver

AcrobotCommandReceiver::AcrobotCommandReceiver() {
  this->DeclareAbstractInputPort();
  this->DeclareVectorOutputPort(systems::BasicVector<double>(1),
                                &AcrobotCommandReceiver::OutputCommandAsVector);
}

void AcrobotCommandReceiver::OutputCommandAsVector(
    const Context<double>& context,
    systems::BasicVector<double>* output) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->GetValue<lcmt_acrobot_u>();
  output->SetAtIndex(0, command.tau);
}

/*--------------------------------------------------------------------------*/
// methods implementation for AcrobotStateSender

AcrobotStateSender::AcrobotStateSender() {
  this->DeclareInputPort(systems::kVectorValued, kNumJoints * 2);
  this->DeclareAbstractOutputPort(&AcrobotStateSender::OutputState);
}

void AcrobotStateSender::OutputState(const Context<double>& context,
                                     lcmt_acrobot_x* status) const {
  const systems::BasicVector<double>* state = this->EvalVectorInput(context, 0);
  status->theta1 = state->GetAtIndex(0);
  status->theta2 = state->GetAtIndex(1);
  status->theta1Dot = state->GetAtIndex(2);
  status->theta2Dot = state->GetAtIndex(3);
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
