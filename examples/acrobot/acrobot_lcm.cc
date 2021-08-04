#include "drake/examples/acrobot/acrobot_lcm.h"

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
  this->DeclareAbstractInputPort("lcmt_acrobot_x",
                                 Value<lcmt_acrobot_x>{});
  this->DeclareVectorOutputPort("acrobot_state",
                                &AcrobotStateReceiver::CopyStateOut);
}

void AcrobotStateReceiver::CopyStateOut(
    const Context<double>& context, AcrobotState<double>* output) const {
  const AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& state = input->get_value<lcmt_acrobot_x>();
  auto output_vec = output->get_mutable_value();

  output_vec(0) = state.theta1;
  output_vec(1) = state.theta2;
  output_vec(2) = state.theta1Dot;
  output_vec(3) = state.theta2Dot;
}

/*--------------------------------------------------------------------------*/
// methods implementation for AcrobotCommandSender.

AcrobotCommandSender::AcrobotCommandSender() {
  this->DeclareInputPort("elbow_torque", systems::kVectorValued, 1);
  this->DeclareAbstractOutputPort("lcmt_acrobot_u",
                                  &AcrobotCommandSender::OutputCommand);
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
  this->DeclareAbstractInputPort("lcmt_acrobot_u",
                                 Value<lcmt_acrobot_u>());
  this->DeclareVectorOutputPort("elbow_torque", 1,
                                &AcrobotCommandReceiver::OutputCommandAsVector);
}

void AcrobotCommandReceiver::OutputCommandAsVector(
    const Context<double>& context,
    systems::BasicVector<double>* output) const {
  const AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->get_value<lcmt_acrobot_u>();
  output->SetAtIndex(0, command.tau);
}

/*--------------------------------------------------------------------------*/
// methods implementation for AcrobotStateSender

AcrobotStateSender::AcrobotStateSender() {
  this->DeclareInputPort("acrobot_state", systems::kVectorValued,
                         kNumJoints * 2);
  this->DeclareAbstractOutputPort("lcmt_acrobot_x",
                                  &AcrobotStateSender::OutputState);
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
