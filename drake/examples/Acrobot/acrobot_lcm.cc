#include "drake/examples/Acrobot/acrobot_lcm.h"
#include "drake/examples/Acrobot/lcmt_acrobot_u.hpp"
#include "drake/examples/Acrobot/lcmt_acrobot_x.hpp"

namespace drake {
namespace examples {
namespace acrobot {

using systems::Context;
using systems::SystemOutput;

static const int kNumJoints = 2;

/*--------------------------------------------------------------------------*/
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
  /*
  std::cout << "\ntheta1 = " << output_vec(0)
            << "\ntheta2 = " << output_vec(1)
            << "\ntheta1Dot = " << output_vec(2)
            << "\ntheta2Dot = " << output_vec(3);
  */
}

/*--------------------------------------------------------------------------*/
AcrobotCommandSender::AcrobotCommandSender() {
  this->DeclareInputPort(systems::kVectorValued, 1);
  this->DeclareAbstractOutputPort();
}

std::unique_ptr<systems::SystemOutput<double>>
AcrobotCommandSender::AllocateOutput(
    const systems::Context<double>& context) const {
  auto output = std::make_unique<systems::LeafSystemOutput<double>>();
  lcmt_acrobot_u msg{};
  msg.tau = 0;

  output->get_mutable_ports()->emplace_back(
      std::make_unique<systems::OutputPort>(
          std::make_unique<systems::Value<lcmt_acrobot_u>>(msg)));
  return std::unique_ptr<SystemOutput<double>>(output.release());
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
  /*
  std::cout << "\ntheta1 = " << output_vec(0)
            << "\ntheta2 = " << output_vec(1)
            << "\ntheta1Dot = " << output_vec(2)
            << "\ntheta2Dot = " << output_vec(3);
  */
}

/*--------------------------------------------------------------------------*/
AcrobotStateSender::AcrobotStateSender() {
  this->DeclareInputPort(systems::kVectorValued, kNumJoints * 2);
  this->DeclareAbstractOutputPort();
}

std::unique_ptr<systems::SystemOutput<double>>
AcrobotStateSender::AllocateOutput(
    const systems::Context<double>& context) const {
  auto output = std::make_unique<systems::LeafSystemOutput<double>>();
  lcmt_acrobot_x msg{};
  msg.theta1 = 0;
  msg.theta2 = 0;
  msg.theta1Dot = 0;
  msg.theta2Dot = 0;

  output->get_mutable_ports()->emplace_back(
      std::make_unique<systems::OutputPort>(
          std::make_unique<systems::Value<lcmt_acrobot_x>>(msg)));
  return std::unique_ptr<SystemOutput<double>>(output.release());
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
