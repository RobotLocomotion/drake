#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"

#include "drake/common/drake_assert.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using systems::BasicVector;
using systems::Context;
using systems::DiscreteValues;
using systems::State;
using systems::SystemOutput;

// This value is chosen to match the value in getSendPeriodMilliSec()
// when initializing the FRI configuration on the iiwa's control
// cabinet.
const double kIiwaLcmStatusPeriod = 0.005;

IiwaCommandReceiver::IiwaCommandReceiver(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareAbstractInputPort();
  this->DeclareVectorOutputPort(systems::BasicVector<double>(num_joints_ * 2),
                                &IiwaCommandReceiver::OutputCommand);
  this->DeclareDiscreteUpdatePeriodSec(kIiwaLcmStatusPeriod);
  this->DeclareDiscreteState(num_joints_ * 2);
}

void IiwaCommandReceiver::set_initial_position(
    Context<double>* context,
    const Eigen::Ref<const VectorX<double>> x) const {
  auto state_value =
      context->get_mutable_discrete_state(0)->get_mutable_value();
  DRAKE_ASSERT(x.size() == num_joints_);
  state_value.head(num_joints_) = x;
  state_value.tail(num_joints_) = VectorX<double>::Zero(num_joints_);
}

void IiwaCommandReceiver::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->GetValue<lcmt_iiwa_command>();
  // TODO(sam.creasey) Support torque control.
  DRAKE_ASSERT(command.num_torques == 0);


  // If we're using a default constructed message (haven't received
  // a command yet), keep using the initial state.
  if (command.num_joints != 0) {
    DRAKE_DEMAND(command.num_joints == num_joints_);
    VectorX<double> new_positions(num_joints_);
    for (int i = 0; i < command.num_joints; ++i) {
      new_positions(i) = command.joint_position[i];
    }

    BasicVector<double>* state = discrete_state->get_mutable_vector(0);
    auto state_value = state->get_mutable_value();
    state_value.tail(num_joints_) =
        (new_positions - state_value.head(num_joints_)) / kIiwaLcmStatusPeriod;
    state_value.head(num_joints_) = new_positions;
  }
}

void IiwaCommandReceiver::OutputCommand(const Context<double>& context,
                                        BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec =
      output->get_mutable_value();
  output_vec = context.get_discrete_state(0)->get_value();
}

IiwaCommandSender::IiwaCommandSender(int num_joints)
    : num_joints_(num_joints),
      position_input_port_(
          this->DeclareInputPort(
              systems::kVectorValued, num_joints_).get_index()),
      torque_input_port_(
          this->DeclareInputPort(
              systems::kVectorValued, num_joints_).get_index()) {
  this->DeclareAbstractOutputPort(&IiwaCommandSender::OutputCommand);
}

void IiwaCommandSender::OutputCommand(
    const Context<double>& context, lcmt_iiwa_command* output) const {
  lcmt_iiwa_command& command = *output;

  command.utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* positions =
      this->EvalVectorInput(context, 0);

  command.num_joints = num_joints_;
  command.joint_position.resize(num_joints_);
  for (int i = 0; i < num_joints_; ++i) {
    command.joint_position[i] = positions->GetAtIndex(i);
  }

  const systems::BasicVector<double>* torques =
      this->EvalVectorInput(context, 1);
  if (torques == nullptr) {
    command.num_torques = 0;
    command.joint_torque.clear();
  } else {
    command.num_torques = num_joints_;
    command.joint_torque.resize(num_joints_);
    for (int i = 0; i < num_joints_; ++i) {
      command.joint_torque[i] = torques->GetAtIndex(i);
    }
  }
}

IiwaStatusReceiver::IiwaStatusReceiver(int num_joints)
    : num_joints_(num_joints),
      measured_position_output_port_(
          this->DeclareVectorOutputPort(
                  systems::BasicVector<double>(num_joints_ * 2),
                  &IiwaStatusReceiver::OutputMeasuredPosition)
              .get_index()),
      commanded_position_output_port_(
          this->DeclareVectorOutputPort(
                  systems::BasicVector<double>(num_joints_),
                  &IiwaStatusReceiver::OutputCommandedPosition)
              .get_index()) {
  this->DeclareAbstractInputPort();
  this->DeclareDiscreteState(num_joints_ * 3);
  this->DeclareDiscreteUpdatePeriodSec(kIiwaLcmStatusPeriod);
}

void IiwaStatusReceiver::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& status = input->GetValue<lcmt_iiwa_status>();

  // If we're using a default constructed message (haven't received
  // status yet), keep using the initial state.
  if (status.num_joints != 0) {
    DRAKE_DEMAND(status.num_joints == num_joints_);

    VectorX<double> measured_position(num_joints_);
    VectorX<double> commanded_position(num_joints_);
    for (int i = 0; i < status.num_joints; ++i) {
      measured_position(i) = status.joint_position_measured[i];
      commanded_position(i) = status.joint_position_commanded[i];
    }

    BasicVector<double>* state = discrete_state->get_mutable_vector(0);
    auto state_value = state->get_mutable_value();
    state_value.segment(num_joints_, num_joints_) =
        (measured_position - state_value.head(num_joints_)) /
        kIiwaLcmStatusPeriod;
    state_value.head(num_joints_) = measured_position;
    state_value.tail(num_joints_) = commanded_position;
  }
}

void IiwaStatusReceiver::OutputMeasuredPosition(const Context<double>& context,
                                       BasicVector<double>* output) const {
  const auto state_value = context.get_discrete_state(0)->get_value();

  Eigen::VectorBlock<VectorX<double>> measured_position_output =
      output->get_mutable_value();
  measured_position_output = state_value.head(num_joints_ * 2);
}

void IiwaStatusReceiver::OutputCommandedPosition(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto state_value = context.get_discrete_state(0)->get_value();

  Eigen::VectorBlock<VectorX<double>> commanded_position_output =
      output->get_mutable_value();
  commanded_position_output = state_value.tail(num_joints_);
}

IiwaStatusSender::IiwaStatusSender(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareInputPort(systems::kVectorValued, num_joints_ * 2);
  this->DeclareInputPort(systems::kVectorValued, num_joints_ * 2);
  this->DeclareAbstractOutputPort(&IiwaStatusSender::MakeOutputStatus,
                                  &IiwaStatusSender::OutputStatus);
}

lcmt_iiwa_status IiwaStatusSender::MakeOutputStatus() const {
  lcmt_iiwa_status msg{};
  msg.num_joints = num_joints_;
  msg.joint_position_measured.resize(msg.num_joints, 0);
  msg.joint_position_commanded.resize(msg.num_joints, 0);
  msg.joint_position_ipo.resize(msg.num_joints, 0);
  msg.joint_torque_measured.resize(msg.num_joints, 0);
  msg.joint_torque_commanded.resize(msg.num_joints, 0);
  msg.joint_torque_external.resize(msg.num_joints, 0);
  return msg;
}

void IiwaStatusSender::OutputStatus(
    const Context<double>& context, lcmt_iiwa_status* output) const {
  lcmt_iiwa_status& status = *output;

  status.utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* command =
      this->EvalVectorInput(context, 0);
  const systems::BasicVector<double>* state =
      this->EvalVectorInput(context, 1);
  for (int i = 0; i < num_joints_; ++i) {
    status.joint_position_measured[i] = state->GetAtIndex(i);
    status.joint_position_commanded[i] = command->GetAtIndex(i);
  }
}


}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
