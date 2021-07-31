#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"

#include "drake/common/drake_throw.h"
#include "drake/lcm/lcm_messages.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

using Eigen::VectorXd;
using lcm::AreLcmMessagesEqual;
using systems::BasicVector;
using systems::CompositeEventCollection;
using systems::Context;
using systems::DiscreteValues;
using systems::DiscreteUpdateEvent;
using systems::NumericParameterIndex;
using systems::kVectorValued;

IiwaCommandReceiver::IiwaCommandReceiver(int num_joints)
    : num_joints_(num_joints) {
  DRAKE_THROW_UNLESS(num_joints > 0);

  message_input_ = &DeclareAbstractInputPort(
      "lcmt_iiwa_command", Value<lcmt_iiwa_command>());
  position_measured_input_ = &DeclareInputPort(
      "position_measured", kVectorValued, num_joints);

  // This cache entry provides either the input (iff connected) or else zero.
  position_measured_or_zero_ = &DeclareCacheEntry(
      "position_measured_or_zero", BasicVector<double>(num_joints),
      &IiwaCommandReceiver::CalcPositionMeasuredOrZero,
      {position_measured_input_->ticket()});

  // When a simulation begins, we will latch position_measured_or_zero into a
  // state variable, so that we will hold that pose until the first message is
  // received.  Prior to that event, we continue to use the unlatched value.
  latched_position_measured_is_set_ = DeclareDiscreteState(VectorXd::Zero(1));
  latched_position_measured_ = DeclareDiscreteState(VectorXd::Zero(num_joints));
  defaulted_command_ = &DeclareCacheEntry(
      "defaulted_command", &IiwaCommandReceiver::CalcDefaultedCommand,
      {message_input_->ticket(),
       discrete_state_ticket(latched_position_measured_is_set_),
       discrete_state_ticket(latched_position_measured_),
       position_measured_or_zero_->ticket()});

  commanded_position_output_ = &DeclareVectorOutputPort(
      "position", num_joints, &IiwaCommandReceiver::CalcPositionOutput,
      {defaulted_command_->ticket()});

  commanded_torque_output_ = &DeclareVectorOutputPort(
      "torque", num_joints, &IiwaCommandReceiver::CalcTorqueOutput,
      {defaulted_command_->ticket()});
}

IiwaCommandReceiver::~IiwaCommandReceiver() = default;

void IiwaCommandReceiver::CalcPositionMeasuredOrZero(
    const Context<double>& context,
    BasicVector<double>* result) const {
  if (position_measured_input_->HasValue(context)) {
    result->SetFromVector(position_measured_input_->Eval(context));
  } else {
    result->SetZero();
  }
}

void IiwaCommandReceiver::LatchInitialPosition(
    const Context<double>& context,
    DiscreteValues<double>* result) const {
  const auto& bool_index = latched_position_measured_is_set_;
  const auto& value_index = latched_position_measured_;
  result->get_mutable_value(bool_index)[0] = 1.0;
  result->get_mutable_vector(value_index).SetFrom(
      position_measured_or_zero_->Eval<BasicVector<double>>(context));
}

void IiwaCommandReceiver::LatchInitialPosition(
    Context<double>* context) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  LatchInitialPosition(*context, &context->get_mutable_discrete_state());
}

// TODO(jwnimmer-tri) This is quite a cumbersome syntax to use for declaring a
// "now" event.  We should try to consolidate it with other similar uses within
// the source tree.  Relates to #11403 somewhat.
void IiwaCommandReceiver::DoCalcNextUpdateTime(
    const Context<double>& context,
    CompositeEventCollection<double>* events, double* time) const {
  // We do not support events other than our own message timing events.
  LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
  DRAKE_THROW_UNLESS(events->HasEvents() == false);
  DRAKE_THROW_UNLESS(std::isinf(*time));

  // If we have a latched position already, then we do not have any updates.
  if (context.get_discrete_state(0).get_value()[0] != 0.0) {
    return;
  }

  // Schedule a discrete update event at now to latch the current position.
  *time = context.get_time();
  auto& discrete_events = events->get_mutable_discrete_update_events();
  discrete_events.AddEvent(DiscreteUpdateEvent<double>(
      [this](const Context<double>& event_context,
             const DiscreteUpdateEvent<double>&,
             DiscreteValues<double>* next_values) {
        LatchInitialPosition(event_context, next_values);
      }));
}

void IiwaCommandReceiver::CalcDefaultedCommand(
    const Context<double>& context, lcmt_iiwa_command* result) const {
  // Copy the input value into our tentative result.
  *result = message_input_->Eval<lcmt_iiwa_command>(context);

  // If we haven't received a message yet, then fall back to the default
  // position.
  if (AreLcmMessagesEqual(*result, lcmt_iiwa_command{})) {
    const BasicVector<double>& latch_is_set = context.get_discrete_state(
        latched_position_measured_is_set_);
    const BasicVector<double>& default_position =
        latch_is_set[0]
         ? context.get_discrete_state(latched_position_measured_)
         : position_measured_or_zero_->Eval<BasicVector<double>>(context);
    const VectorXd vec = default_position.CopyToVector();
    result->num_joints = vec.size();
    result->joint_position = {vec.data(), vec.data() + vec.size()};
  }
}

void IiwaCommandReceiver::CalcPositionOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& message = defaulted_command_->Eval<lcmt_iiwa_command>(context);
  if (message.num_joints != num_joints_) {
    throw std::runtime_error(fmt::format(
        "IiwaCommandReceiver expected num_joints = {}, but received {}",
        num_joints_, message.num_joints));
  }
  output->SetFromVector(Eigen::Map<const VectorXd>(
      message.joint_position.data(),
      message.joint_position.size()));
}

void IiwaCommandReceiver::CalcTorqueOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& message = defaulted_command_->Eval<lcmt_iiwa_command>(context);
  if (message.num_torques == 0) {
    // If torques were not sent, use zeros.
    output->SetZero();
    return;
  }
  if (message.num_torques != num_joints_) {
    throw std::runtime_error(fmt::format(
        "IiwaCommandReceiver expected num_torques = {}, but received {}",
        num_joints_, message.num_torques));
  }
  output->SetFromVector(Eigen::Map<const VectorXd>(
      message.joint_torque.data(),
      message.joint_torque.size()));
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
