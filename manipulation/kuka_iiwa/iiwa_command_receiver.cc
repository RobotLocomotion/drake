#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/lcm_messages.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

using Eigen::VectorXd;
using systems::BasicVector;
using systems::CacheEntry;
using systems::Context;
using systems::DiscreteValues;
using systems::DiscreteStateIndex;
using systems::DiscreteUpdateEvent;
using systems::InputPortIndex;
using systems::NumericParameterIndex;

using InPort = systems::InputPort<double>;

IiwaCommandReceiver::IiwaCommandReceiver(int num_joints) {
  DRAKE_THROW_UNLESS(num_joints > 0);
  message_input_ = &DeclareAbstractInputPort(
      "lcmt_iiwa_command", Value<lcmt_iiwa_command>());
  position_measured_input_ = &DeclareInputPort(
      "position_measured", systems::kVectorValued, num_joints);

  // This (deprecated) parameter stores a value to use for the initial position
  // iff the position_measured input is not connected.
  //
  // On 2020-09-01 when we remove the set_initial_position method, we should
  // also remove this cache entry and parameter and just use the input port
  // directly (and use zero when the port is not connected).
  const NumericParameterIndex initial_state_param{
      DeclareNumericParameter(BasicVector<double>(VectorXd::Zero(num_joints)))};
  const CacheEntry* const position_measured_or_param =
    &DeclareCacheEntry<BasicVector<double>>(
      "position_measured_or_param",
      BasicVector<double>(num_joints),
      [this,
       initial_state_param](
          const Context<double>& context, BasicVector<double>* result) {
        if (position_measured_input_->HasValue(context)) {
          result->SetFromVector(position_measured_input_->Eval(context));
        } else {
          result->SetFrom(context.get_numeric_parameter(initial_state_param));
        }
      }, {
        position_measured_input_->ticket(),
        numeric_parameter_ticket(initial_state_param),
      });

  // When a simulation begins, we will latch position_measured_or_param into a
  // state variable, so that we will hold that pose until the first message is
  // received.  Prior to that event, we continue to use the unlatched value.
  const DiscreteStateIndex latched_position_measured_is_set =
      DeclareDiscreteState(VectorXd::Zero(1));
  const DiscreteStateIndex latched_position_measured =
      DeclareDiscreteState(VectorXd::Zero(num_joints));
  cached_outputs_ = &DeclareCacheEntry<lcmt_iiwa_command>(
      "cached_outputs",
      lcmt_iiwa_command{},
      [num_joints,
       this,
       latched_position_measured_is_set,
       latched_position_measured,
       position_measured_or_param](
          const Context<double>& context, lcmt_iiwa_command* result) {
        // Copy the input value into our tentative result.
        *result = message_input_->Eval<lcmt_iiwa_command>(context);
        // If we haven't received a message yet, then fall back to the default
        // position with zero torques.
        if (lcm::AreLcmMessagesEqual(*result, lcmt_iiwa_command{})) {
          const BasicVector<double>& latch_is_set = context.get_discrete_state(
              latched_position_measured_is_set);
          const BasicVector<double>& latched_value = context.get_discrete_state(
              latched_position_measured);
          const BasicVector<double>& measured_value =
              position_measured_or_param->Eval<BasicVector<double>>(context);
          const VectorXd positions =
              latch_is_set[0]
              ? latched_value.CopyToVector()
              : measured_value.CopyToVector();
          result->num_joints = positions.size();
          result->joint_position =
              {positions.data(), positions.data() + positions.size()};
        }
        // If torques were not sent, pad with zeros.
        if (result->num_torques == 0) {
          result->num_torques = num_joints;
          result->joint_torque.resize(num_joints, 0.0);
        }
      }, {
        message_input_->ticket(),
        discrete_state_ticket(latched_position_measured_is_set),
        discrete_state_ticket(latched_position_measured),
        position_measured_or_param->ticket(),
      });

  DeclareVectorOutputPort(
      "position", BasicVector<double>(num_joints),
      &IiwaCommandReceiver::CalcPositionOutput,
      {cached_outputs_->ticket()});

  DeclareVectorOutputPort(
      "torque", BasicVector<double>(num_joints),
      &IiwaCommandReceiver::CalcTorqueOutput,
      {cached_outputs_->ticket()});
}

void IiwaCommandReceiver::set_initial_position(
    Context<double>* context, const Eigen::Ref<const VectorXd>& q) const {
  context->get_mutable_numeric_parameter(0).SetFromVector(q);
}

namespace {
void DoLatchInitialPosition(
    const InPort& position_measured,
    const systems::Context<double>& context,
    DiscreteValues<double>* next_values) {
  const VectorXd positions =
      position_measured.HasValue(context)
      ? position_measured.Eval(context)
      : context.get_numeric_parameter(0).get_value();
  next_values->get_mutable_vector(0).get_mutable_value()[0] = 1.0;
  next_values->get_mutable_vector(1).get_mutable_value() = positions;
}
}  // namespace

void IiwaCommandReceiver::LatchInitialPosition(
    systems::Context<double>* context) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  DoLatchInitialPosition(
      get_position_measured_input_port(), *context,
      &context->get_mutable_discrete_state());
}

void IiwaCommandReceiver::DoCalcNextUpdateTime(
    const systems::Context<double>& context,
    systems::CompositeEventCollection<double>* events, double* time) const {
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
  const InPort* const position_measured = &get_position_measured_input_port();
  discrete_events.add_event(std::make_unique<DiscreteUpdateEvent<double>>(
      [position_measured](
          const Context<double>& event_context,
          const DiscreteUpdateEvent<double>&,
          DiscreteValues<double>* next_values) {
        DoLatchInitialPosition(*position_measured, event_context, next_values);
      }));
}

void IiwaCommandReceiver::CalcPositionOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const int num_joints = output->size();
  const auto& message = cached_outputs_->Eval<lcmt_iiwa_command>(context);
  if (message.num_joints != num_joints) {
    throw std::runtime_error(fmt::format(
        "IiwaCommandReceiver expected num_joints = {}, but received {}",
        num_joints, message.num_joints));
  }
  output->SetFromVector(Eigen::Map<const VectorXd>(
      message.joint_position.data(),
      message.joint_position.size()));
}

void IiwaCommandReceiver::CalcTorqueOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const int num_joints = output->size();
  const auto& message = cached_outputs_->Eval<lcmt_iiwa_command>(context);
  if (message.num_torques != num_joints) {
    throw std::runtime_error(fmt::format(
        "IiwaCommandReceiver expected num_torques = {}, but received {}",
        num_joints, message.num_torques));
  }
  output->SetFromVector(Eigen::Map<const VectorXd>(
      message.joint_torque.data(),
      message.joint_torque.size()));
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
