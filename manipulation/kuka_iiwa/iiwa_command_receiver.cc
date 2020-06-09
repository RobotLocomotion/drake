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

IiwaCommandReceiver::IiwaCommandReceiver(int num_joints) {
  DRAKE_THROW_UNLESS(num_joints > 0);
  const auto* const message_port = &DeclareAbstractInputPort(
      "lcmt_iiwa_command", Value<lcmt_iiwa_command>());
  const auto* const position_measured_port = &DeclareInputPort(
      "position_measured", systems::kVectorValued, num_joints);

  // This (deprecated) parameter stores a value to use for the initial position
  // iff the position_measured input is not connected.
  //
  // On 2020-09-01 when we remove the set_initial_position method, we should
  // also remove this cache entry and parameter and just use the input port
  // directly (or use zero when the port is not connected).
  const NumericParameterIndex initial_state_param{
      DeclareNumericParameter(BasicVector<double>(VectorXd::Zero(num_joints)))};
  const CacheEntry* const position_measured_or_param =
    &DeclareCacheEntry<BasicVector<double>>(
      "position_measured_or_param",
      BasicVector<double>(num_joints),
      [position_measured_port,
       initial_state_param](
          const Context<double>& context, BasicVector<double>* result) {
        if (position_measured_port->HasValue(context)) {
          result->SetFromVector(position_measured_port->Eval(context));
        } else {
          result->SetFrom(context.get_numeric_parameter(initial_state_param));
        }
      }, {
        position_measured_port->ticket(),
        numeric_parameter_ticket(initial_state_param),
      });

  // When a simulation begins, we will latch position_measured_or_param into a
  // state variable, so that we will hold that pose until the first message is
  // received.  Prior to that event, we continue to use the unlatched value.
  const DiscreteStateIndex latched_position_measured_is_set =
      DeclareDiscreteState(VectorXd::Zero(1));
  const DiscreteStateIndex latched_position_measured =
      DeclareDiscreteState(VectorXd::Zero(num_joints));
  const CacheEntry* const fused_input_and_state =
    &DeclareCacheEntry<lcmt_iiwa_command>(
      "fused_input_and_state",
      lcmt_iiwa_command{},
      [num_joints,
       message_port,
       latched_position_measured_is_set,
       latched_position_measured,
       position_measured_or_param](
          const Context<double>& context, lcmt_iiwa_command* result) {
        // Copy the input value into our tentative result.
        *result = message_port->Eval<lcmt_iiwa_command>(context);
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
        message_port->ticket(),
        discrete_state_ticket(latched_position_measured_is_set),
        discrete_state_ticket(latched_position_measured),
        position_measured_or_param->ticket(),
      });

  DeclareVectorOutputPort(
      "position", BasicVector<double>(num_joints),
      [num_joints,
       fused_input_and_state](
          const Context<double>& context, BasicVector<double>* output) {
        const auto& message =
            fused_input_and_state->Eval<lcmt_iiwa_command>(context);
        if (message.num_joints != num_joints) {
          throw std::runtime_error(fmt::format(
              "IiwaCommandReceiver expected num_joints = {}, but received {}",
              num_joints, message.num_joints));
        }
        output->SetFromVector(Eigen::Map<const VectorXd>(
            message.joint_position.data(),
            message.joint_position.size()));
      },
      {fused_input_and_state->ticket()});

  DeclareVectorOutputPort(
      "torque", BasicVector<double>(num_joints),
      [num_joints,
       fused_input_and_state](
          const Context<double>& context, BasicVector<double>* output) {
        const auto& message =
            fused_input_and_state->Eval<lcmt_iiwa_command>(context);
        if (message.num_torques != num_joints) {
          throw std::runtime_error(fmt::format(
              "IiwaCommandReceiver expected num_torques = {}, but received {}",
              num_joints, message.num_torques));
        }
        output->SetFromVector(Eigen::Map<const VectorXd>(
            message.joint_torque.data(),
            message.joint_torque.size()));
      },
      {fused_input_and_state->ticket()});
}

using InPort = systems::InputPort<double>;
const InPort& IiwaCommandReceiver::get_message_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}
const InPort& IiwaCommandReceiver::get_position_measured_input_port() const {
  return LeafSystem<double>::get_input_port(1);
}
using OutPort = systems::OutputPort<double>;
const OutPort& IiwaCommandReceiver::get_commanded_position_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}
const OutPort& IiwaCommandReceiver::get_commanded_torque_output_port() const {
  return LeafSystem<double>::get_output_port(1);
}

void IiwaCommandReceiver::set_initial_position(
    Context<double>* context, const Eigen::Ref<const VectorXd>& q) const {
  context->get_mutable_numeric_parameter(0).SetFromVector(q);
}

namespace {
void DoLatchInitialPosition(
    const InPort& position_measured,
    const systems::Context<double>& context,
    DiscreteValues<double>* new_discrete_values) {
  const VectorXd positions =
      position_measured.HasValue(context)
      ? position_measured.Eval(context)
      : context.get_numeric_parameter(0).get_value();
  new_discrete_values->get_mutable_vector(0).get_mutable_value()[0] = 1.0;
  new_discrete_values->get_mutable_vector(1).get_mutable_value() = positions;
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

  // Schedule a discrete update event at now to set the position command.
  *time = context.get_time();
  auto& discrete_events = events->get_mutable_discrete_update_events();
  discrete_events.add_event(std::make_unique<DiscreteUpdateEvent<double>>(
      [this](
          const Context<double>& event_context,
          const DiscreteUpdateEvent<double>&,
          DiscreteValues<double>* new_discrete_values) {
        DoLatchInitialPosition(this->get_position_measured_input_port(),
            event_context, new_discrete_values);
      }));
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
