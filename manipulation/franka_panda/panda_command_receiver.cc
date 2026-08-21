#include "drake/manipulation/franka_panda/panda_command_receiver.h"

#include <string>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/lcm/lcm_messages.h"
#include "drake/lcmt_panda_status.hpp"
#include "drake/manipulation/franka_panda/panda_constants.h"

namespace drake {
namespace manipulation {
namespace franka_panda {

using drake::lcmt_panda_command;
using drake::lcmt_panda_status;
using drake::Value;
using drake::lcm::AreLcmMessagesEqual;
using drake::systems::BasicVector;
using drake::systems::CompositeEventCollection;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::kVectorValued;
using Eigen::VectorXd;

PandaCommandReceiver::PandaCommandReceiver(int num_joints,
                                           PandaControlMode control_mode)
    : num_joints_(num_joints), control_mode_(control_mode) {
  DRAKE_THROW_UNLESS(num_joints > 0);

  message_input_ = &DeclareAbstractInputPort("lcmt_panda_command",
                                             Value<lcmt_panda_command>());
  position_measured_input_ =
      &DeclareInputPort("position_measured", kVectorValued, num_joints);

  // Even if we have not yet received a message, we still need to provide
  // default values for our output ports.  We use a cache entry to compute
  // those values, by populating a default lcmt_panda_control message.
  if ((control_mode_ & PandaControlModes::kPosition) !=
      PandaControlModes::kNone) {
    // When in position control mode, the default is derived from the
    // "position_measured" input.
    latched_position_measured_is_set_ = DeclareDiscreteState(VectorXd::Zero(1));
    latched_position_measured_ =
        DeclareDiscreteState(VectorXd::Zero(num_joints));
    default_command_ = &DeclareCacheEntry(
        "default_command", &PandaCommandReceiver::CalcDefaultCommand,
        {discrete_state_ticket(latched_position_measured_is_set_),
         discrete_state_ticket(latched_position_measured_),
         position_measured_input_->ticket()});
  } else {
    // When not in position control mode, the default is just zeros.
    default_command_ = &DeclareCacheEntry(
        "default_command", &PandaCommandReceiver::CalcDefaultCommand,
        {nothing_ticket()});
  }

  // For the convenience of our output calc functions, this cache entry
  // provides either the "message_input" (iff a message has been received)
  // or else the "default_command".
  message_input_or_default_ = &DeclareCacheEntry(
      "message_input_or_default",
      &PandaCommandReceiver::CalcMessageInputOrDefault,
      {message_input_->ticket(), default_command_->ticket()});

  PandaControlMode remaining = control_mode_;
  if ((control_mode_ & PandaControlModes::kPosition) !=
      PandaControlModes::kNone) {
    remaining &= ~PandaControlModes::kPosition;
    commanded_position_output_ =
        &DeclareVectorOutputPort("position", BasicVector<double>(num_joints),
                                 &PandaCommandReceiver::CalcPositionOutput,
                                 {message_input_or_default_->ticket()});
  }

  if ((control_mode_ & PandaControlModes::kVelocity) !=
      PandaControlModes::kNone) {
    remaining &= ~PandaControlModes::kVelocity;
    commanded_velocity_output_ =
        &DeclareVectorOutputPort("velocity", BasicVector<double>(num_joints),
                                 &PandaCommandReceiver::CalcVelocityOutput,
                                 {message_input_or_default_->ticket()});
  }

  if ((control_mode_ & PandaControlModes::kTorque) !=
      PandaControlModes::kNone) {
    remaining &= ~PandaControlModes::kTorque;
    commanded_torque_output_ =
        &DeclareVectorOutputPort("torque", BasicVector<double>(num_joints),
                                 &PandaCommandReceiver::CalcTorqueOutput,
                                 {message_input_or_default_->ticket()});
  }

  if (remaining != PandaControlModes::kNone) {
    throw std::logic_error(fmt::format("Invalid control_mode bits set: 0x{:x}",
                                       to_int(remaining)));
  }
}

PandaCommandReceiver::~PandaCommandReceiver() = default;

const drake::systems::OutputPort<double>&
PandaCommandReceiver::get_commanded_position_output_port() const {
  if (commanded_position_output_ == nullptr) {
    throw std::runtime_error(
        "Invalid call to PandaCommandReceiver::get_command_position_output_port"
        " when control_mode does not include position");
  }
  return *commanded_position_output_;
}

const drake::systems::OutputPort<double>&
PandaCommandReceiver::get_commanded_velocity_output_port() const {
  if (commanded_velocity_output_ == nullptr) {
    throw std::runtime_error(
        "Invalid call to PandaCommandReceiver::get_command_velocity_output_port"
        " when control_mode does not include velocity");
  }
  return *commanded_velocity_output_;
}

const drake::systems::OutputPort<double>&
PandaCommandReceiver::get_commanded_torque_output_port() const {
  if (commanded_torque_output_ == nullptr) {
    throw std::runtime_error(
        "Invalid call to PandaCommandReceiver::get_command_torque_output_port"
        " when control_mode does not include torque");
  }
  return *commanded_torque_output_;
}

void PandaCommandReceiver::LatchInitialPosition(
    const Context<double>& context, DiscreteValues<double>* result) const {
  DRAKE_DEMAND((control_mode_ & PandaControlModes::kPosition) !=
               PandaControlModes::kNone);
  const auto& bool_index = latched_position_measured_is_set_;
  const auto& value_index = latched_position_measured_;
  result->get_mutable_vector(bool_index).get_mutable_value()[0] = 1.0;
  result->get_mutable_vector(value_index)
      .SetFrom(position_measured_input_->Eval<BasicVector<double>>(context));
}

void PandaCommandReceiver::LatchInitialPosition(
    Context<double>* context) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  DRAKE_THROW_UNLESS((control_mode_ & PandaControlModes::kPosition) !=
                     PandaControlModes::kNone);
  LatchInitialPosition(*context, &context->get_mutable_discrete_state());
}

// TODO(jwnimmer-tri) This is quite a cumbersome syntax to use for declaring a
// "now" event.  We should try to consolidate it with other similar uses within
// the source tree.  Relates to #11403 somewhat.
void PandaCommandReceiver::DoCalcNextUpdateTime(
    const Context<double>& context, CompositeEventCollection<double>* events,
    double* time) const {
  // We do not support events other than our own message timing events.
  LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
  DRAKE_THROW_UNLESS(events->HasEvents() == false);
  DRAKE_THROW_UNLESS(std::isinf(*time));

  // If we're not using position control, then we have no state to latch.
  if ((control_mode_ & PandaControlModes::kPosition) ==
      PandaControlModes::kNone) {
    return;
  }

  // If we have a latched position already, then we do not have any updates.
  if (context.get_discrete_state(0).get_value()[0] != 0.0) {
    return;
  }

  // Schedule a discrete update event at now to latch the current position.
  *time = context.get_time();
  auto& discrete_events = events->get_mutable_discrete_update_events();
  discrete_events.AddEvent(DiscreteUpdateEvent<double>(
      [this](const System<double>&, const Context<double>& event_context,
             const DiscreteUpdateEvent<double>&,
             DiscreteValues<double>* next_values) {
        LatchInitialPosition(event_context, next_values);
        return drake::systems::EventStatus::Succeeded();
      }));
}

void PandaCommandReceiver::CalcDefaultCommand(
    const Context<double>& context, lcmt_panda_command* result) const {
  *result = {};
  result->control_mode_expected = to_int(control_mode_);
  if ((control_mode_ & PandaControlModes::kPosition) !=
      PandaControlModes::kNone) {
    const BasicVector<double>& latch_is_set =
        context.get_discrete_state(latched_position_measured_is_set_);
    const BasicVector<double>& default_position =
        latch_is_set[0]
            ? context.get_discrete_state(latched_position_measured_)
            : position_measured_input_->Eval<BasicVector<double>>(context);
    const VectorXd vec = default_position.CopyToVector();
    DRAKE_DEMAND(vec.size() == num_joints_);
    result->num_joint_position = num_joints_;
    result->joint_position = {vec.data(), vec.data() + num_joints_};
  }
  if ((control_mode_ & PandaControlModes::kVelocity) !=
      PandaControlModes::kNone) {
    result->num_joint_velocity = num_joints_;
    result->joint_velocity = std::vector<double>(num_joints_, 0.0);
  }
  if ((control_mode_ & PandaControlModes::kTorque) !=
      PandaControlModes::kNone) {
    result->num_joint_torque = num_joints_;
    result->joint_torque = std::vector<double>(num_joints_, 0.0);
  }
}

void PandaCommandReceiver::CalcMessageInputOrDefault(
    const Context<double>& context, lcmt_panda_command* result) const {
  // Copy the input value into our tentative result.
  *result = message_input_->Eval<lcmt_panda_command>(context);
  if (AreLcmMessagesEqual(*result, lcmt_panda_command{})) {
    // We haven't received a message yet; fall back to the default outputs.
    *result = default_command_->Eval<lcmt_panda_command>(context);
  } else {
    // We have received a message; validate it.
    // TODO(jeremy.nimmer) Here is where we should check control_mode_ vs
    // result->control_mode_expected.
  }
}

namespace {
void CheckNumJoints(const std::string& system_name, const char* vector_name,
                    int actual, int expected) {
  if (actual != expected) {
    throw std::runtime_error(fmt::format(
        "PandaCommandReceiver {} expected {} {} joints, but received {}",
        system_name, expected, vector_name, actual));
  }
}
}  // namespace

void PandaCommandReceiver::CalcPositionOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  DRAKE_DEMAND((control_mode_ & PandaControlModes::kPosition) !=
               PandaControlModes::kNone);
  const auto& message =
      message_input_or_default_->Eval<lcmt_panda_command>(context);
  DRAKE_DEMAND(message.control_mode_expected == to_int(control_mode_));
  CheckNumJoints(get_name(), "position", message.num_joint_position,
                 num_joints_);
  CheckNumJoints(get_name(), "position", message.joint_position.size(),
                 num_joints_);
  output->SetFromVector(Eigen::Map<const Eigen::VectorXd>(
      message.joint_position.data(), message.joint_position.size()));
}

void PandaCommandReceiver::CalcVelocityOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  DRAKE_DEMAND((control_mode_ & PandaControlModes::kVelocity) !=
               PandaControlModes::kNone);
  const auto& message =
      message_input_or_default_->Eval<lcmt_panda_command>(context);
  DRAKE_DEMAND(message.control_mode_expected == to_int(control_mode_));
  CheckNumJoints(get_name(), "velocity", message.num_joint_velocity,
                 num_joints_);
  CheckNumJoints(get_name(), "velocity", message.joint_velocity.size(),
                 num_joints_);
  output->SetFromVector(Eigen::Map<const Eigen::VectorXd>(
      message.joint_velocity.data(), message.joint_velocity.size()));
}

void PandaCommandReceiver::CalcTorqueOutput(const Context<double>& context,
                                            BasicVector<double>* output) const {
  DRAKE_DEMAND((control_mode_ & PandaControlModes::kTorque) !=
               PandaControlModes::kNone);
  const auto& message =
      message_input_or_default_->Eval<lcmt_panda_command>(context);
  DRAKE_DEMAND(message.control_mode_expected == to_int(control_mode_));
  CheckNumJoints(get_name(), "torque", message.num_joint_torque, num_joints_);
  CheckNumJoints(get_name(), "torque", message.joint_torque.size(),
                 num_joints_);
  output->SetFromVector(Eigen::Map<const Eigen::VectorXd>(
      message.joint_torque.data(), message.joint_torque.size()));
}

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
