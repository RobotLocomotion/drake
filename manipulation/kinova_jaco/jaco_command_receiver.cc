#include "drake/manipulation/kinova_jaco/jaco_command_receiver.h"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/lcm_messages.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {

using Eigen::VectorXd;
using systems::BasicVector;
using systems::CompositeEventCollection;
using systems::Context;
using systems::DiscreteUpdateEvent;
using systems::DiscreteValues;
using systems::kVectorValued;

JacoCommandReceiver::JacoCommandReceiver(int num_joints, int num_fingers)
    : num_joints_(num_joints), num_fingers_(num_fingers) {
  message_input_ = &DeclareAbstractInputPort("lcmt_jaco_command",
                                             Value<lcmt_jaco_command>());
  position_measured_input_ = &DeclareInputPort(
      "position_measured", kVectorValued, num_joints + num_fingers);

  // This cache entry provides either the input (iff connected) or else zero.
  position_measured_or_zero_ =
      &DeclareCacheEntry("position_measured_or_zero",
                         BasicVector<double>(num_joints + num_fingers),
                         &JacoCommandReceiver::CalcPositionMeasuredOrZero,
                         {position_measured_input_->ticket()});

  // When a simulation begins, we will latch positions into a state variable,
  // so that we will hold that pose until the first message is received.
  // Prior to that event, we continue to use the unlatched value.
  latched_position_measured_is_set_ = DeclareDiscreteState(VectorXd::Zero(1));
  latched_position_measured_ =
      DeclareDiscreteState(VectorXd::Zero(num_joints + num_fingers));

  groomed_input_ = &DeclareCacheEntry(
      "groomed_input", &JacoCommandReceiver::CalcInput,
      {message_input_->ticket(),
       discrete_state_ticket(latched_position_measured_is_set_),
       discrete_state_ticket(latched_position_measured_),
       position_measured_or_zero_->ticket()});

  commanded_position_output_ = &DeclareVectorOutputPort(
      "position", num_joints + num_fingers,
      &JacoCommandReceiver::CalcPositionOutput, {groomed_input_->ticket()});

  commanded_velocity_output_ = &DeclareVectorOutputPort(
      "velocity", num_joints + num_fingers,
      &JacoCommandReceiver::CalcVelocityOutput, {groomed_input_->ticket()});

  time_output_ =
      &DeclareVectorOutputPort("time", 1, &JacoCommandReceiver::CalcTimeOutput,
                               {groomed_input_->ticket()});
}

void JacoCommandReceiver::CalcPositionMeasuredOrZero(
    const Context<double>& context, BasicVector<double>* result) const {
  if (position_measured_input_->HasValue(context)) {
    result->SetFromVector(position_measured_input_->Eval(context));
  } else {
    result->SetZero();
  }
}

void JacoCommandReceiver::LatchInitialPosition(
    const Context<double>& context, DiscreteValues<double>* result) const {
  const auto& bool_index = latched_position_measured_is_set_;
  const auto& value_index = latched_position_measured_;
  result->get_mutable_value(bool_index)[0] = 1.0;
  result->get_mutable_vector(value_index)
      .SetFrom(position_measured_or_zero_->Eval<BasicVector<double>>(context));
}

void JacoCommandReceiver::LatchInitialPosition(Context<double>* context) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  LatchInitialPosition(*context, &context->get_mutable_discrete_state());
}

// TODO(jwnimmer-tri) This is quite a cumbersome syntax to use for declaring a
// "now" event.  We should try to consolidate it with other similar uses within
// the source tree.  Relates to #11403 somewhat.
void JacoCommandReceiver::DoCalcNextUpdateTime(
    const Context<double>& context, CompositeEventCollection<double>* events,
    double* time) const {
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
      [this](const System<double>&, const Context<double>& event_context,
             const DiscreteUpdateEvent<double>&,
             DiscreteValues<double>* next_values) {
        LatchInitialPosition(event_context, next_values);
        return systems::EventStatus::Succeeded();
      }));
}

// Returns (in "result") the command message input, or if a message has not
// been received yet returns the initial command (as optionally set by the
// user).  The result will always have num_joints_ positions and velocities.
void JacoCommandReceiver::CalcInput(const Context<double>& context,
                                    lcmt_jaco_command* result) const {
  if (!get_message_input_port().HasValue(context)) {
    throw std::logic_error("JacoCommandReceiver has no input connected");
  }

  // Copy the input value into our tentative result.
  *result = get_message_input_port().Eval<lcmt_jaco_command>(context);

  // If we haven't received a non-default message yet, use the initial command.
  // N.B. This works due to lcm::Serializer<>::CreateDefaultValue() using
  // value-initialization.
  if (lcm::AreLcmMessagesEqual(*result, lcmt_jaco_command{})) {
    const BasicVector<double>& latch_is_set =
        context.get_discrete_state(latched_position_measured_is_set_);
    const BasicVector<double>& default_position =
        latch_is_set[0]
            ? context.get_discrete_state(latched_position_measured_)
            : position_measured_or_zero_->Eval<BasicVector<double>>(context);

    result->num_joints = num_joints_;
    const VectorXd& vec = default_position.value();
    result->joint_position = {vec.data(), vec.data() + num_joints_};
    result->joint_velocity.resize(num_joints_, 0);

    result->num_fingers = num_fingers_;
    if (num_fingers_) {
      result->finger_position = {vec.data() + num_joints_,
                                 vec.data() + num_joints_ + num_fingers_};
      result->finger_velocity.resize(num_fingers_, 0);
    } else {
      result->finger_position.clear();
      result->finger_velocity.clear();
    }
  } else {
    for (int i = 0; i < result->num_fingers; ++i) {
      result->finger_position[i] *= kFingerSdkToUrdf;
      result->finger_velocity[i] *= kFingerSdkToUrdf;
    }
  }

  // Sanity check the joint sizes.
  if (result->num_joints != num_joints_) {
    throw std::runtime_error(fmt::format(
        "JacoCommandReceiver expected num_joints = {}, but received {}",
        num_joints_, result->num_joints));
  }

  if (result->num_fingers != num_fingers_) {
    throw std::runtime_error(fmt::format(
        "JacoCommandReceiver expected num_fingers = {}, but received {}",
        num_fingers_, result->num_fingers));
  }
}

void JacoCommandReceiver::CalcPositionOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& message = groomed_input_->Eval<lcmt_jaco_command>(context);
  if (message.num_joints != num_joints_) {
    throw std::runtime_error(fmt::format(
        "JacoCommandReceiver expected num_joints = {}, but received {}",
        num_joints_, message.num_joints));
  }
  if (message.num_fingers != num_fingers_) {
    throw std::runtime_error(fmt::format(
        "JacoCommandReceiver expected num_fingers = {}, but received {}",
        num_fingers_, message.num_fingers));
  }

  Eigen::VectorXd position(num_joints_ + num_fingers_);
  position.head(num_joints_) = Eigen::Map<const VectorXd>(
      message.joint_position.data(), message.joint_position.size());
  if (num_fingers_) {
    position.segment(num_joints_, num_fingers_) = Eigen::Map<const VectorXd>(
        message.finger_position.data(), message.finger_position.size());
  }

  output->SetFromVector(position);
}

void JacoCommandReceiver::CalcVelocityOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& message = groomed_input_->Eval<lcmt_jaco_command>(context);
  if (message.num_joints != num_joints_) {
    throw std::runtime_error(fmt::format(
        "JacoCommandReceiver expected num_joints = {}, but received {}",
        num_joints_, message.num_joints));
  }
  if (message.num_fingers != num_fingers_) {
    throw std::runtime_error(fmt::format(
        "JacoCommandReceiver expected num_fingers = {}, but received {}",
        num_fingers_, message.num_fingers));
  }

  Eigen::VectorXd velocity(num_joints_ + num_fingers_);
  velocity.head(num_joints_) = Eigen::Map<const VectorXd>(
      message.joint_velocity.data(), message.joint_velocity.size());
  if (num_fingers_) {
    velocity.segment(num_joints_, num_fingers_) = Eigen::Map<const VectorXd>(
        message.finger_velocity.data(), message.finger_velocity.size());
  }

  output->SetFromVector(velocity);
}

void JacoCommandReceiver::CalcTimeOutput(const Context<double>& context,
                                         BasicVector<double>* output) const {
  const auto& message = groomed_input_->Eval<lcmt_jaco_command>(context);
  (*output)[0] = static_cast<double>(message.utime) / 1e6;
}

}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
