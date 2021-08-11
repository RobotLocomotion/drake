#include "drake/systems/lcm/lcm_publisher_system.h"

#include <cstdint>
#include <utility>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"

namespace drake {
namespace systems {
namespace lcm {

using drake::lcm::DrakeLcmInterface;
using drake::lcm::DrakeLcm;
using systems::TriggerTypeSet;

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    std::unique_ptr<SerializerInterface> serializer,
    DrakeLcmInterface* lcm,
    const TriggerTypeSet& publish_triggers,
    double publish_period)
    : channel_(channel),
      serializer_(std::move(serializer)),
      owned_lcm_(lcm ? nullptr : new DrakeLcm()),
      lcm_(lcm ? lcm : owned_lcm_.get()),
      publish_period_(publish_period) {
  DRAKE_DEMAND(serializer_ != nullptr);
  DRAKE_DEMAND(lcm_ != nullptr);
  DRAKE_DEMAND(publish_period >= 0.0);
  DRAKE_DEMAND(!publish_triggers.empty());

  // Check that publish_triggers does not contain an unsupported trigger.
  for (const auto& trigger : publish_triggers) {
      DRAKE_THROW_UNLESS((trigger == TriggerType::kForced) ||
        (trigger == TriggerType::kPeriodic) ||
        (trigger == TriggerType::kPerStep));
  }

  // Declare a forced publish so that any time Publish(.) is called on this
  // system (or a Diagram containing it), a message is emitted.
  if (publish_triggers.find(TriggerType::kForced) != publish_triggers.end()) {
    this->DeclareForcedPublishEvent(
      &LcmPublisherSystem::PublishInputAsLcmMessage);
  }

  DeclareAbstractInputPort("lcm_message", *serializer_->CreateDefaultValue());

  set_name(make_name(channel_));
  if (publish_triggers.find(TriggerType::kPeriodic) != publish_triggers.end()) {
    DRAKE_THROW_UNLESS(publish_period > 0.0);
    const double offset = 0.0;
    this->DeclarePeriodicPublishEvent(
        publish_period, offset,
        &LcmPublisherSystem::PublishInputAsLcmMessage);
  } else {
    // publish_period > 0 without TriggerType::kPeriodic has no meaning and is
    // likely a mistake.
    DRAKE_THROW_UNLESS(publish_period == 0.0);
  }

  if (publish_triggers.find(TriggerType::kPerStep) != publish_triggers.end()) {
    this->DeclarePerStepEvent(
    systems::PublishEvent<double>([this](
        const systems::Context<double>& context,
        const systems::PublishEvent<double>&) {
      this->PublishInputAsLcmMessage(context);
    }));
  }
}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    std::unique_ptr<SerializerInterface> serializer,
    DrakeLcmInterface* lcm, double publish_period)
    : LcmPublisherSystem(channel, std::move(serializer), lcm,
      (publish_period > 0.0) ?
      TriggerTypeSet({TriggerType::kForced, TriggerType::kPeriodic}) :
      TriggerTypeSet({TriggerType::kForced, TriggerType::kPerStep}),
      publish_period) {}

LcmPublisherSystem::~LcmPublisherSystem() {}

void LcmPublisherSystem::AddInitializationMessage(
    InitializationPublisher initialization_publisher) {
  DRAKE_DEMAND(initialization_publisher != nullptr);

  initialization_publisher_ = std::move(initialization_publisher);

  DeclareInitializationEvent(systems::PublishEvent<double>(
      TriggerType::kInitialization,
      [this](const systems::Context<double>& context,
             const systems::PublishEvent<double>&) {
        this->initialization_publisher_(context, this->lcm_);
      }));
}

std::string LcmPublisherSystem::make_name(const std::string& channel) {
  return "LcmPublisherSystem(" + channel + ")";
}

const std::string& LcmPublisherSystem::get_channel_name() const {
  return channel_;
}

double LcmPublisherSystem::get_publish_period() const {
  return publish_period_;
}

// Takes the VectorBase from the input port of the context and publishes
// it onto an LCM channel. This function is called automatically, as
// necessary, at the requisite publishing period (if a positive publish period
// was passed to the constructor) or per a simulation step (if no publish
// period or publish period = 0.0 was passed to the constructor).
EventStatus LcmPublisherSystem::PublishInputAsLcmMessage(
    const Context<double>& context) const {
  DRAKE_LOGGER_TRACE("Publishing LCM {} message", channel_);
  DRAKE_ASSERT(serializer_ != nullptr);

  // Converts the input into LCM message bytes.
  const AbstractValue& input = get_input_port().Eval<AbstractValue>(context);
  std::vector<uint8_t> message_bytes;
  serializer_->Serialize(input, &message_bytes);

  // Publishes onto the specified LCM channel.
  lcm_->Publish(channel_, message_bytes.data(), message_bytes.size(),
                context.get_time());

  return EventStatus::Succeeded();
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
