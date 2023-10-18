#include "drake/systems/lcm/lcm_publisher_system.h"

#include <cstdint>
#include <utility>
#include <vector>

#include "drake/common/nice_type_name.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/lcm/lcm_system_graphviz.h"

namespace drake {
namespace systems {
namespace lcm {

using drake::lcm::DrakeLcm;
using drake::lcm::DrakeLcmInterface;
using systems::TriggerTypeSet;

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    std::shared_ptr<const SerializerInterface> serializer,
    DrakeLcmInterface* lcm, const TriggerTypeSet& publish_triggers,
    double publish_period, double publish_offset)
    : channel_(channel),
      serializer_(std::move(serializer)),
      owned_lcm_(lcm ? nullptr : new DrakeLcm()),
      lcm_(lcm ? lcm : owned_lcm_.get()),
      publish_period_(publish_period),
      publish_offset_(publish_offset) {
  DRAKE_THROW_UNLESS(serializer_ != nullptr);
  DRAKE_THROW_UNLESS(lcm_ != nullptr);
  DRAKE_THROW_UNLESS(publish_period >= 0.0);
  DRAKE_THROW_UNLESS(std::isfinite(publish_offset));
  DRAKE_THROW_UNLESS(publish_offset >= 0.0);
  DRAKE_THROW_UNLESS(!publish_triggers.empty());

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
    this->DeclarePeriodicPublishEvent(
        publish_period, publish_offset,
        &LcmPublisherSystem::PublishInputAsLcmMessage);
  } else {
    // publish_period > 0 without TriggerType::kPeriodic has no meaning and is
    // likely a mistake.
    DRAKE_THROW_UNLESS(publish_period == 0.0);
    DRAKE_THROW_UNLESS(publish_offset == 0.0);
  }

  if (publish_triggers.find(TriggerType::kPerStep) != publish_triggers.end()) {
    this->DeclarePerStepPublishEvent(
        &LcmPublisherSystem::PublishInputAsLcmMessage);
  }
}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    std::shared_ptr<const SerializerInterface> serializer,
    DrakeLcmInterface* lcm, double publish_period, double publish_offset)
    : LcmPublisherSystem(
          channel, std::move(serializer), lcm,
          (publish_period > 0.0)
              ? TriggerTypeSet({TriggerType::kForced, TriggerType::kPeriodic})
              : TriggerTypeSet({TriggerType::kForced, TriggerType::kPerStep}),
          publish_period, publish_offset) {}

LcmPublisherSystem::~LcmPublisherSystem() {}

void LcmPublisherSystem::AddInitializationMessage(
    InitializationPublisher initialization_publisher) {
  DRAKE_THROW_UNLESS(initialization_publisher != nullptr);
  initialization_publisher_ = std::move(initialization_publisher);
  DeclareInitializationPublishEvent(&LcmPublisherSystem::Initialize);
}

// Note that this function will *only* be called by the framework after
// AddInitializationMessage has called DeclareInitializationPublishEvent
// to register this member function.
EventStatus LcmPublisherSystem::Initialize(
    const Context<double>& context) const {
  // AddInitializationMessage already checked this for nullness.
  DRAKE_DEMAND(initialization_publisher_ != nullptr);
  initialization_publisher_(context, this->lcm_);
  return EventStatus::Succeeded();
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

double LcmPublisherSystem::get_publish_offset() const {
  return publish_offset_;
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

LeafSystem<double>::GraphvizFragment LcmPublisherSystem::DoGetGraphvizFragment(
    const GraphvizFragmentParams& params) const {
  internal::LcmSystemGraphviz lcm_system_graphviz(
      *lcm_, channel_, &serializer_->CreateDefaultValue()->static_type_info(),
      /* publish = */ true,
      /* subscribe = */ false);
  return lcm_system_graphviz.DecorateResult(
      LeafSystem<double>::DoGetGraphvizFragment(
          lcm_system_graphviz.DecorateParams(params)));
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
