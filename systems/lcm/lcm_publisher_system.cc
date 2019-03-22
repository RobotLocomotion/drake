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

// TODO(jwnimmer-tri) The "serializer xor translator" disjoint implementations
// within the method bodies below are not ideal, because of the code smell, and
// because it is likely confusing for users.  We should take further steps to
// make the Value<LcmMessage> port the primary input port, and find a better
// phrasing for the vector-valued input port for users.  For now though, this
// implementation serves as a transition point where we don't have to rewrite
// the old code yet, but still can supply the AbstractValue port for new code.

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    const LcmAndVectorBaseTranslator* translator,
    std::unique_ptr<const LcmAndVectorBaseTranslator> owned_translator,
    std::unique_ptr<SerializerInterface> serializer,
    DrakeLcmInterface* lcm,
    const TriggerTypeSet& publish_triggers,
    double publish_period)
    : channel_(channel),
      translator_(owned_translator ? owned_translator.get() : translator),
      owned_translator_(std::move(owned_translator)),
      serializer_(std::move(serializer)),
      owned_lcm_(lcm ? nullptr : new DrakeLcm()),
      lcm_(lcm ? lcm : owned_lcm_.get()) {
  DRAKE_DEMAND((translator_ != nullptr) != (serializer_.get() != nullptr));
  DRAKE_DEMAND(lcm_);
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

  if (translator_ != nullptr) {
    // If the translator provides a specific storage type (i.e., if it returns
    // non-nullptr from AllocateOutputVector), then use its storage to declare
    // our input type.  This is important when the basic vector subtype has
    // intrinsic constraints (e.g., a bounding box on its values), or to enable
    // type-checking for Diagram wiring or FixInputPort values.
    std::unique_ptr<BasicVector<double>> model_vector =
        translator_->AllocateOutputVector();
    if (!model_vector) {
      model_vector = std::make_unique<BasicVector<double>>(
          translator_->get_vector_size());
    }
    DeclareVectorInputPort("lcm_message", *model_vector);
  } else {
    DeclareAbstractInputPort("lcm_message", *serializer_->CreateDefaultValue());
  }

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
      // TODO(edrumwri) Remove this code once set_publish_period(.) has
      // been removed; it exists so that one does not get both a per-step
      // publish and a periodic publish if a user constructs the publisher
      // the "old" way (construction followed by set_publish_period()).
      if (this->disable_internal_per_step_publish_events_)
        return;

      this->PublishInputAsLcmMessage(context);
    }));
  }
}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    const LcmAndVectorBaseTranslator* translator,
    std::unique_ptr<const LcmAndVectorBaseTranslator> owned_translator,
    std::unique_ptr<SerializerInterface> serializer,
    DrakeLcmInterface* lcm, double publish_period)
    : LcmPublisherSystem(channel, translator, std::move(owned_translator),
      std::move(serializer), lcm,
      (publish_period > 0.0) ?
      TriggerTypeSet({TriggerType::kForced, TriggerType::kPeriodic}) :
      TriggerTypeSet({TriggerType::kForced, TriggerType::kPerStep}),
      publish_period) {}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel, std::unique_ptr<SerializerInterface> serializer,
    drake::lcm::DrakeLcmInterface* lcm,
    double publish_period)
    : LcmPublisherSystem(channel, nullptr, nullptr, std::move(serializer),
                         lcm, publish_period) {}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel, std::unique_ptr<SerializerInterface> serializer,
    drake::lcm::DrakeLcmInterface* lcm,
    const TriggerTypeSet& publish_triggers,
    double publish_period)
    : LcmPublisherSystem(channel, nullptr, nullptr, std::move(serializer),
                         lcm, publish_triggers, publish_period) {}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    const LcmAndVectorBaseTranslator& translator,
    drake::lcm::DrakeLcmInterface* lcm,
    double publish_period)
    : LcmPublisherSystem(channel, &translator, nullptr, nullptr,
                         lcm, publish_period) {}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    std::unique_ptr<const LcmAndVectorBaseTranslator> translator,
    drake::lcm::DrakeLcmInterface* lcm,
    double publish_period)
    : LcmPublisherSystem(channel, nullptr, std::move(translator), nullptr,
                         lcm, publish_period) {}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    const LcmTranslatorDictionary& translator_dictionary,
    DrakeLcmInterface* lcm,
    double publish_period)
    : LcmPublisherSystem(
          channel,
          translator_dictionary.GetTranslator(channel),
          lcm, publish_period) {}
#pragma GCC diagnostic pop

LcmPublisherSystem::~LcmPublisherSystem() {}

void LcmPublisherSystem::AddInitializationMessage(
    InitializationPublisher initialization_publisher) {
  DRAKE_DEMAND(!!initialization_publisher);

  initialization_publisher_ = std::move(initialization_publisher);

  DeclareInitializationEvent(systems::PublishEvent<double>(
      systems::TriggerType::kInitialization,
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

// Takes the VectorBase from the input port of the context and publishes
// it onto an LCM channel. This function is called automatically, as
// necessary, at the requisite publishing period (if a positive publish period
// was passed to the constructor) or per a simulation step (if no publish
// period or publish period = 0.0 was passed to the constructor).
EventStatus LcmPublisherSystem::PublishInputAsLcmMessage(
    const Context<double>& context) const {
  SPDLOG_TRACE(drake::log(), "Publishing LCM {} message", channel_);
  DRAKE_ASSERT((translator_ != nullptr) != (serializer_.get() != nullptr));

  // Converts the input into LCM message bytes.
  std::vector<uint8_t> message_bytes;
  if (translator_ != nullptr) {
    const auto& input = get_input_port().Eval<BasicVector<double>>(context);
    translator_->Serialize(context.get_time(), input, &message_bytes);
  } else {
    const AbstractValue& input = get_input_port().Eval<AbstractValue>(context);
    serializer_->Serialize(input, &message_bytes);
  }

  // Publishes onto the specified LCM channel.
  lcm_->Publish(channel_, message_bytes.data(), message_bytes.size(),
                context.get_time());

  return EventStatus::Succeeded();
}

const LcmAndVectorBaseTranslator& LcmPublisherSystem::get_translator() const {
  DRAKE_DEMAND(translator_ != nullptr);
  return *translator_;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
