#include "drake/systems/lcm/lcm_publisher_system.h"

#include <cstdint>
#include <utility>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/fixed_input_port_value.h"

namespace drake {
namespace systems {
namespace lcm {

using drake::lcm::DrakeLcmInterface;
using drake::lcm::DrakeLcm;

namespace {
const int kPortIndex = 0;
}  // namespace

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
    DrakeLcmInterface* lcm)
    : channel_(channel),
      translator_(owned_translator ? owned_translator.get() : translator),
      owned_translator_(std::move(owned_translator)),
      serializer_(std::move(serializer)),
      owned_lcm_(lcm ? nullptr : new DrakeLcm()),
      lcm_(lcm ? lcm : owned_lcm_.get()) {
  DRAKE_DEMAND((translator_ != nullptr) != (serializer_.get() != nullptr));
  DRAKE_DEMAND(lcm_);

  if (translator_ != nullptr) {
    DeclareInputPort("lcm_message", kVectorValued,
                     translator_->get_vector_size());
  } else {
    DeclareAbstractInputPort("lcm_message");
  }

  set_name(make_name(channel_));
}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel, std::unique_ptr<SerializerInterface> serializer,
    drake::lcm::DrakeLcmInterface* lcm)
    : LcmPublisherSystem(channel, nullptr, nullptr, std::move(serializer),
                         lcm) {}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    const LcmAndVectorBaseTranslator& translator,
    drake::lcm::DrakeLcmInterface* lcm)
    : LcmPublisherSystem(channel, &translator, nullptr, nullptr, lcm) {}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    std::unique_ptr<const LcmAndVectorBaseTranslator> translator,
    drake::lcm::DrakeLcmInterface* lcm)
    : LcmPublisherSystem(channel, nullptr, std::move(translator), nullptr,
                         lcm) {}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    const LcmTranslatorDictionary& translator_dictionary,
    DrakeLcmInterface* lcm)
    : LcmPublisherSystem(
          channel,
          translator_dictionary.GetTranslator(channel),
          lcm) {}

LcmPublisherSystem::~LcmPublisherSystem() {}

void LcmPublisherSystem::AddInitializationMessage(
    InitializationPublisher initialization_publisher) {
  DRAKE_DEMAND(!!initialization_publisher);

  initialization_publisher_ = std::move(initialization_publisher);

  DeclareInitializationEvent(systems::PublishEvent<double>(
      systems::Event<double>::TriggerType::kInitialization,
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

void LcmPublisherSystem::set_publish_period(double period) {
  LeafSystem<double>::DeclarePeriodicPublish(period);
}

void LcmPublisherSystem::DoPublish(
    const Context<double>& context,
    const std::vector<const systems::PublishEvent<double>*>& events) const {

  DRAKE_DEMAND(!events.empty());  // Framework guarantees this.
  const auto& event = events.front();

  if (event->get_trigger_type() ==
      systems::Event<double>::TriggerType::kInitialization) {
    // We shouldn't get another event along with our own initialization event.
    DRAKE_DEMAND(events.size() == 1);
    SPDLOG_TRACE(drake::log(), "Invoking initialization publisher");
    event->handle(context);
    return;
  }

  // If the event isn't initialization, we assume it is a request to publish
  // the input port contents as an LCM message. (This is likely to be a periodic
  // event, but could be a forced event or any other type.) If multiple events
  // occur simultaneously (for example, due to occasional synchronization of
  // periods from different periodic events), we still only want to publish the
  // input port values once, so we don't care if there are more events.

  SPDLOG_TRACE(drake::log(), "Publishing LCM {} message", channel_);
  DRAKE_ASSERT((translator_ != nullptr) != (serializer_.get() != nullptr));

  // Converts the input into LCM message bytes.
  std::vector<uint8_t> message_bytes;
  if (translator_ != nullptr) {
    const VectorBase<double>* const input_vector =
        this->EvalVectorInput(context, kPortIndex);
    DRAKE_ASSERT(input_vector != nullptr);
    translator_->Serialize(context.get_time(), *input_vector, &message_bytes);
  } else {
    const AbstractValue* const input_value =
        this->EvalAbstractInput(context, kPortIndex);
    DRAKE_ASSERT(input_value != nullptr);
    serializer_->Serialize(*input_value, &message_bytes);
  }

  // Publishes onto the specified LCM channel.
  lcm_->Publish(channel_, message_bytes.data(), message_bytes.size(),
                context.get_time());
}

const LcmAndVectorBaseTranslator& LcmPublisherSystem::get_translator() const {
  DRAKE_DEMAND(translator_ != nullptr);
  return *translator_;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
