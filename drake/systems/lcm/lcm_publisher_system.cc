#include "drake/systems/lcm/lcm_publisher_system.h"

#include <cstdint>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/system_input.h"

namespace drake {
namespace systems {
namespace lcm {

using drake::lcm::DrakeLcmInterface;

namespace {
const int kPortIndex = 0;
}  // namespace

// TODO(jwnimmer-tri) The "serializer xor translator" disjoint implementations
// within the method bodies below are not ideal, because of the code smell, and
// because it is likely confusing for users.  We should take further steps to
// make the Value<LcmMessage> port the primary output port, and find a better
// phrasing for the vector-valued output port for users.  For now though, this
// implementation serves as a transition point where we don't have to rewrite
// the old code yet, but still can supply the AbstractValue port for new code.

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    const LcmAndVectorBaseTranslator* translator,
    std::unique_ptr<SerializerInterface> serializer,
    DrakeLcmInterface* lcm)
    : channel_(channel),
      translator_(translator),
      serializer_(std::move(serializer)),
      lcm_(lcm) {
  DRAKE_DEMAND((translator_ != nullptr) != (serializer_.get() != nullptr));
  DRAKE_DEMAND(lcm_);

  if (translator_ != nullptr) {
    DeclareInputPort(kVectorValued, translator_->get_vector_size(),
                     kContinuousSampling);
  } else {
    DeclareAbstractInputPort(kContinuousSampling);
  }
}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    std::unique_ptr<SerializerInterface> serializer,
    drake::lcm::DrakeLcmInterface* lcm)
    : LcmPublisherSystem(channel, nullptr, std::move(serializer), lcm) {}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    const LcmAndVectorBaseTranslator& translator,
    drake::lcm::DrakeLcmInterface* lcm)
    : LcmPublisherSystem(channel, &translator, nullptr, lcm) {}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    const LcmTranslatorDictionary& translator_dictionary,
    DrakeLcmInterface* lcm)
    : LcmPublisherSystem(
          channel,
          translator_dictionary.GetTranslator(channel),
          lcm) {}

LcmPublisherSystem::~LcmPublisherSystem() {}

std::string LcmPublisherSystem::get_name() const {
  return get_name(channel_);
}

std::string LcmPublisherSystem::get_name(const std::string& channel) {
  return "LcmPublisherSystem(" + channel + ")";
}

const std::string& LcmPublisherSystem::get_channel_name() const {
  return channel_;
}

void LcmPublisherSystem::DoPublish(const Context<double>& context) const {
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
  lcm_->Publish(channel_, message_bytes.data(), message_bytes.size());
}

const LcmAndVectorBaseTranslator& LcmPublisherSystem::get_translator() const {
  DRAKE_DEMAND(translator_ != nullptr);
  return *translator_;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
