#include "drake/systems/lcm/lcm_publisher_system.h"

#include <cstdint>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/systems/framework/system_input.h"

// Clean up windows junk; see http://stackoverflow.com/questions/4111899/.
#if defined(WIN32) || defined(WIN64)
  #undef GetMessage
#endif

namespace drake {
namespace systems {
namespace lcm {

namespace {
const int kPortIndex = 0;
}  // namespace

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel, const LcmAndVectorBaseTranslator& translator,
    ::lcm::LCM* lcm)
    : channel_(channel), translator_(translator), lcm_(lcm) {
  DeclareInputPort(kVectorValued, translator_.get_vector_size(),
                   kContinuousSampling);
}

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    const LcmTranslatorDictionary& translator_dictionary, ::lcm::LCM* lcm)
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

void LcmPublisherSystem::DoPublish(const Context<double>& context) const {
  SPDLOG_TRACE(drake::log(), "Publishing LCM {} message", channel_);

  // Obtains the input vector.
  const VectorBase<double>* const input_vector =
      this->EvalVectorInput(context, kPortIndex);

  // Translates the input vector into LCM message bytes.
  translator_.Serialize(context.get_time(), *input_vector, &message_bytes_);

  // Publishes onto the specified LCM channel.
  lcm_->publish(channel_, message_bytes_.data(), message_bytes_.size());
}

std::vector<uint8_t> LcmPublisherSystem::GetMessage() const {
  return message_bytes_;
}

void LcmPublisherSystem::GetMessage(BasicVector<double>* message_vector) const {
  // We use GetMessage() here to ensure we stay in sync with its implementation.
  const std::vector<uint8_t> bytes = GetMessage();
  translator_.Deserialize(bytes.data(), bytes.size(), message_vector);
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
