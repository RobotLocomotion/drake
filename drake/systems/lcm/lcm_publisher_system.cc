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

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel, const LcmAndVectorBaseTranslator& translator,
    DrakeLcmInterface* lcm)
    : channel_(channel), translator_(translator), lcm_(lcm) {
  DeclareInputPort(kVectorValued, translator_.get_vector_size(),
                   kContinuousSampling);
}

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

  // Obtains the input vector.
  const VectorBase<double>* const input_vector =
      this->EvalVectorInput(context, kPortIndex);

  // Translates the input vector into LCM message bytes.
  std::vector<uint8_t> message_bytes;
  translator_.Serialize(context.get_time(), *input_vector, &message_bytes);

  // Publishes onto the specified LCM channel.
  lcm_->Publish(channel_, message_bytes.data(), message_bytes.size());
}

const LcmAndVectorBaseTranslator& LcmPublisherSystem::get_translator() const {
  return translator_;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
