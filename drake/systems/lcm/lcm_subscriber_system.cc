#include "drake/systems/lcm/lcm_subscriber_system.h"

#include <iostream>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {
namespace lcm {

using drake::lcm::DrakeLcmInterface;

using std::make_unique;

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    const LcmAndVectorBaseTranslator& translator, DrakeLcmInterface* lcm)
    : channel_(channel),
      translator_(translator) {
  DRAKE_DEMAND(lcm);
  lcm->Subscribe(channel_, this);
  DeclareOutputPort(kVectorValued, translator_.get_vector_size(),
      kContinuousSampling);
}

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    const LcmTranslatorDictionary& translator_dictionary,
    DrakeLcmInterface* lcm)
    : LcmSubscriberSystem(channel, translator_dictionary.GetTranslator(channel),
                          lcm) {}

LcmSubscriberSystem::~LcmSubscriberSystem() {}

std::string LcmSubscriberSystem::get_name() const {
  return get_name(channel_);
}

std::string LcmSubscriberSystem::get_name(const std::string& channel) {
  return "LcmSubscriberSystem(" + channel + ")";
}

const std::string& LcmSubscriberSystem::get_channel_name() const {
  return channel_;
}

void LcmSubscriberSystem::EvalOutput(const Context<double>&,
                                     SystemOutput<double>* output) const {
  VectorBase<double>* const output_vector = output->GetMutableVectorData(0);
  std::lock_guard<std::mutex> lock(received_message_mutex_);
  if (!received_message_.empty()) {
    translator_.Deserialize(
        received_message_.data(), received_message_.size(), output_vector);
  }
}

std::unique_ptr<BasicVector<double>> LcmSubscriberSystem::AllocateOutputVector(
    const SystemPortDescriptor<double>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() == 0);
  auto result = translator_.AllocateOutputVector();
  if (result) {
    return result;
  }
  return LeafSystem<double>::AllocateOutputVector(descriptor);
}

void LcmSubscriberSystem::HandleMessage(const std::string& channel,
    const void* message_buffer, int message_size) {
  SPDLOG_TRACE(drake::log(), "Receiving LCM {} message", channel);

  if (channel == channel_) {
    const uint8_t* const rbuf_begin =
        reinterpret_cast<const uint8_t*>(message_buffer);
    const uint8_t* const rbuf_end = rbuf_begin + message_size;
    std::lock_guard<std::mutex> lock(received_message_mutex_);
    received_message_.clear();
    received_message_.insert(received_message_.begin(), rbuf_begin, rbuf_end);
  } else {
    std::cerr << "LcmSubscriberSystem: HandleMessage: WARNING: Received a "
              << "message for channel \"" << channel
              << "\" instead of channel \"" << channel_ << "\". Ignoring it."
              << std::endl;
  }
}

const LcmAndVectorBaseTranslator& LcmSubscriberSystem::get_translator() const {
  return translator_;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
