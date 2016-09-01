#include "drake/systems/lcm/lcm_subscriber_system.h"

#include <iostream>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {
namespace lcm {

using std::make_unique;

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    const LcmAndVectorBaseTranslator& translator, ::lcm::LCM* lcm)
    : channel_(channel),
      translator_(translator) {
  DRAKE_ABORT_UNLESS(lcm);
  // Initializes the communication layer.
  ::lcm::Subscription* sub =
      lcm->subscribe(channel_, &LcmSubscriberSystem::HandleMessage, this);
  sub->setQueueCapacity(1);
  DeclareOutputPort(kVectorValued, translator_.get_vector_size(),
      kContinuousSampling);
}

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    const LcmTranslatorDictionary& translator_dictionary, ::lcm::LCM* lcm)
    : LcmSubscriberSystem(channel, translator_dictionary.GetTranslator(channel),
                          lcm) {}

LcmSubscriberSystem::~LcmSubscriberSystem() {}

std::string LcmSubscriberSystem::get_name() const {
  return "LcmSubscriberSystem::" + channel_;
}

void LcmSubscriberSystem::EvalOutput(const Context<double>&,
                                     SystemOutput<double>* output) const {
  VectorBase<double>* const output_vector = output->GetMutableVectorData(0);
  std::lock_guard<std::mutex> lock(received_message_mutex_);
  if (!received_message_.empty()) {
    translator_.TranslateLcmToVectorBase(
        received_message_.data(), received_message_.size(), output_vector);
  }
}

void LcmSubscriberSystem::SetMessage(std::vector<uint8_t> message_bytes) {
  std::lock_guard<std::mutex> lock(received_message_mutex_);
  received_message_ = message_bytes;
}

std::unique_ptr<BasicVector<double>> LcmSubscriberSystem::AllocateOutputVector(
    const SystemPortDescriptor<double>& descriptor) const {
  DRAKE_ABORT_UNLESS(descriptor.get_index() == 0);
  auto result = translator_.AllocateOutputVector();
  if (result) {
    return result;
  }
  return LeafSystem<double>::AllocateOutputVector(descriptor);
}

void LcmSubscriberSystem::HandleMessage(const ::lcm::ReceiveBuffer* rbuf,
                                        const std::string& channel) {
  SPDLOG_TRACE(drake::log(), "Receiving LCM {} message", channel);

  if (channel == channel_) {
    const uint8_t* const rbuf_begin =
        reinterpret_cast<const uint8_t*>(rbuf->data);
    const uint8_t* const rbuf_end = rbuf_begin + rbuf->data_size;
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

}  // namespace lcm
}  // namespace systems
}  // namespace drake
