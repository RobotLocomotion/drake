#include "drake/systems/lcm/lcm_subscriber_system.h"

#include <iostream>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {
namespace lcm {

using std::make_unique;

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    const LcmAndVectorBaseTranslator& translator, ::lcm::LCM* lcm)
    : channel_(channel),
      translator_(translator),
      basic_vector_(translator_.get_vector_size()) {
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

void LcmSubscriberSystem::EvalOutput(const ContextBase<double>&,
                                     SystemOutput<double>* output) const {
  BasicVector<double>& output_vector =
      dynamic_cast<BasicVector<double>&>(*output->GetMutableVectorData(0));

  std::lock_guard<std::mutex> lock(data_mutex_);
  output_vector.set_value(basic_vector_.get_value());
}

void LcmSubscriberSystem::HandleMessage(const ::lcm::ReceiveBuffer* rbuf,
                                        const std::string& channel) {
  if (channel == channel_) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    translator_.TranslateLcmToVectorBase(
        rbuf->data, rbuf->data_size, &basic_vector_);
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
