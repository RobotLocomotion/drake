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

// TODO(jwnimmer-tri) The "serializer xor translator" disjoint implementations
// within the method bodies below are not ideal, because of the code smell, and
// because it is likely confusing for users.  We should take further steps to
// make the Value<LcmMessage> port the primary output port, and find a better
// phrasing for the vector-valued output port for users.  For now though, this
// implementation serves as a transition point where we don't have to rewrite
// the old code yet, but still can supply the AbstractValue port for new code.

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    const LcmAndVectorBaseTranslator* translator,
    std::unique_ptr<SerializerInterface> serializer,
    drake::lcm::DrakeLcmInterface* lcm)
    : channel_(channel),
      translator_(translator),
      serializer_(std::move(serializer)) {
  DRAKE_DEMAND((translator_ != nullptr) != (serializer_.get() != nullptr));
  DRAKE_DEMAND(lcm);

  lcm->Subscribe(channel_, this);
  if (translator_ != nullptr) {
    DeclareOutputPort(kVectorValued, translator_->get_vector_size(),
                      kContinuousSampling);
  } else {
    DeclareAbstractOutputPort(kContinuousSampling);
  }
}

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    std::unique_ptr<SerializerInterface> serializer,
    DrakeLcmInterface* lcm)
    : LcmSubscriberSystem(channel, nullptr, std::move(serializer), lcm) {}

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    const LcmAndVectorBaseTranslator& translator, DrakeLcmInterface* lcm)
    : LcmSubscriberSystem(channel, &translator, nullptr, lcm) {}

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
  DRAKE_ASSERT((translator_ != nullptr) != (serializer_.get() != nullptr));

  if (translator_ != nullptr) {
    VectorBase<double>* const output_vector = output->GetMutableVectorData(0);
    DRAKE_ASSERT(output_vector != nullptr);

    std::lock_guard<std::mutex> lock(received_message_mutex_);
    if (!received_message_.empty()) {
      translator_->Deserialize(
          received_message_.data(), received_message_.size(), output_vector);
    }
  } else {
    AbstractValue* const output_value = output->GetMutableData(0);
    DRAKE_ASSERT(output_value != nullptr);

    std::lock_guard<std::mutex> lock(received_message_mutex_);
    if (!received_message_.empty()) {
      serializer_->Deserialize(
          received_message_.data(), received_message_.size(), output_value);
    }
  }
}

// This is called no matter what output port type we are using.
std::unique_ptr<SystemOutput<double>>
LcmSubscriberSystem::AllocateOutput(const Context<double>& context) const {
  DRAKE_DEMAND((translator_ != nullptr) != (serializer_.get() != nullptr));

  if (translator_ != nullptr) {
    // For vector-valued output, the base class implementation is correct.
    return LeafSystem<double>::AllocateOutput(context);
  } else {
    // For abstract-valued output, we need to roll our own.
    auto output = std::make_unique<LeafSystemOutput<double>>();
    output->get_mutable_ports()->emplace_back(
        std::make_unique<OutputPort>(serializer_->CreateDefaultValue()));
    return std::unique_ptr<SystemOutput<double>>(output.release());
  }
}

// This is only called if our output port is vector-valued.
std::unique_ptr<BasicVector<double>> LcmSubscriberSystem::AllocateOutputVector(
    const SystemPortDescriptor<double>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() == 0);
  DRAKE_DEMAND(translator_ != nullptr);
  DRAKE_DEMAND(serializer_.get() == nullptr);
  auto result = translator_->AllocateOutputVector();
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
  DRAKE_DEMAND(translator_ != nullptr);
  return *translator_;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
