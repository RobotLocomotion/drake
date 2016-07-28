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
    const LcmAndVectorInterfaceTranslator& translator,
    ::lcm::LCM* lcm)
    : channel_(channel),
      translator_(translator),
      basic_vector_(translator_.get_vector_size()) {
  DRAKE_ABORT_UNLESS(lcm);
  // Initializes the communication layer.
  ::lcm::Subscription* sub = lcm->subscribe(channel_,
      &LcmSubscriberSystem::HandleMessage, this);
  sub->setQueueCapacity(1);
}

LcmSubscriberSystem::LcmSubscriberSystem(
    const std::string& channel,
    const LcmTranslatorDictionary& translator_dictionary,
    ::lcm::LCM* lcm)
    : LcmSubscriberSystem(
          channel,
          translator_dictionary.GetTranslator(channel),
          lcm) {
}

LcmSubscriberSystem::~LcmSubscriberSystem() {}

std::string LcmSubscriberSystem::get_name() const {
  return "LcmSubscriberSystem::" + channel_;
}

std::unique_ptr<ContextBase<double>> LcmSubscriberSystem::CreateDefaultContext()
    const {
  // Creates a new context for this system and sets the number of input ports
  // to be zero. It leaves the context's state uninitialized since this system
  // does not use it.
  std::unique_ptr<Context<double>> context(new Context<double>());
  context->SetNumInputPorts(0);

  // Returns this system's context.
  return std::unique_ptr<ContextBase<double>>(context.release());
}

std::unique_ptr<SystemOutput<double>> LcmSubscriberSystem::AllocateOutput(
    const ContextBase<double>& context) const {
  // Instantiates a BasicVector object and stores it in a managed pointer.
  std::unique_ptr<BasicVector<double>> data(
      new BasicVector<double>(translator_.get_vector_size()));

  // Instantiates an OutputPort with the above BasicVector as the data type.
  std::unique_ptr<OutputPort<double>> port(
      new OutputPort<double>(std::move(data)));

  // Stores the above-defined OutputPort in this system output.
  auto output = std::make_unique<LeafSystemOutput<double>>();
  output->get_mutable_ports()->push_back(std::move(port));

  // Returns this system's output.
  return std::unique_ptr<SystemOutput<double>>(output.release());
}

void LcmSubscriberSystem::EvalOutput(const ContextBase<double>& context,
                                     SystemOutput<double>* output) const {
  BasicVector<double>& output_vector = dynamic_cast<BasicVector<double>&>(
      *output->get_mutable_port(0)->GetMutableVectorData());

  std::lock_guard<std::mutex> lock(data_mutex_);
  output_vector.set_value(basic_vector_.get_value());
}

void LcmSubscriberSystem::HandleMessage(const ::lcm::ReceiveBuffer* rbuf,
                                        const std::string& channel) {
  if (channel == channel_) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    translator_.TranslateLcmToVectorInterface(rbuf, &basic_vector_);
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
