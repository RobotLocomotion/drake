#include "drake/systems/lcm/lcm_publisher_system.h"

#include "drake/systems/framework/system_input.h"

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
  return "LcmPublisherSystem::" + channel_;
}

std::unique_ptr<ContextBase<double>> LcmPublisherSystem::CreateDefaultContext()
    const {
  std::unique_ptr<Context<double>> context(new Context<double>());
  context->SetNumInputPorts(get_num_input_ports());
  return std::unique_ptr<ContextBase<double>>(context.release());
}

std::unique_ptr<SystemOutput<double>> LcmPublisherSystem::AllocateOutput(
    const ContextBase<double>& context) const {
  std::unique_ptr<SystemOutput<double>> output(new LeafSystemOutput<double>);
  return output;
}

void LcmPublisherSystem::DoPublish(const ContextBase<double>& context) const {
  // Obtains the input vector.
  const VectorBase<double>* input_vector =
      context.get_vector_input(kPortIndex);

  // Translates the input vector into an LCM message and publishes it onto the
  // specified LCM channel.
  translator_.PublishVectorBaseToLCM(*input_vector, channel_, lcm_);
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
