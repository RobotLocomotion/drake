#include "drake/systems/lcm/lcm_publisher_system.h"

#include "drake/systems/framework/system_input.h"

namespace drake {
namespace systems {
namespace lcm {

namespace {
const int kNumInputPorts = 1;
const int kPortIndex = 0;
}  // namespace

LcmPublisherSystem::LcmPublisherSystem(
    const std::string& channel,
    const LcmAndVectorInterfaceTranslator& translator, ::lcm::LCM* lcm)
    : channel_(channel), translator_(translator), lcm_(lcm) {}

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
  context->SetNumInputPorts(kNumInputPorts);
  return std::unique_ptr<ContextBase<double>>(context.release());
}

std::unique_ptr<SystemOutput<double>> LcmPublisherSystem::AllocateOutput(
    const ContextBase<double>& context) const {
  std::unique_ptr<SystemOutput<double>> output(new LeafSystemOutput<double>);
  return output;
}

// TODO(liang.fok) Move the LCM message publishing logic into another method
// that's more appropriate once it is defined by System. See:
// https://github.com/RobotLocomotion/drake/issues/2836.
void LcmPublisherSystem::EvalOutput(const ContextBase<double>& context,
                                    SystemOutput<double>* output) const {
  // Obtains the input vector.
  const VectorInterface<double>* input_vector =
      context.get_vector_input(kPortIndex);

  // Translates the input vector into an LCM message and publishes it onto the
  // specified LCM channel.
  translator_.PublishVectorInterfaceToLCM(*input_vector, channel_, lcm_);
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
