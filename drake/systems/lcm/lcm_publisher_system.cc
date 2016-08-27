#include "drake/systems/lcm/lcm_publisher_system.h"

#include <cstdint>
#include <vector>

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
    const LcmAndVectorBaseTranslator& translator, ::lcm::LCM* lcm)
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
  const VectorBase<double>* const input_vector =
      context.get_vector_input(kPortIndex);

  // Translates the input vector into LCM message bytes.
  std::vector<uint8_t> lcm_message;
  translator_.TranslateVectorBaseToLcm(*input_vector, &lcm_message);

  // Publishes onto the specified LCM channel.
  lcm_->publish(channel_, lcm_message.data(), lcm_message.size());
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
