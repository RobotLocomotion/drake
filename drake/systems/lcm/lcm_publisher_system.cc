#include "drake/systems/lcm/lcm_publisher_system.h"

#include "drake/systems/framework/system_input.h"

namespace drake {
namespace systems {
namespace lcm {

LcmPublisherSystem::LcmPublisherSystem(const std::string& channel,
                      const LcmAndVectorInterfaceTranslator& translator,
                      ::lcm::LCM* lcm)
    : channel_(channel),
      translator_(translator),
      lcm_(lcm),
      basic_vector_(translator.get_vector_size()) {
}

LcmPublisherSystem::~LcmPublisherSystem() {
}

std::string LcmPublisherSystem::get_name() const {
  return "LcmPublisherSystem::" + channel_;
}

std::unique_ptr<Context<double>> LcmPublisherSystem::CreateDefaultContext()
    const {
  std::unique_ptr<VectorInterface<double>> vector_data(
    new BasicVector<double>(translator_.get_vector_size()));

  std::unique_ptr<InputPort<double>> inputPort(
    new FreestandingInputPort<double>(std::move(vector_data)));

  std::unique_ptr<Context<double>> context(new Context<double>());
  context->SetNumInputPorts(kNumInputPorts);
  context->SetInputPort(kPortIndex, std::move(inputPort));

  return context;
}

std::unique_ptr<SystemOutput<double>> LcmPublisherSystem::AllocateOutput()
    const {
  std::unique_ptr<SystemOutput<double>> output(new SystemOutput<double>);
  return output;
}

void LcmPublisherSystem::EvalOutput(const Context<double>& context,
                SystemOutput<double>* output) const {
  // Obtains the input vector.
  const VectorInterface<double>* input_vector =
      context.get_vector_input(kPortIndex);
  const BasicVector<double>& basic_input_vector =
      dynamic_cast<const BasicVector<double>&>(*input_vector);

  // Translates the input vector into an LCM message. The data is stored in
  // the memory pointed to by variable buffer.
  translator_.TranslateAndSendVectorInterfaceToLCM(basic_input_vector, channel_,
      lcm_);
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
