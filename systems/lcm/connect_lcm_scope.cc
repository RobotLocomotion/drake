#include "drake/systems/lcm/connect_lcm_scope.h"

#include <vector>

#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"

namespace drake {
namespace systems {
namespace lcm {
namespace {

class TranslatorSystem final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TranslatorSystem)

  explicit TranslatorSystem(int size)
      : helper_(size) {
    this->DeclareVectorInputPort("input", BasicVector<double>(size));
    this->DeclareAbstractOutputPort("output", &TranslatorSystem::CalcOutput);
  }

 private:
  void CalcOutput(const Context<double>& context,
                  lcmt_drake_signal* output) const {
    std::vector<uint8_t> lcm_message_bytes;
    helper_.Serialize(
        context.get_time(),
        this->get_input_port(0).Eval<BasicVector<double>>(context),
        &lcm_message_bytes);
    const int status = output->decode(
        lcm_message_bytes.data(), 0, lcm_message_bytes.size());
    DRAKE_THROW_UNLESS(status >= 0);
  }

  LcmtDrakeSignalTranslator helper_;
};

}  // namespace

LcmPublisherSystem* ConnectLcmScope(const OutputPort<double>& src,
                                    const std::string& channel,
                                    systems::DiagramBuilder<double>* builder,
                                    drake::lcm::DrakeLcmInterface* lcm) {
  DRAKE_DEMAND(builder != nullptr);
  auto translator = builder->AddSystem<TranslatorSystem>(src.size());
  auto publisher = builder->AddSystem(
      LcmPublisherSystem::Make<lcmt_drake_signal>(channel, lcm));
  builder->Connect(src, translator->get_input_port(0));
  builder->Connect(*translator, *publisher);
  return publisher;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
