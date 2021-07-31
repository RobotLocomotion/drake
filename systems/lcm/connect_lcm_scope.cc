#include "drake/systems/lcm/connect_lcm_scope.h"

#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace lcm {
namespace {

class TranslatorSystem final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TranslatorSystem)

  explicit TranslatorSystem(int size)
      : size_(size) {
    this->DeclareVectorInputPort("input", size);
    this->DeclareAbstractOutputPort("output", &TranslatorSystem::CalcOutput);
  }

 private:
  void CalcOutput(const Context<double>& context,
                  lcmt_drake_signal* output) const {
    const auto& input = this->get_input_port(0).Eval(context);
    *output = {};
    output->dim = size_;
    output->val.resize(size_);
    output->coord.resize(size_);
    output->timestamp = static_cast<int64_t>(context.get_time() * 1000);
    for (int i = 0; i < size_; ++i) {
      output->val[i] = input[i];
    }
  }

  const int size_;
};

}  // namespace

LcmPublisherSystem* ConnectLcmScope(const OutputPort<double>& src,
                                    const std::string& channel,
                                    systems::DiagramBuilder<double>* builder,
                                    drake::lcm::DrakeLcmInterface* lcm,
                                    double publish_period) {
  DRAKE_DEMAND(builder != nullptr);
  auto translator = builder->AddSystem<TranslatorSystem>(src.size());
  auto publisher = builder->AddSystem(
      LcmPublisherSystem::Make<lcmt_drake_signal>(
          channel, lcm, publish_period));
  builder->Connect(src, translator->get_input_port(0));
  builder->Connect(*translator, *publisher);
  return publisher;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
