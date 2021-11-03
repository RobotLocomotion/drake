#include "drake/systems/lcm/lcm_scope_system.h"

#include <memory>

#include "drake/common/drake_throw.h"
#include "drake/lcmt_scope.hpp"

namespace drake {
namespace systems {
namespace lcm {
namespace {

void Convert(
    const double time,
    const VectorX<double>& input,
    lcmt_scope* output) {
  output->utime = static_cast<int64_t>(time * 1e6);
  output->size = input.size();
  output->value.resize(output->size);
  for (int i = 0; i < output->size; ++i) {
    output->value[i] = input[i];
  }
}

}  // namespace

LcmScopeSystem::LcmScopeSystem(int size) {
  const InputPort<double>& input_port = this->DeclareVectorInputPort(
      "input", size);
  this->DeclareAbstractOutputPort(
      "output",
      []() { return std::make_unique<Value<lcmt_scope>>(lcmt_scope{}); },
      [&input_port](const Context<double>& context, AbstractValue* output) {
        const auto& input = input_port.Eval(context);
        auto& message = output->get_mutable_value<lcmt_scope>();
        Convert(context.get_time(), input, &message);
      });
}

std::tuple<LcmScopeSystem*, LcmPublisherSystem*> LcmScopeSystem::AddToBuilder(
    systems::DiagramBuilder<double>* builder,
    drake::lcm::DrakeLcmInterface* lcm,
    const OutputPort<double>& signal,
    const std::string& channel,
    double publish_period) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(lcm != nullptr);
  auto* scope = builder->AddSystem<LcmScopeSystem>(signal.size());
  builder->Connect(signal, scope->get_input_port());
  auto* publisher = builder->AddSystem(LcmPublisherSystem::Make<lcmt_scope>(
      channel, lcm, publish_period));
  builder->Connect(*scope, *publisher);
  return {scope, publisher};
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
