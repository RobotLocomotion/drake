#include "drake/systems/lcm/scope.h"

#include <memory>
#include <utility>

#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"

namespace drake {
namespace systems {
namespace lcm {

LcmPublisherSystem* LcmScopeOutput(const OutputPort<double>& src,
                                   const std::string& channel,
                                   systems::DiagramBuilder<double>* builder,
                                   drake::lcm::DrakeLcmInterface* lcm) {
  auto translator = std::make_unique<LcmtDrakeSignalTranslator>(src.size());
  auto publisher = builder->AddSystem<LcmPublisherSystem>(
      channel, std::move(translator), lcm);
  builder->Connect(src, publisher->get_input_port());

  return publisher;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
