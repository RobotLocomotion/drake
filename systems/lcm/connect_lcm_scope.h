#pragma once

#include <string>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace systems {
namespace lcm {

DRAKE_DEPRECATED("2021-11-01",
    "Prefer to use LcmScopeSystem::AddToBuilder instead of this function;"
    " the LcmScopeSystem provides more detailed timestamps.")
LcmPublisherSystem* ConnectLcmScope(const OutputPort<double>& src,
                                    const std::string& channel,
                                    systems::DiagramBuilder<double>* builder,
                                    drake::lcm::DrakeLcmInterface* lcm,
                                    double publish_period = 0.0);

}  // namespace lcm
}  // namespace systems
}  // namespace drake
