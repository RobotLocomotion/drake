#pragma once

#include <string>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Provides the ability to publish any VectorBase<double> output port to the LCM
 * @p channel, using the drake::lcmt_drake_signal LCM message type, by adding
 * an appropriate LCMPublisherSystem to the DiagramBuilder.  If @p lcm is
 * null, then an LCM instance will be created automatically (but this is
 * expensive, and creating multiple LCM instances in a single process should be
 * avoided).
 *
 * The intention is to enable logging and debugging in complex diagrams
 * using external tools like `lcm-spy`.
 *
 * @ingroup message_passing
 */
LcmPublisherSystem* LcmScopeOutput(const OutputPort<double>& src,
                                   const std::string& channel,
                                   systems::DiagramBuilder<double>* builder,
                                   drake::lcm::DrakeLcmInterface* lcm);

}  // namespace lcm
}  // namespace systems
}  // namespace drake
