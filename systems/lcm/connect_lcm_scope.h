#pragma once

#include <string>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 Provides the ability to publish any vector-valued output port to the LCM
 @p channel, using the drake::lcmt_drake_signal LCM message type, by adding
 an appropriate LcmPublisherSystem to the @p builder.  If @p lcm is
 null, then an LCM instance will be created automatically (but this is
 expensive, and creating multiple LCM instances in a single process should be
 avoided).

 The intention is to enable logging and debugging in complex diagrams
 using external tools like `lcm-spy`.  Consider using
 Simulator<T>::set_publish_every_time_step(), or calling the
 LcmPublisherSystem::set_publish_period() on the returned LcmPublisherSystem
 to control when the output is generated.

 @pre @p src must be an OutputPort of a system that has already been added to
 the @p builder.

 @ingroup message_passing
 */
LcmPublisherSystem* ConnectLcmScope(const OutputPort<double>& src,
                                    const std::string& channel,
                                    systems::DiagramBuilder<double>* builder,
                                    drake::lcm::DrakeLcmInterface* lcm);

}  // namespace lcm
}  // namespace systems
}  // namespace drake
