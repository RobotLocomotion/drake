#pragma once

#include <string>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace systems {
namespace lcm {

// TODO(jwnimmer-tri) Add DRAKE_DEPRECATED onto here as of 2021-01-01 or so,
// with the removal date the usual +3 months after the marker gets added.
/**
 (To be deprecated.) Prefer to use LcmScopeSystem::AddToBuilder instead of
 this function; the LcmScopeSystem provides more detailed timestamps.

 Provides the ability to publish any vector-valued output port to the LCM
 @p channel, using the drake::lcmt_drake_signal LCM message type, by adding
 an appropriate LcmPublisherSystem to the @p builder.  If @p lcm is
 null, then an LCM instance will be created automatically (but this is
 expensive, and creating multiple LCM instances in a single process should be
 avoided).

 The intention is to enable logging and debugging in complex diagrams
 using external tools like `lcm-spy`.

 The optional @p publish_period specifies how often messages will be published.
 If the period is zero (the default), then the underlying LcmPublisherSystem
 will publish every step.

 @pre @p src must be an OutputPort of a system that has already been added to
 the @p builder.

 @ingroup message_passing
 */
LcmPublisherSystem* ConnectLcmScope(const OutputPort<double>& src,
                                    const std::string& channel,
                                    systems::DiagramBuilder<double>* builder,
                                    drake::lcm::DrakeLcmInterface* lcm,
                                    double publish_period = 0.0);

}  // namespace lcm
}  // namespace systems
}  // namespace drake
