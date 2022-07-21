#pragma once

#include <map>
#include <string>

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_params.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_buses.h"

namespace drake {
namespace systems {
namespace lcm {

/** Given LCM bus names and parameters, adds an LcmInterfaceSystem for each bus
within the given diagram builder, and returns an LcmBuses object that provides
access to the DrakeLcmInterface objects that were created.

Because the interfaces live within the builder (and so eventually, the diagram),
the diagram will pump the interfaces when it's used with a simulator. Refer to
the LcmInterfaceSystem documentation for details.

The interface pointers remain owned by the builder; the LcmBuses object merely
aliases into the builder (and then eventually, the diagram).

@param lcm_buses A map of {bus_name: params} for LCM transceivers, to be used
used by drivers, sensors, etc. */
LcmBuses ApplyLcmBusConfig(
    const std::map<std::string, drake::lcm::DrakeLcmParams>& lcm_buses,
    systems::DiagramBuilder<double>* builder);

}  // namespace lcm
}  // namespace systems
}  // namespace drake
