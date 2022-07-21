#pragma once

#include <map>
#include <string>

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_params.h"
#include "drake/systems/framework/diagram_builder.h"
#include "sim/common/lcm_buses.h"

namespace anzu {
namespace sim {
// TODO(zachfang): move this file (and the cc file) to `drake/systems/lcm`.

/** Given a list of LCM buses, adds an LcmInterfaceSystem for each bus within
the given diagram builder, and returns a {bus_name: interface} map.

Because the interface lives within the builder (and so eventually, the diagram),
the diagram will pump the interface when it's used with a simulator.  Refer to
the LcmInterfaceSystem documentation for details.

The returned pointers (i.e., the values of the map) remain owned by the builder;
the returned value merely aliases into the builder (eventually, the diagram).

@param lcm_buses A map of {bus_name: lcm_params} for LCM transceivers to be
used by drivers, sensors, etc. */
LcmBuses ApplyLcmBusConfig(
    const std::map<std::string, drake::lcm::DrakeLcmParams>& lcm_buses,
    drake::systems::DiagramBuilder<double>* builder);

}  // namespace sim
}  // namespace anzu
