#pragma once

#include <map>
#include <string>

#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_buses.h"
#include "sim/common/wsg_driver.h"

namespace anzu {
namespace sim {

/** Wires up Drake systems between an LCM interface and the actuation input
ports of a MultibodyPlant. This simulates the role that driver software and
robot firmware would take in real life. */
void ApplyDriverConfig(
    const WsgDriver& driver_config,
    const std::string& model_instance_name,
    const drake::multibody::MultibodyPlant<double>& sim_plant,
    const std::map<std::string, drake::multibody::parsing::ModelInstanceInfo>&
        models_from_directives,
    const drake::systems::lcm::LcmBuses& lcms,
    drake::systems::DiagramBuilder<double>* builder);

}  // namespace sim
}  // namespace anzu
