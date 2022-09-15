#pragma once

#include <map>
#include <string>

#include "drake/manipulation/kuka_iiwa/iiwa_driver.h"
#include "drake/multibody/parsing/model_instance_info.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_buses.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

/** Wires up Drake systems between an LCM interface and the actuation input
ports of a MultibodyPlant. This simulates the role that driver software and
control cabinets would take in real life.

@pre model_instance_name is in models_from_directives.
@pre driver_config.hand_model_name is in models_from_directives. */
void ApplyDriverConfig(
    const IiwaDriver& driver_config, const std::string& model_instance_name,
    const multibody::MultibodyPlant<double>& sim_plant,
    const std::map<std::string, multibody::parsing::ModelInstanceInfo>&
        models_from_directives,
    const systems::lcm::LcmBuses& lcms,
    systems::DiagramBuilder<double>* builder);

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
