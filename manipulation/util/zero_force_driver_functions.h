#pragma once

#include <map>
#include <string>

#include "drake/manipulation/util/zero_force_driver.h"
#include "drake/multibody/parsing/model_instance_info.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_buses.h"

namespace drake {
namespace manipulation {

/** Applies zero actuation to every joint of a model mainly for debugging and
testing purposes.
@pre The sim_plant.is_finalized() is true. */
void ApplyDriverConfig(
    const ZeroForceDriver& driver_config,
    const std::string& model_instance_name,
    const multibody::MultibodyPlant<double>& sim_plant,
    const std::map<std::string, multibody::parsing::ModelInstanceInfo>&
        models_from_directives,
    const systems::lcm::LcmBuses& lcms,
    systems::DiagramBuilder<double>* builder);

}  // namespace manipulation
}  // namespace drake
