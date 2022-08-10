#pragma once

#include <map>
#include <string>

#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_buses.h"
#include "sim/common/zero_force_driver.h"

namespace anzu {
namespace sim {

/** Applies zero actuation to every joint of a model mainly for debugging and
testing purposes. */
void ApplyDriverConfig(
    const ZeroForceDriver& driver_config,
    const std::string& model_instance_name,
    const drake::multibody::MultibodyPlant<double>& sim_plant,
    const std::map<std::string, drake::multibody::parsing::ModelInstanceInfo>&
        models_from_directives,
    const drake::systems::lcm::LcmBuses& /* lcms */,
    drake::systems::DiagramBuilder<double>* builder);

}  // namespace sim
}  // namespace anzu
