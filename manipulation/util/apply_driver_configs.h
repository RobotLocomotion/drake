#pragma once

#include <map>
#include <string>
#include <variant>

#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_buses.h"

/// @file This file tracks methods we can use to stack Drake systems between
/// an LCM interface and the unactuated ports of an MbP.
///
/// These conceptually simulate "drivers" -- they take the role that driver
/// software and control cabinets would take in real life -- but may also
/// model some physical properties of the robot that are not easily reflected
/// in MbP (the WSG belt drive, eg).

namespace anzu {
namespace sim {

// TODO(ggould) Eliminate our reliance on models_from_directives below, once we
// have a better alternative to `MakeArmControllerModel`.

// TODO(jeremy.nimmer) This function needs a unit test.
/// Apply many driver configurations to a model.
///
/// @p driver_configs The configurations to apply.
/// @p sim_plant The plant containing the model.
/// @p models_from_directives All of the `ModelInstanceInfo`s from previously-
///    loaded directives, keyed on model_name.
/// @p lcm_buses The available LCM buses to drive and/or sense from this driver.
/// @p builder The `DiagramBuilder` into which to install this driver.
template <typename... Types>
void ApplyDriverConfigs(
    const std::map<std::string, std::variant<Types...>>& driver_configs,
    const drake::multibody::MultibodyPlant<double>& sim_plant,
    const std::map<std::string, drake::multibody::parsing::ModelInstanceInfo>&
        models_from_directives,
    const drake::systems::lcm::LcmBuses& lcm_buses,
    drake::systems::DiagramBuilder<double>* builder) {
  for (const auto& [model_instance_name, driver_config] : driver_configs) {
    std::visit([&](const auto& driver) {
      ApplyDriverConfig(
          driver, model_instance_name, sim_plant, models_from_directives,
          lcm_buses, builder);
    }, driver_config);
  }
}

}  // namespace sim
}  // namespace anzu
