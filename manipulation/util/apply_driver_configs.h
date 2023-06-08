#pragma once

#include <map>
#include <string>
#include <variant>
#include <vector>

#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_buses.h"

namespace drake {
namespace manipulation {

// TODO(ggould) Eliminate our reliance on models_from_directives below, once we
// have a better alternative to `MakeArmControllerModel`.

// TODO(jeremy.nimmer) This function is not unit tested; it's just acceptance-
// tested via drake/examples/hardware_sim. Consider whether there is any direct
// unit test that would add value.

/// Apply many driver configurations to a model.
///
/// A "driver configuration" helps stack Drake systems between an LCM interface
/// subscriber system and the actuation input ports of a MultibodyPlant (to
/// enact the driver command), as well as the output ports of a MultibodyPlant
/// back to an LCM interface publisher system (to provide the driver status).
///
/// These conceptually simulate "drivers" -- they take the role that driver
/// software and control cabinets would take in real life -- but may also
/// model some physical properties of the robot that are not easily reflected
/// in MultibodyPlant (e.g., the WSG belt drive).
///
/// The caller of this function is responsible for including the variant
/// members' apply function, such as schunk_wsg_driver_functions.h.
///
/// @p driver_configs The configurations to apply.
/// @p sim_plant The plant containing the model.
/// @p models_from_directives All of the `ModelInstanceInfo`s from previously-
///    loaded directives.
/// @p lcm_buses The available LCM buses to drive and/or sense from this driver.
/// @p builder The `DiagramBuilder` into which to install this driver.
template <typename... Types>
void ApplyDriverConfigs(
    const std::map<std::string, std::variant<Types...>>& driver_configs,
    const multibody::MultibodyPlant<double>& sim_plant,
    const std::vector<multibody::parsing::ModelInstanceInfo>&
        models_from_directives,
    const systems::lcm::LcmBuses& lcm_buses,
    systems::DiagramBuilder<double>* builder) {
  std::map<std::string, multibody::parsing::ModelInstanceInfo>
      models_from_directives_map;
  for (const auto& info : models_from_directives) {
    models_from_directives_map.emplace(info.model_name, info);
  }
  for (const auto& config_pair : driver_configs) {
    // N.B. We can't use a structured binding here due to a Clang bug.
    const std::string& model_instance_name = config_pair.first;
    const auto& driver_config = config_pair.second;
    std::visit(
        [&](const auto& driver) {
          ApplyDriverConfig(driver, model_instance_name, sim_plant,
                            models_from_directives_map, lcm_buses, builder);
        },
        driver_config);
  }
}

}  // namespace manipulation
}  // namespace drake
