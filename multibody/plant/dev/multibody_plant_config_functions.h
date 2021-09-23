#pragma once

#include <string>

#include "drake/multibody/plant/dev/multibody_plant_config.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace anzu {
namespace sim {

/// Adds a new MultibodyPlant and SceneGraph to the given `builder`.  The
/// plant's settings such as `time_step` are set using the given `config`.
drake::multibody::AddMultibodyPlantSceneGraphResult<double>
AddMultibodyPlant(
    const MultibodyPlantConfig& config,
    drake::systems::DiagramBuilder<double>* builder);

namespace internal {

// (Exposed for unit testing only.)
// Parses a string name for a contact model and returns the enumerated value.
// Valid string names are listed in MultibodyPlantConfig's class overview.
// @throws std::exception if an invalid string is passed in.
drake::multibody::ContactModel GetContactModelFromString(
    const std::string_view& contact_model);

// (Exposed for unit testing only.)
// Returns the string name of an enumerated value for a contact model.
std::string ContactModelToString(
    drake::multibody::ContactModel contact_model);

}  // namespace internal
}  // namespace sim
}  // namespace anzu
