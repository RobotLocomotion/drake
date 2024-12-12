#pragma once

#include <string>

#include "drake/geometry/query_results/contact_surface.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {

/// Adds a new MultibodyPlant and SceneGraph to the given `builder`.  The
/// plant's settings such as `time_step` are set using the given `config`.
AddMultibodyPlantSceneGraphResult<double> AddMultibodyPlant(
    const MultibodyPlantConfig& config,
    systems::DiagramBuilder<double>* builder);

/// Adds a new MultibodyPlant and SceneGraph to the given `builder`.  The
/// plant's settings such as `time_step` are set using the given
/// `plant_config`. The scene graph's settings are set using the given
/// `scene_graph_config`.
AddMultibodyPlantSceneGraphResult<double> AddMultibodyPlant(
    const MultibodyPlantConfig& plant_config,
    const geometry::SceneGraphConfig& scene_graph_config,
    systems::DiagramBuilder<double>* builder);

/// Applies settings given in `config` to an existing `plant`. The `time_step`
/// is the one value in `config` that cannot be updated -- it can only be set
/// in the MultibodyPlant constructor. Consider using AddMultibodyPlant() or
/// manually passing `config.time_step` when you construct the MultibodyPlant.
///
/// This method must be called pre-Finalize.
/// @throws std::exception if `plant` is finalized or if time_step is changed.
void ApplyMultibodyPlantConfig(const MultibodyPlantConfig& config,
                               MultibodyPlant<double>* plant);

namespace internal {

// (Exposed for unit testing only.)
// Parses a string name for a contact model and returns the enumerated value.
// Valid string names are listed in MultibodyPlantConfig's class overview.
// @throws std::exception if an invalid string is passed in.
ContactModel GetContactModelFromString(std::string_view contact_model);

// (Exposed for unit testing only.)
// Returns the string name of an enumerated value for a contact model.
std::string GetStringFromContactModel(ContactModel contact_model);

DiscreteContactApproximation GetDiscreteContactApproximationFromString(
    std::string_view discrete_contact_approximation);
std::string GetStringFromDiscreteContactApproximation(
    DiscreteContactApproximation discrete_contact_approximation);

// (Exposed for unit testing only.)
// Parses a string name for a contact representation and returns the enumerated
// value.  Valid string names are listed in MultibodyPlantConfig's class
// overview.
// @throws std::exception if an invalid string is passed in.
geometry::HydroelasticContactRepresentation
GetContactSurfaceRepresentationFromString(
    std::string_view contact_representation);

// (Exposed for unit testing only.)
// Returns the string name of an enumerated value for a contact representation.
std::string GetStringFromContactSurfaceRepresentation(
    geometry::HydroelasticContactRepresentation contact_representation);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
