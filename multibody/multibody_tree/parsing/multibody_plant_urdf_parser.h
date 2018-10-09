#pragma once

#include <string>
#include <vector>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace parsing {

/// Parses a `<robot>` element from the URDF file specified by @p file_name and
/// adds it to @p plant.  A new model instance will be added to @p plant.
///
/// @throws std::runtime_error if the file is not in accordance with the URDF
/// specification.  The exception contains a message with a list of errors
/// encountered while parsing the file.
///
/// @param file_name
///   The name of the URDF file to be parsed.
/// @param model_name
///   The name given to the newly created instance of this model.  If
///   empty, the "name" attribute from the model tag will be used.
/// @param plant
///   A pointer to a mutable MultibodyPlant object to which the model will be
///   added.
/// @param scene_graph
///   A pointer to a mutable SceneGraph object used for geometry registration
///   (either to model visual or contact geometry).  May be nullptr.
/// @returns The model instance index for the newly added model.
ModelInstanceIndex AddModelFromUrdfFile(
    const std::string& file_name,
    const std::string& model_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr);

/// Alternate version of AddModelFromUrdfFile which always uses the "name"
/// element from the model tag for the name of the newly created model
/// instance.
ModelInstanceIndex AddModelFromUrdfFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr);

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
