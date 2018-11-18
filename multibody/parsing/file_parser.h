#pragma once

#include <string>
#include <vector>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace parsing {

/// Parses the SDF or URDF file named in @p file_name and adds its model(s) to
/// @p plant.
///
/// SDF files may contain multiple `<model>` elements.  New model instances
/// will be added to @p plant for each `<model>` tag in the file.
///
/// URDF files contain a single `<robot>` element.  Only a single model
/// instance will be added to @p plant.
///
/// @throws std::exception in case of errors.
///
/// @param file_name
///   The name of the SDF or URDF file to be parsed.  The file type will be
///   inferred from the extension.
/// @param model_name
///   The name given to the newly created instance of this model.  If empty,
///   the "name" attribute from the `<model>` or `<robot>` tag will be used.
///   If an SDF file contains multiple `<model>` tags, this argument must be
///   empty.
/// @param plant
///   A pointer to a mutable MultibodyPlant object to which the model(s) will
///   be added; `plant->is_finalized()` must be `false`.
/// @param scene_graph
///   A pointer to a mutable SceneGraph object used for geometry registration
///   (either to model visual or contact geometry).  May be nullptr.
/// @returns The set of model instance indices for the newly added models.
std::vector<ModelInstanceIndex> AddModelsFromFile(
    const std::string& file_name,
    const std::string& model_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr);

/// Alternate version of AddModelsFromFile that always uses the input file's
/// "name" element for the name of the newly created model instance(s).
std::vector<ModelInstanceIndex> AddModelsFromFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr);

/// Alternate *singular* version of AddModelsFromFile where exactly one model
/// is added.  It is an error to call this using an SDF file with more than one
/// `<model>` element.
ModelInstanceIndex AddModelFromFile(
    const std::string& file_name,
    const std::string& model_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr);

/// Alternate *singular* version of AddModelsFromFile where exactly model is
/// added, that uses the input file's "name" element for the name of the newly
/// created model instance.  It is an error to call this using an SDF file with
/// more than one `<model>` element.
ModelInstanceIndex AddModelFromFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr);

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
