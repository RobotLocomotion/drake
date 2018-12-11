#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_deprecated.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace detail {

/// Parses a `<model>` element from the SDF file specified by `file_name` and
/// adds it to `plant`. The SDF file can only contain a single `<model>`
/// element. `<world>` elements (used for instance to specify gravity) are
/// ignored by this method.  A new model instance will be added to @p plant.
///
/// @throws std::runtime_error if the file is not in accordance with the SDF
/// specification containing a message with a list of errors encountered while
/// parsing the file.
/// @throws std::runtime_error if there is more than one `<model>` element or
/// zero of them.
/// @throws std::exception if plant is nullptr or if MultibodyPlant::Finalize()
/// was already called on `plant`.
///
/// @param file_name
///   The name of the SDF file to be parsed.
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
ModelInstanceIndex AddModelFromSdfFile(
    const std::string& file_name,
    const std::string& model_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr);

/// Parses all `<model>` elements from the SDF file specified by `file_name`
/// and adds them to `plant`. The SDF file can contain multiple `<model>`
/// elements. New model instances will be added to @p plant for each
/// `<model>` tag in the SDF file.
///
/// @throws std::runtime_error if the file is not in accordance with the SDF
/// specification containing a message with a list of errors encountered while
/// parsing the file.
/// @throws std::runtime_error if the file contains no models.
/// @throws std::exception if plant is nullptr or if MultibodyPlant::Finalize()
/// was already called on `plant`.
///
/// @param file_name
///   The name of the SDF file to be parsed.
/// @param plant
///   A pointer to a mutable MultibodyPlant object to which the model will be
///   added.
/// @param scene_graph
///   A pointer to a mutable SceneGraph object used for geometry registration
///   (either to model visual or contact geometry).  May be nullptr.
/// @returns The set of model instance indices for the newly added models.
std::vector<ModelInstanceIndex> AddModelsFromSdfFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr);

}  // namespace detail

#ifndef DRAKE_DOXYGEN_CXX
// TODO(jwnimmer-tri) Remove these forwarders on or about 2019-03-01.
namespace parsing {

DRAKE_DEPRECATED(
    "AddModelFromSdfFile is deprecated; please use the class "
    "drake::multibody::Parser instead.")
inline ModelInstanceIndex AddModelFromSdfFile(
    const std::string& file_name,
    const std::string& model_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  return detail::AddModelFromSdfFile(file_name, model_name, plant, scene_graph);
}

// Alternate version of AddModelFromSdfFile which always uses the "name"
// element from the model tag for the name of the newly created model instance.
DRAKE_DEPRECATED(
    "AddModelFromSdfFile is deprecated; please use the class "
    "drake::multibody::Parser instead.")
inline ModelInstanceIndex AddModelFromSdfFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  return detail::AddModelFromSdfFile(file_name, "", plant, scene_graph);
}

DRAKE_DEPRECATED(
    "AddModelsFromSdfFile is deprecated; please use the class "
    "drake::multibody::Parser instead.")
inline std::vector<ModelInstanceIndex> AddModelsFromSdfFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  return detail::AddModelsFromSdfFile(file_name, plant, scene_graph);
}

}  // namespace parsing
#endif  // DRAKE_DOXYGEN_CXX

}  // namespace multibody
}  // namespace drake
