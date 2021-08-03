#pragma once

#include <optional>
#include <string>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

// Parses a `<robot>` element from the URDF file specified by @p file_name and
// adds it to @p plant.  A new model instance will be added to @p plant.
//
// @throws std::exception if the file is not in accordance with the URDF
// specification.  The exception contains a message with a list of errors
// encountered while parsing the file.
//
// @param data_source
//   The URDF data to be parsed.
// @param model_name
//   The name given to the newly created instance of this model.  If
//   empty, the "name" attribute from the model tag will be used.
// @param parent_model_name
//   Optional name of parent model. If set, this will be prefixed with the model
//   name (either `model_name` or from the "name" attribute) using the SDFormat
//   scope delimiter "::". The prefixed name will used as the name given to the
//   newly created instance of this model.
// @param plant
//   A pointer to a mutable MultibodyPlant object to which the model will be
//   added.
// @param scene_graph
//   A pointer to a mutable SceneGraph object used for geometry registration
//   (either to model visual or contact geometry).  May be nullptr.
// @returns The model instance index for the newly added model.
ModelInstanceIndex AddModelFromUrdf(
    const DataSource& data_source,
    const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const PackageMap& package_map,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
