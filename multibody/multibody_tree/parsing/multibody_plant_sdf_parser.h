#pragma once

#include <string>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace parsing {

/// Parses a `<model>` element from the SDF file specified by `file_name` and
/// adds it to `plant`. The SDF file can only contain a single `<model>`
/// element. `<world>` elements (used for instance to specify gravity) are
/// ignored by this method.
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
/// @param plant
///   A pointer to a mutable MultibodyPlant object to which the model will be
///   added.
/// @param scene_graph
///   A pointer to a mutable SceneGraph object used for geometry registration
///   (either to model visual or contact geometry).
void AddModelFromSdfFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr);
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
