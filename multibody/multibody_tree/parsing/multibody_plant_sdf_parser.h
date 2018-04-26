#pragma once

#include <memory>
#include <string>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace parsing {

/// This method makes a MultibodyPlant model for an acrobot SDF model.
/// @throws std::runtime_error if the parsing the acrobot SDF model fails.
void AddModelFromSdfFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant);
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
