#pragma once

#include <functional>
#include <string>

#include "drake/common/find_resource.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace test {

/// Load the model with a base name of @p base_name for this test
/// into @p plant.  The derived class will add the appropriate
/// extension to the filename (urdf, sdf) and call the appropriate
/// function to load the model.
typedef std::function<void(
    const std::string& base_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph)> ModelLoadFunction;

void LoadFromSdf(
    const std::string& base_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  const std::string sdf_path = FindResourceOrThrow(base_name + ".sdf");
  AddModelFromSdfFile(sdf_path, plant, scene_graph);
}

}  // namespace test
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
