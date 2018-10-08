#include "drake/multibody/multibody_tree/parsing/test/test_loaders.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace test {

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
