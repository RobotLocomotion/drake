#include "drake/multibody/parsing/test/test_loaders.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/sdf_parser.h"
#include "drake/multibody/parsing/urdf_parser.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace test {

void LoadFromSdf(
    const std::string& base_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  // The empty-string second argument here means that the model_name comes from
  // the "name" attribute of the SDF.  This is a sensible default for the unit
  // tests that call us.
  const std::string sdf_path = FindResourceOrThrow(base_name + ".sdf");
  parsing::detail::AddModelFromSdfFile(sdf_path, "", plant, scene_graph);
}

void LoadFromUrdf(
    const std::string& base_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  // The empty-string second argument here means that the model_name comes from
  // the "name" attribute of the URDF.  This is a sensible default for the unit
  // tests that call us.
  const std::string urdf_path = FindResourceOrThrow(base_name + ".urdf");
  parsing::detail::AddModelFromUrdfFile(urdf_path, "", plant, scene_graph);
}

}  // namespace test
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
