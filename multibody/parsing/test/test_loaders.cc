#include "drake/multibody/parsing/test/test_loaders.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/detail_sdf_parser.h"
#include "drake/multibody/parsing/detail_urdf_parser.h"

namespace drake {
namespace multibody {
namespace test {

void LoadFromSdf(
    const std::string& base_name,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  // The empty-string second argument here means that the model_name comes from
  // the "name" attribute of the SDF.  This is a sensible default for the unit
  // tests that call us.
  const std::string sdf_path = FindResourceOrThrow(base_name + ".sdf");
  detail::AddModelFromSdfFile(sdf_path, "", plant, scene_graph);
}

void LoadFromUrdf(
    const std::string& base_name,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  // The empty-string second argument here means that the model_name comes from
  // the "name" attribute of the URDF.  This is a sensible default for the unit
  // tests that call us.
  const std::string urdf_path = FindResourceOrThrow(base_name + ".urdf");
  detail::AddModelFromUrdfFile(urdf_path, "", plant, scene_graph);
}

}  // namespace test
}  // namespace multibody
}  // namespace drake
