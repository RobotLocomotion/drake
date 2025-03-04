#include "drake/multibody/parsing/test/test_loaders.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace multibody {
namespace test {

void LoadFromSdf(const std::string& base_name, MultibodyPlant<double>* plant,
                 geometry::SceneGraph<double>* scene_graph) {
  const std::string sdf_path = FindResourceOrThrow(base_name + ".sdf");
  Parser parser(plant, scene_graph);
  parser.AddModels(sdf_path);
}

void LoadFromUrdf(const std::string& base_name, MultibodyPlant<double>* plant,
                  geometry::SceneGraph<double>* scene_graph) {
  const std::string urdf_path = FindResourceOrThrow(base_name + ".urdf");
  Parser parser(plant, scene_graph);
  parser.AddModels(urdf_path);
}

std::ostream& operator<<(std::ostream& os, const ModelLoadFunction& func) {
  const auto* const target =
      func.target<void (*)(const std::string&, MultibodyPlant<double>*,
                           geometry::SceneGraph<double>*)>();
  if (target != nullptr && *target == LoadFromSdf) {
    os << "LoadFromSdf()";
  } else if (target != nullptr && *target == LoadFromUrdf) {
    os << "LoadFromUrdf()";
  } else {
    os << "unknown ModelLoadFunction";
  }
  return os;
}

}  // namespace test
}  // namespace multibody
}  // namespace drake
