#include "drake/multibody/parsing/parser.h"

#include <spruce.hh>

#include "drake/multibody/parsing/sdf_parser.h"
#include "drake/multibody/parsing/urdf_parser.h"

namespace drake {
namespace multibody {
namespace parsing {

Parser::Parser(
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph)
    : plant_(plant), scene_graph_(scene_graph) {
  DRAKE_THROW_UNLESS(plant != nullptr);
}

std::vector<ModelInstanceIndex> Parser::AddModelsFromFile(
    const std::string& file_name,
    const std::string& model_name) {
  const std::string ext = spruce::path(file_name).extension();
  if ((ext == ".urdf") || (ext == ".URDF")) {
    return {AddModelFromUrdfFile(file_name, model_name, plant_, scene_graph_)};
  }
  if ((ext == ".sdf") || (ext == ".SDF")) {
    if (model_name.empty()) {
      return AddModelsFromSdfFile(file_name, plant_, scene_graph_);
    } else {
      return {AddModelFromSdfFile(file_name, model_name, plant_, scene_graph_)};
    }
  }
  throw std::runtime_error(fmt::format(
      "AddModelsFromFile: The file type '{}' is not supported for '{}'",
      ext, file_name));
}

ModelInstanceIndex Parser::AddModelFromFile(
    const std::string& file_name,
    const std::string& model_name) {
  const std::vector<ModelInstanceIndex> indices =
      this->AddModelsFromFile(file_name, model_name);
  if (indices.size() != 1) {
    throw std::runtime_error(fmt::format(
        "AddModelFromFile: The file '{}' contained multiple ({}) models",
        file_name, indices.size()));
  }
  return indices[0];
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
