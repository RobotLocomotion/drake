#include "drake/multibody/parsing/file_parser.h"

#include <spruce.hh>

#include "drake/multibody/parsing/sdf_parser.h"
#include "drake/multibody/parsing/urdf_parser.h"

namespace drake {
namespace multibody {
namespace parsing {

std::vector<ModelInstanceIndex> AddModelsFromFile(
    const std::string& file_name,
    const std::string& model_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  const std::string ext = spruce::path(file_name).extension();
  if ((ext == ".urdf") || (ext == ".URDF")) {
    return {AddModelFromUrdfFile(file_name, model_name, plant, scene_graph)};
  }
  if ((ext == ".sdf") || (ext == ".SDF")) {
    if (model_name.empty()) {
      return AddModelsFromSdfFile(file_name, plant, scene_graph);
    } else {
      return {AddModelFromSdfFile(file_name, model_name, plant, scene_graph)};
    }
  }
  throw std::runtime_error(fmt::format(
      "AddModelsFromFile: The file type '{}' is not supported for '{}'",
      ext, file_name));
}

std::vector<ModelInstanceIndex> AddModelsFromFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  return AddModelsFromFile(file_name, {}, plant, scene_graph);
}

ModelInstanceIndex AddModelFromFile(
    const std::string& file_name,
    const std::string& model_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  const std::vector<ModelInstanceIndex> indices =
      AddModelsFromFile(file_name, model_name, plant, scene_graph);
  if (indices.size() != 1) {
    throw std::runtime_error(fmt::format(
        "AddModelFromFile: The file '{}' contained too many ({}) models",
        file_name, indices.size()));
  }
  return indices[0];
}

/// Alternate *singular* version of AddModelsFromFile where exactly model is
/// added, that uses the input file's "name" element for the name of the newly
/// created model instance.  It is an error to call this using an SDF file with
/// more than one `<model>` element.
ModelInstanceIndex AddModelFromFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  return AddModelFromFile(file_name, {}, plant, scene_graph);
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
