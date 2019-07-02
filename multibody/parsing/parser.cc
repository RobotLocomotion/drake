#include "drake/multibody/parsing/parser.h"

#include <spruce.hh>

#include "drake/multibody/parsing/detail_sdf_parser.h"
#include "drake/multibody/parsing/detail_urdf_parser.h"

namespace drake {
namespace multibody {

using internal::AddModelFromSdfFile;
using internal::AddModelFromUrdfFile;
using internal::AddModelsFromSdfFile;

Parser::Parser(
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph)
    : plant_(plant), scene_graph_(scene_graph) {
  DRAKE_THROW_UNLESS(plant != nullptr);
}

namespace {
enum class FileType { kSdf, kUrdf };
FileType DetermineFileType(const std::string& file_name) {
  const std::string ext = spruce::path(file_name).extension();
  if ((ext == ".urdf") || (ext == ".URDF")) {
    return FileType::kUrdf;
  }
  if ((ext == ".sdf") || (ext == ".SDF")) {
    return FileType::kSdf;
  }
  throw std::runtime_error(fmt::format(
      "The file type '{}' is not supported for '{}'",
      ext, file_name));
}
}  // namespace

std::vector<ModelInstanceIndex> Parser::AddAllModelsFromFile(
    const std::string& file_name) {
  // Always search for a package.xml file, starting the crawl upward from
  // the file's path.
  package_map_.PopulateUpstreamToDrake(file_name);
  const FileType type = DetermineFileType(file_name);
  if (type == FileType::kSdf) {
    return AddModelsFromSdfFile(file_name, package_map_, plant_,
        scene_graph_);
  } else {
    return {AddModelFromUrdfFile(
        file_name, {}, package_map_, plant_, scene_graph_)};
  }
}

ModelInstanceIndex Parser::AddModelFromFile(
    const std::string& file_name,
    const std::string& model_name) {
  // Always search for a package.xml file, starting the crawl upward from
  // the file's path.
  package_map_.PopulateUpstreamToDrake(file_name);
  const FileType type = DetermineFileType(file_name);
  if (type == FileType::kSdf) {
    return AddModelFromSdfFile(file_name, model_name, package_map_,
        plant_, scene_graph_);
  } else {
    return AddModelFromUrdfFile(
        file_name, model_name, package_map_, plant_, scene_graph_);
  }
}

}  // namespace multibody
}  // namespace drake
