#include "drake/multibody/parsing/parser.h"

#include "drake/common/filesystem.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_parsing_workspace.h"
#include "drake/multibody/parsing/detail_sdf_parser.h"
#include "drake/multibody/parsing/detail_urdf_parser.h"

namespace drake {
namespace multibody {

using internal::AddModelFromSdf;
using internal::AddModelFromUrdf;
using internal::AddModelsFromSdf;
using internal::DataSource;
using internal::ParsingWorkspace;

Parser::Parser(
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph)
    : plant_(plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  if (scene_graph != nullptr && !plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }
}

namespace {
enum class FileType { kSdf, kUrdf };
FileType DetermineFileType(const std::string& file_name) {
  const std::string ext = filesystem::path(file_name).extension().string();
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
  DataSource data_source(DataSource::kFilename, &file_name);
  const FileType type = DetermineFileType(file_name);
  if (type == FileType::kSdf) {
    return AddModelsFromSdf(data_source, package_map_, plant_);
  } else {
    ParsingWorkspace workspace{package_map_, diagnostic_policy_, plant_};
    return {AddModelFromUrdf(data_source, {}, {}, workspace)};
  }
}

ModelInstanceIndex Parser::AddModelFromFile(
    const std::string& file_name,
    const std::string& model_name) {
  DataSource data_source(DataSource::kFilename, &file_name);
  const FileType type = DetermineFileType(file_name);
  if (type == FileType::kSdf) {
    return AddModelFromSdf(data_source, model_name, package_map_, plant_);
  } else {
    ParsingWorkspace workspace{package_map_, diagnostic_policy_, plant_};
    return AddModelFromUrdf(data_source, model_name, {}, workspace);
  }
}

ModelInstanceIndex Parser::AddModelFromString(
    const std::string& file_contents,
    const std::string& file_type,
    const std::string& model_name) {
  DataSource data_source(DataSource::kContents, &file_contents);
  const FileType type = DetermineFileType(
      data_source.GetStem() + "." + file_type);
  if (type == FileType::kSdf) {
    return AddModelFromSdf(data_source, model_name, package_map_, plant_);
  } else {
    ParsingWorkspace workspace{package_map_, diagnostic_policy_, plant_};
    return AddModelFromUrdf(data_source, model_name, {}, workspace);
  }
}

}  // namespace multibody
}  // namespace drake
