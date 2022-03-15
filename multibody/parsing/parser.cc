#include "drake/multibody/parsing/parser.h"

#include <optional>

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
  ParsingWorkspace workspace{package_map_, diagnostic_policy_, plant_};
  const FileType type = DetermineFileType(file_name);
  if (type == FileType::kSdf) {
    return AddModelsFromSdf(data_source, workspace);
  } else {
    const std::optional<ModelInstanceIndex> maybe_model =
        AddModelFromUrdf(data_source, {}, {}, workspace);
    if (maybe_model.has_value()) {
      return {*maybe_model};
    } else {
      throw std::runtime_error(
          fmt::format("{}: URDF model file parsing failed", file_name));
    }
  }
}

ModelInstanceIndex Parser::AddModelFromFile(
    const std::string& file_name,
    const std::string& model_name) {
  DataSource data_source(DataSource::kFilename, &file_name);
  ParsingWorkspace workspace{package_map_, diagnostic_policy_, plant_};
  const FileType type = DetermineFileType(file_name);
  std::optional<ModelInstanceIndex> maybe_model;
  if (type == FileType::kSdf) {
    maybe_model = AddModelFromSdf(data_source, model_name, workspace);
  } else {
    maybe_model = AddModelFromUrdf(data_source, model_name, {}, workspace);
  }
  if (!maybe_model.has_value()) {
    throw std::runtime_error(
        fmt::format("{}: parsing failed", file_name));
  }
  return *maybe_model;
}

ModelInstanceIndex Parser::AddModelFromString(
    const std::string& file_contents,
    const std::string& file_type,
    const std::string& model_name) {
  DataSource data_source(DataSource::kContents, &file_contents);
  const std::string pseudo_name(data_source.GetStem() + "." + file_type);
  ParsingWorkspace workspace{package_map_, diagnostic_policy_, plant_};
  const FileType type = DetermineFileType(pseudo_name);
  std::optional<ModelInstanceIndex> maybe_model;
  if (type == FileType::kSdf) {
    maybe_model = AddModelFromSdf(data_source, model_name, workspace);
  } else {
    maybe_model = AddModelFromUrdf(data_source, model_name, {}, workspace);
  }
  if (!maybe_model.has_value()) {
    throw std::runtime_error(
        fmt::format("{}: parsing failed", pseudo_name));
  }
  return *maybe_model;
}

}  // namespace multibody
}  // namespace drake
