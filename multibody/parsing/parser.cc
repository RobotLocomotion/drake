#include "drake/multibody/parsing/parser.h"

#include <optional>
#include <set>

#include "drake/common/filesystem.h"
#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_composite_parse.h"
#include "drake/multibody/parsing/detail_dmd_parser.h"
#include "drake/multibody/parsing/detail_mujoco_parser.h"
#include "drake/multibody/parsing/detail_parsing_workspace.h"
#include "drake/multibody/parsing/detail_sdf_parser.h"
#include "drake/multibody/parsing/detail_urdf_parser.h"

namespace drake {
namespace multibody {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;
using internal::AddModelFromSdf;
using internal::AddModelFromUrdf;
using internal::AddModelsFromSdf;
using internal::CollisionFilterGroupResolver;
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

  auto warnings_maybe_strict =
      [this](const DiagnosticDetail& detail) {
        if (is_strict_) {
          diagnostic_policy_.Error(detail);
        } else {
          diagnostic_policy_.WarningDefaultAction(detail);
        }
      };
  diagnostic_policy_.SetActionForWarnings(warnings_maybe_strict);
}

namespace {

bool EndsWith(const std::string_view str, const std::string_view ext) {
  return (ext.size() < str.size()) &&
      std::equal(str.end() - ext.size(), str.end(), ext.begin());
}

enum class FileType { kSdf, kUrdf, kMjcf, kDmdf };
FileType DetermineFileType(const std::string& file_name) {
  if (EndsWith(file_name, ".urdf") || EndsWith(file_name, ".URDF")) {
    return FileType::kUrdf;
  }
  if (EndsWith(file_name, ".sdf") || EndsWith(file_name, ".SDF")) {
    return FileType::kSdf;
  }
  if (EndsWith(file_name, ".xml") || EndsWith(file_name, ".XML")) {
    return FileType::kMjcf;
  }
  if (EndsWith(file_name, ".dmd.yaml") || EndsWith(file_name, ".DMD.YAML")) {
    return FileType::kDmdf;
  }
  throw std::runtime_error(fmt::format(
      "The file '{}' is not a recognized type."
      " Known types are: .urdf, .sdf, .xml (Mujoco), .dmd.yaml",
      file_name));
}

internal::ParserInterface* SelectParser(const std::string& file_name) {
  static internal::UrdfParser urdf;
  static internal::SdfParser sdf;
  static internal::MujocoParser mujoco;
  static internal::DmdParser dmd;
  const FileType type = DetermineFileType(file_name);
  switch (type) {
    case FileType::kUrdf:
      return &urdf;
    case FileType::kSdf:
      return &sdf;
    case FileType::kMjcf:
      return &mujoco;
    case FileType::kDmdf:
      return &dmd;
  }
  DRAKE_UNREACHABLE();
}

}  // namespace

std::vector<ModelInstanceIndex> Parser::AddAllModelsFromFile(
    const std::string& file_name) {
  return CompositeAddAllModelsFromFile(file_name, {});
}

std::vector<ModelInstanceIndex> Parser::CompositeAddAllModelsFromFile(
    const std::string& file_name,
    internal::CompositeParse* composite) {
  DataSource data_source(DataSource::kFilename, &file_name);
  CollisionFilterGroupResolver resolver{plant_};
  ParsingWorkspace workspace{
    package_map_, diagnostic_policy_, plant_,
    composite ? &composite->collision_resolver() : &resolver, SelectParser};
  internal::ParserInterface* parser = SelectParser(file_name);
  std::vector<ModelInstanceIndex> result;
  result = parser->AddAllModels(data_source, {}, workspace);
  if (result.empty()) {
    throw std::runtime_error(
        fmt::format("{}: parsing failed", file_name));
  }
  if (!composite) { resolver.Resolve(diagnostic_policy_); }
  return result;
}

ModelInstanceIndex Parser::AddModelFromFile(
    const std::string& file_name,
    const std::string& model_name) {
  return CompositeAddModelFromFile(file_name, model_name, {});
}

ModelInstanceIndex Parser::CompositeAddModelFromFile(
    const std::string& file_name,
    const std::string& model_name,
    internal::CompositeParse* composite) {
  DataSource data_source(DataSource::kFilename, &file_name);
  CollisionFilterGroupResolver resolver{plant_};
  ParsingWorkspace workspace{
    package_map_, diagnostic_policy_, plant_,
    composite ? &composite->collision_resolver() : &resolver, SelectParser};
  std::optional<ModelInstanceIndex> maybe_model;
  internal::ParserInterface* parser = SelectParser(file_name);
  maybe_model = parser->AddModel(data_source, model_name, {}, workspace);
  if (!maybe_model.has_value()) {
    throw std::runtime_error(
        fmt::format("{}: parsing failed", file_name));
  }
  if (!composite) { resolver.Resolve(diagnostic_policy_); }
  return *maybe_model;
}

ModelInstanceIndex Parser::AddModelFromString(
    const std::string& file_contents,
    const std::string& file_type,
    const std::string& model_name) {
  return CompositeAddModelFromString(file_contents, file_type, model_name, {});
}

ModelInstanceIndex Parser::CompositeAddModelFromString(
    const std::string& file_contents,
    const std::string& file_type,
    const std::string& model_name,
    internal::CompositeParse* composite) {
  DataSource data_source(DataSource::kContents, &file_contents);
  const std::string pseudo_name(data_source.GetStem() + "." + file_type);
  CollisionFilterGroupResolver resolver{plant_};
  ParsingWorkspace workspace{
    package_map_, diagnostic_policy_, plant_,
    composite ? &composite->collision_resolver() : &resolver, SelectParser};
  std::optional<ModelInstanceIndex> maybe_model;
  internal::ParserInterface* parser = SelectParser(pseudo_name);
  maybe_model = parser->AddModel(data_source, model_name, {}, workspace);
  if (!maybe_model.has_value()) {
    throw std::runtime_error(
        fmt::format("{}: parsing failed", pseudo_name));
  }
  if (!composite) { resolver.Resolve(diagnostic_policy_); }
  return *maybe_model;
}

}  // namespace multibody
}  // namespace drake
