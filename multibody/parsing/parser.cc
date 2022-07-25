#include "drake/multibody/parsing/parser.h"

#include <optional>
#include <set>

#include "drake/common/filesystem.h"
#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_composite_parse.h"
#include "drake/multibody/parsing/detail_parsing_workspace.h"
#include "drake/multibody/parsing/detail_select_parser.h"

namespace drake {
namespace multibody {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;
using internal::CollisionFilterGroupResolver;
using internal::DataSource;
using internal::ParserInterface;
using internal::ParsingWorkspace;
using internal::SelectParser;

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
  ParserInterface* parser = SelectParser(diagnostic_policy_, file_name);
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
  ParserInterface* parser = SelectParser(diagnostic_policy_, file_name);
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
  ParserInterface* parser = SelectParser(diagnostic_policy_, pseudo_name);
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
