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

Parser::Parser(MultibodyPlant<double>* plant,
               geometry::SceneGraph<double>* scene_graph)
    : Parser({}, plant, scene_graph) {}

Parser::Parser(std::string_view model_scope, MultibodyPlant<double>* plant,
               geometry::SceneGraph<double>* scene_graph)
    : plant_(plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  if (!model_scope.empty()) {
    model_scope_ = std::string(model_scope);
  }

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
  DataSource data_source(DataSource::kFilename, &file_name);
  ParserInterface& parser = SelectParser(diagnostic_policy_, file_name);
  auto composite = internal::CompositeParse::MakeCompositeParse(this);
  return parser.AddAllModels(data_source, model_scope_, composite->workspace());
}

std::vector<ModelInstanceIndex> Parser::AddModelsFromString(
    const std::string& file_contents, const std::string& file_type) {
  DataSource data_source(DataSource::kContents, &file_contents);
  const std::string pseudo_name(data_source.GetStem() + "." + file_type);
  ParserInterface& parser = SelectParser(diagnostic_policy_, pseudo_name);
  auto composite = internal::CompositeParse::MakeCompositeParse(this);
  return parser.AddAllModels(data_source, model_scope_, composite->workspace());
}

ModelInstanceIndex Parser::AddModelFromFile(
    const std::string& file_name,
    const std::string& model_name) {
  DataSource data_source(DataSource::kFilename, &file_name);
  ParserInterface& parser = SelectParser(diagnostic_policy_, file_name);
  auto composite = internal::CompositeParse::MakeCompositeParse(this);
  std::optional<ModelInstanceIndex> maybe_model;
  maybe_model = parser.AddModel(data_source, model_name, model_scope_,
                                composite->workspace());
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
  ParserInterface& parser = SelectParser(diagnostic_policy_, pseudo_name);
  auto composite = internal::CompositeParse::MakeCompositeParse(this);
  std::optional<ModelInstanceIndex> maybe_model;
  maybe_model = parser.AddModel(data_source, model_name, model_scope_,
                                composite->workspace());
  if (!maybe_model.has_value()) {
    throw std::runtime_error(
        fmt::format("{}: parsing failed", pseudo_name));
  }
  return *maybe_model;
}

}  // namespace multibody
}  // namespace drake
