#include "drake/multibody/parsing/parser.h"

#include <optional>
#include <set>

#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_composite_parse.h"
#include "drake/multibody/parsing/detail_parsing_workspace.h"
#include "drake/multibody/parsing/detail_path_utils.h"
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
    : Parser(plant, scene_graph, {}) {}

Parser::Parser(MultibodyPlant<double>* plant,
               std::string_view model_name_prefix)
    : Parser(plant, nullptr, model_name_prefix) {}

Parser::Parser(MultibodyPlant<double>* plant,
               geometry::SceneGraph<double>* scene_graph,
               std::string_view model_name_prefix)
    : plant_(plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  if (!model_name_prefix.empty()) {
    model_name_prefix_ = std::string(model_name_prefix);
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

std::vector<ModelInstanceIndex> Parser::AddModels(
    const std::filesystem::path& file_name) {
  const std::string filename_string{file_name.string()};
  DataSource data_source(DataSource::kFilename, &filename_string);
  ParserInterface& parser = SelectParser(diagnostic_policy_, file_name);
  auto composite = internal::CompositeParse::MakeCompositeParse(this);
  return parser.AddAllModels(data_source, model_name_prefix_,
                             composite->workspace());
}

std::vector<ModelInstanceIndex> Parser::AddModelsFromUrl(
    const std::string& url) {
  const std::string file_name = internal::ResolveUri(
      diagnostic_policy_, url, package_map_, {});
  if (file_name.empty()) {
    return {};
  }
  return AddModels(file_name);
}

std::vector<ModelInstanceIndex> Parser::AddAllModelsFromFile(
    const std::string& file_name) {
  return AddModels(file_name);
}

std::vector<ModelInstanceIndex> Parser::AddModelsFromString(
    const std::string& file_contents, const std::string& file_type) {
  DataSource data_source(DataSource::kContents, &file_contents);
  const std::string pseudo_name(data_source.GetStem() + "." + file_type);
  ParserInterface& parser = SelectParser(diagnostic_policy_, pseudo_name);
  auto composite = internal::CompositeParse::MakeCompositeParse(this);
  return parser.AddAllModels(data_source, model_name_prefix_,
                             composite->workspace());
}

ModelInstanceIndex Parser::AddModelFromFile(
    const std::string& file_name,
    const std::string& model_name) {
  DataSource data_source(DataSource::kFilename, &file_name);
  ParserInterface& parser = SelectParser(diagnostic_policy_, file_name);
  auto composite = internal::CompositeParse::MakeCompositeParse(this);
  std::optional<ModelInstanceIndex> maybe_model;
  maybe_model = parser.AddModel(data_source, model_name, model_name_prefix_,
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
  maybe_model = parser.AddModel(data_source, model_name, model_name_prefix_,
                                composite->workspace());
  if (!maybe_model.has_value()) {
    throw std::runtime_error(
        fmt::format("{}: parsing failed", pseudo_name));
  }
  return *maybe_model;
}

}  // namespace multibody
}  // namespace drake
