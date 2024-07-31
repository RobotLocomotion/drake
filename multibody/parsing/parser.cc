#include "drake/multibody/parsing/parser.h"

#include <optional>
#include <set>
#include <utility>

#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"
#include "drake/multibody/parsing/detail_collision_filter_groups_impl.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_composite_parse.h"
#include "drake/multibody/parsing/detail_instanced_name.h"
#include "drake/multibody/parsing/detail_parsing_workspace.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/detail_select_parser.h"

namespace drake {
namespace multibody {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;
using internal::CollisionFilterGroupResolver;
using internal::CollisionFilterGroupsImpl;
using internal::DataSource;
using internal::InstancedName;
using internal::ParserInterface;
using internal::ParsingWorkspace;
using internal::SelectParser;

// Storage for internals that need to have the same lifetime as a Parser
// instance, but avoid adding internal namespace details to parser.h.
struct Parser::ParserInternalData {
  // Collision filter groups that use InstancedName. This representation is
  // invariant with respect to model renaming via the plant.
  CollisionFilterGroupsImpl<InstancedName> collision_filter_groups_storage_;
};

Parser::Parser(MultibodyPlant<double>* plant,
               geometry::SceneGraph<double>* scene_graph)
    : Parser(plant, scene_graph, {}) {}

Parser::Parser(MultibodyPlant<double>* plant,
               std::string_view model_name_prefix)
    : Parser(plant, nullptr, model_name_prefix) {}

Parser::Parser(MultibodyPlant<double>* plant,
               geometry::SceneGraph<double>* scene_graph,
               std::string_view model_name_prefix)
    : plant_(plant), data_(new Parser::ParserInternalData) {
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

Parser::~Parser() {}

CollisionFilterGroups Parser::GetCollisionFilterGroups() const {
  return CollisionFilterGroups(ConvertInstancedNamesToStrings(
      data_->collision_filter_groups_storage_, *plant_));
}

std::vector<ModelInstanceIndex> Parser::AddModels(
    const std::filesystem::path& file_name) {
  const std::string filename_string{file_name.string()};
  DataSource data_source(DataSource::kFilename, &filename_string);
  ParserInterface& parser = SelectParser(diagnostic_policy_, file_name);
  auto composite = internal::CompositeParse::MakeCompositeParse(this);
  auto result = parser.AddAllModels(data_source, model_name_prefix_,
                                    composite->workspace());
  composite->Finish();
  return result;
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

std::vector<ModelInstanceIndex> Parser::AddModelsFromString(
    const std::string& file_contents, const std::string& file_type) {
  DataSource data_source(DataSource::kContents, &file_contents);
  const std::string pseudo_name(data_source.GetStem() + "." + file_type);
  ParserInterface& parser = SelectParser(diagnostic_policy_, pseudo_name);
  auto composite = internal::CompositeParse::MakeCompositeParse(this);
  auto result = parser.AddAllModels(data_source, model_name_prefix_,
                                    composite->workspace());
  composite->Finish();
  return result;
}

void Parser::ResolveCollisionFilterGroupsFromCompositeParse(
    internal::CollisionFilterGroupResolver* resolver) {
  DRAKE_DEMAND(resolver != nullptr);

  const CollisionFilterGroupsImpl<internal::InstancedName> groups =
      resolver->Resolve(diagnostic_policy_);

  // Merge the groups found during a composite parse into the accumulated groups
  // held by this parser.
  data_->collision_filter_groups_storage_.Merge(groups);

  // Deprecated 2024-10-01: copy the merged groups to the legacy data object.
  collision_filter_groups_ = GetCollisionFilterGroups();
}

}  // namespace multibody
}  // namespace drake
