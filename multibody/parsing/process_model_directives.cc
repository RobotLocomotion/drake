#include "drake/multibody/parsing/process_model_directives.h"

#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "drake/common/diagnostic_policy.h"
#include "drake/multibody/parsing/detail_composite_parse.h"
#include "drake/multibody/parsing/detail_dmd_parser.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace multibody {
namespace parsing {

using std::make_unique;

using drake::multibody::internal::CompositeParse;
using multibody::internal::DataSource;

namespace {

// If `*ptr` is null, construct `T(args...)` and reassign the pointer.
template <typename T, typename... Args>
std::unique_ptr<T> ConstructIfNullAndReassign(T** ptr, Args&&... args) {
  DRAKE_DEMAND(ptr != nullptr);
  std::unique_ptr<T> out;
  if (!*ptr) {
    out = std::make_unique<T>(std::forward<Args>(args)...);
    *ptr = out.get();
  }
  return out;
}

}  // namespace

std::string ResolveModelDirectiveUri(const std::string& uri,
                                     const PackageMap& package_map) {
  ::drake::internal::DiagnosticPolicy policy;
  return ::drake::multibody::internal::ResolveUri(policy, uri, package_map, "");
}

void ProcessModelDirectives(
    const ModelDirectives& directives,
    MultibodyPlant<double>* plant,
    std::vector<ModelInstanceInfo>* added_models,
    Parser* parser) {
  auto tmp_parser = ConstructIfNullAndReassign<Parser>(&parser, plant);
  auto composite = CompositeParse::MakeCompositeParse(parser);
  std::vector<ModelInstanceInfo> models =
      multibody::internal::ParseModelDirectives(
          directives, "", composite->workspace());
  if (added_models) {
    added_models->insert(added_models->end(), models.begin(), models.end());
  }
}

std::vector<ModelInstanceInfo> ProcessModelDirectives(
    const ModelDirectives& directives,
    Parser* parser) {
  DRAKE_THROW_UNLESS(parser != nullptr);
  std::vector<ModelInstanceInfo> added_models;
  ProcessModelDirectives(directives, &parser->plant(), &added_models, parser);
  return added_models;
}

ModelDirectives LoadModelDirectives(const std::string& filename) {
  return multibody::internal::LoadModelDirectives(
      {DataSource::kFilename, &filename});
}

ModelDirectives LoadModelDirectivesFromString(
    const std::string& model_directives) {
  return multibody::internal::LoadModelDirectives(
      {DataSource::kContents, &model_directives});
}

void FlattenModelDirectives(
    const ModelDirectives& directives,
    const PackageMap& package_map,
    ModelDirectives* out) {
  // NOTE: Does not handle scoping!
  for (auto& directive : directives.directives) {
    if (directive.add_directives) {
      auto& sub = *directive.add_directives;
      const auto sub_file = ResolveModelDirectiveUri(sub.file, package_map);
      FlattenModelDirectives(LoadModelDirectives(sub_file), package_map, out);
    } else {
      out->directives.push_back(directive);
    }
  }
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
