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
#include "drake/multibody/tree/scoped_name.h"

namespace drake {
namespace multibody {
namespace parsing {

using std::make_unique;

using drake::multibody::internal::CompositeParse;
using drake::multibody::internal::DmdScopedNameJoin;
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

AddWeld ApplyDirectiveNamespace(const AddWeld& orig,
                                const std::string& model_namespace) {
  if (model_namespace.empty()) return orig;
  AddWeld result = orig;
  result.parent = DmdScopedNameJoin(model_namespace, orig.parent).to_string();
  result.child = DmdScopedNameJoin(model_namespace, orig.child).to_string();
  return result;
}

AddModel ApplyDirectiveNamespace(const AddModel& orig,
                                 const std::string& model_namespace) {
  if (model_namespace.empty()) return orig;
  AddModel result = orig;
  result.name = DmdScopedNameJoin(model_namespace, orig.name).to_string();
  return result;
}

AddModelInstance ApplyDirectiveNamespace(const AddModelInstance& orig,
                                         const std::string& model_namespace) {
  if (model_namespace.empty()) return orig;
  AddModelInstance result = orig;
  result.name = DmdScopedNameJoin(model_namespace, orig.name).to_string();
  return result;
}

AddFrame ApplyDirectiveNamespace(const AddFrame& orig,
                                 const std::string& model_namespace) {
  DRAKE_THROW_UNLESS(orig.IsValid());  // Ensures `base_frame` is present.
  AddFrame result = orig;
  // Handle the general case of prepending the model namespace.
  if (!model_namespace.empty()) {
    result.name = DmdScopedNameJoin(model_namespace, orig.name).to_string();
    result.X_PF.base_frame =
        DmdScopedNameJoin(model_namespace, orig.X_PF.base_frame.value())
            .to_string();
  }
  // Handle the special case of an add_frame name with no namespace.
  // Note that we don't need to update result.X_PF.base_frame -- it's already
  // correct.
  const ScopedName orig_scoped_name = ScopedName::Parse(orig.name);
  if (orig_scoped_name.get_namespace().empty()) {
    // If the original directive's name had no namespace, the flattened
    // directive's name has to get both the model namespace and the
    // base frame's namespace prepended.
    const std::string base_frame_namespace(
        ScopedName::Parse(orig.X_PF.base_frame.value()).get_namespace());
    const std::string orig_element(orig_scoped_name.get_element());
    result.name =
        DmdScopedNameJoin(
            model_namespace,
            DmdScopedNameJoin(base_frame_namespace, orig_element).to_string())
            .to_string();
  }
  return result;
}

AddCollisionFilterGroup ApplyDirectiveNamespace(
    const AddCollisionFilterGroup& orig, const std::string& model_namespace) {
  if (model_namespace.empty()) return orig;
  AddCollisionFilterGroup result;
  ScopedName orig_name =
      DmdScopedNameJoin(orig.model_namespace.value_or(""), orig.name);
  ScopedName full_name =
      DmdScopedNameJoin(model_namespace, orig_name.to_string());
  result.name = full_name.get_element();
  if (!full_name.get_namespace().empty()) {
    result.model_namespace = full_name.get_namespace();
  }
  // These will also have the `.model_namespace` applied.
  result.members = orig.members;
  result.ignored_collision_filter_groups = orig.ignored_collision_filter_groups;
  return result;
}

void FlattenModelDirectivesInternal(const ModelDirectives& directives,
                                    const PackageMap& package_map,
                                    ModelDirectives* out,
                                    const std::string& model_namespace) {
  for (auto& directive : directives.directives) {
    if (directive.add_directives) {
      const AddDirectives& sub = *directive.add_directives;
      const std::string sub_file =
          ResolveModelDirectiveUri(sub.file, package_map);
      FlattenModelDirectivesInternal(
          LoadModelDirectives(sub_file), package_map, out,
          (sub.model_namespace.has_value()
               ? DmdScopedNameJoin(model_namespace, *sub.model_namespace)
                     .to_string()
               : model_namespace));
    } else if (directive.add_weld) {
      ModelDirective result;
      result.add_weld =
          ApplyDirectiveNamespace(*directive.add_weld, model_namespace);
      out->directives.push_back(result);
    } else if (directive.add_model) {
      ModelDirective result;
      result.add_model =
          ApplyDirectiveNamespace(*directive.add_model, model_namespace);
      out->directives.push_back(result);
    } else if (directive.add_model_instance) {
      ModelDirective result;
      result.add_model_instance = ApplyDirectiveNamespace(
          *directive.add_model_instance, model_namespace);
      out->directives.push_back(result);
    } else if (directive.add_frame) {
      ModelDirective result;
      result.add_frame =
          ApplyDirectiveNamespace(*directive.add_frame, model_namespace);
      out->directives.push_back(result);
    } else if (directive.add_collision_filter_group) {
      ModelDirective result;
      result.add_collision_filter_group = ApplyDirectiveNamespace(
          *directive.add_collision_filter_group, model_namespace);
      out->directives.push_back(result);
    } else {
      DRAKE_UNREACHABLE();
    }
  }
}

}  // namespace

std::string ResolveModelDirectiveUri(const std::string& uri,
                                     const PackageMap& package_map) {
  ::drake::internal::DiagnosticPolicy policy;
  const auto resolved =
      ::drake::multibody::internal::ResolveUri(policy, uri, package_map, "");
  return resolved.GetStringPathIfExists();
}

void ProcessModelDirectives(const ModelDirectives& directives,
                            MultibodyPlant<double>* plant,
                            std::vector<ModelInstanceInfo>* added_models,
                            Parser* parser) {
  auto tmp_parser = ConstructIfNullAndReassign<Parser>(&parser, plant);
  auto composite = CompositeParse::MakeCompositeParse(parser);
  std::vector<ModelInstanceInfo> models =
      multibody::internal::ParseModelDirectives(directives, "",
                                                composite->workspace());
  if (added_models) {
    added_models->insert(added_models->end(), models.begin(), models.end());
  }
  composite->Finish();
}

std::vector<ModelInstanceInfo> ProcessModelDirectives(
    const ModelDirectives& directives, Parser* parser) {
  DRAKE_THROW_UNLESS(parser != nullptr);
  std::vector<ModelInstanceInfo> added_models;
  ProcessModelDirectives(directives, &parser->plant(), &added_models, parser);
  return added_models;
}

ModelDirectives LoadModelDirectives(const std::filesystem::path& filename) {
  const std::string filename_str = filename.string();
  return multibody::internal::LoadModelDirectives(
      {DataSource::kFilename, &filename_str});
}

ModelDirectives LoadModelDirectivesFromString(
    const std::string& model_directives) {
  return multibody::internal::LoadModelDirectives(
      {DataSource::kContents, &model_directives});
}

void FlattenModelDirectives(const ModelDirectives& directives,
                            const PackageMap& package_map,
                            ModelDirectives* out) {
  FlattenModelDirectivesInternal(directives, package_map, out, "");
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
