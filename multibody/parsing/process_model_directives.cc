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

std::string NamespaceJoin(const std::string& prefix, const std::string& name) {
  return ScopedName::Join(prefix, name).to_string();
}

AddWeld ApplyDirectiveNamespace(const AddWeld& orig,
                                const std::string& prefix) {
  if (prefix.empty()) return orig;
  AddWeld result = orig;
  result.parent = NamespaceJoin(prefix, orig.parent);
  result.child = NamespaceJoin(prefix, orig.child);
  return result;
}

AddModel ApplyDirectiveNamespace(const AddModel& orig,
                                 const std::string& prefix) {
  if (prefix.empty()) return orig;
  AddModel result = orig;
  result.name = NamespaceJoin(prefix, orig.name);
  return result;
}

AddModelInstance ApplyDirectiveNamespace(const AddModelInstance& orig,
                                         const std::string& prefix) {
  if (prefix.empty()) return orig;
  AddModelInstance result = orig;
  result.name = NamespaceJoin(prefix, orig.name);
  return result;
}

AddFrame ApplyDirectiveNamespace(const AddFrame& orig,
                                 const std::string& prefix) {
  if (prefix.empty()) return orig;
  AddFrame result = orig;
  result.name = NamespaceJoin(prefix, orig.name);
  result.X_PF.base_frame = NamespaceJoin(prefix, *orig.X_PF.base_frame);
  return result;
}

AddCollisionFilterGroup ApplyDirectiveNamespace(
    const AddCollisionFilterGroup& orig, const std::string& prefix) {
  if (prefix.empty()) return orig;
  AddCollisionFilterGroup result = orig;
  result.name = NamespaceJoin(prefix, orig.name);
  result.members = {};
  for (const std::string& member : orig.members) {
    result.members.push_back(NamespaceJoin(prefix, member));
  }
  result.ignored_collision_filter_groups = {};
  for (const std::string& ignored : orig.ignored_collision_filter_groups) {
    result.ignored_collision_filter_groups.push_back(
        NamespaceJoin(prefix, ignored));
  }
  return result;
}

void FlattenModelDirectivesInternal(const ModelDirectives& directives,
                                    const PackageMap& package_map,
                                    ModelDirectives* out,
                                    const std::string& prefix) {
  for (auto& directive : directives.directives) {
    if (directive.add_directives) {
      const AddDirectives& sub = *directive.add_directives;
      const std::string sub_file =
          ResolveModelDirectiveUri(sub.file, package_map);
      FlattenModelDirectivesInternal(
          LoadModelDirectives(sub_file), package_map, out,
          (sub.model_namespace.has_value()
               ? NamespaceJoin(prefix, *sub.model_namespace)
               : prefix));
    } else if (directive.add_weld) {
      ModelDirective result;
      result.add_weld = ApplyDirectiveNamespace(*directive.add_weld, prefix);
      out->directives.push_back(result);
    } else if (directive.add_model) {
      ModelDirective result;
      result.add_model = ApplyDirectiveNamespace(*directive.add_model, prefix);
      out->directives.push_back(result);
    } else if (directive.add_model_instance) {
      ModelDirective result;
      result.add_model_instance =
          ApplyDirectiveNamespace(*directive.add_model_instance, prefix);
      out->directives.push_back(result);
    } else if (directive.add_frame) {
      ModelDirective result;
      // AddFrame has a subtle alteration of the scoped name semantics.  In
      // the event that the frame name has no namespace, it gets the namespace
      // of the base frame of its X_PF offset.
      ScopedName scoped_name = ScopedName::Parse(directive.add_frame->name);
      result.add_frame = ApplyDirectiveNamespace(*directive.add_frame, prefix);
      if (scoped_name.get_namespace().empty()) {
        // `name` was bare, so infer the namespace of the X_PF base frame.
        std::string base_frame_prefix =
            std::string(ScopedName::Parse(*result.add_frame->X_PF.base_frame)
                            .get_namespace());
        result.add_frame->name = NamespaceJoin(
            base_frame_prefix, std::string(scoped_name.get_element()));
      }
      out->directives.push_back(result);
    } else if (directive.add_collision_filter_group) {
      ModelDirective result;
      result.add_collision_filter_group = ApplyDirectiveNamespace(
          *directive.add_collision_filter_group, prefix);
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
  FlattenModelDirectivesInternal(directives, package_map, out, "");
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
