#include "drake/multibody/parsing/process_model_directives.h"

#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/schema/transform.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"
#include "drake/multibody/parsing/detail_composite_parse.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/parsing/scoped_names.h"

namespace drake {
namespace multibody {
namespace parsing {

using std::make_unique;

namespace fs = drake::filesystem;
using drake::FindResourceOrThrow;
using drake::math::RigidTransformd;
using drake::multibody::FixedOffsetFrame;
using drake::multibody::Frame;
using drake::multibody::internal::CollisionFilterGroupResolver;
using drake::multibody::internal::CompositeParse;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::PackageMap;
using drake::multibody::Parser;
using drake::yaml::LoadYamlFile;

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

namespace {

// Add a new weld joint to @p plant from @p parent_frame as indicated by @p
// weld (as resolved relative to @p model_namespace) and update the @p info
// for the child model accordingly.
void AddWeld(
    const Frame<double>& parent_frame,
    const Frame<double>& child_frame,
    MultibodyPlant<double>* plant,
    std::vector<ModelInstanceInfo>* added_models) {
  plant->WeldFrames(parent_frame, child_frame);
  if (added_models) {
    // Record weld info into crappy ModelInstanceInfo struct.
    bool found = false;
    for (auto& info : *added_models) {
      if (info.model_instance == child_frame.model_instance()) {
        found = true;
        // See warning in ModelInstanceInfo about these members.
        info.parent_frame_name = parent_frame.name();
        info.child_frame_name = child_frame.name();
      }
    }
    DRAKE_DEMAND(found);
  }
}

void ProcessModelDirectivesImpl(
    const ModelDirectives& directives, MultibodyPlant<double>* plant,
    std::vector<ModelInstanceInfo>* added_models, Parser* parser,
    CompositeParse* composite,
    const std::string& model_namespace) {
  drake::log()->debug("ProcessModelDirectives(MultibodyPlant)");
  DRAKE_DEMAND(plant != nullptr);
  DRAKE_DEMAND(added_models != nullptr);
  // TODO(eric.cousineau): Somehow assert that our `parser` doesn't have a
  // different plant?
  DRAKE_DEMAND(parser != nullptr);
  auto get_scoped_frame = [&](const std::string& name) -> const Frame<double>& {
    // TODO(eric.cousineau): Simplify logic?
    if (name == "world") {
      return plant->world_frame();
    }
    return GetScopedFrameByName(*plant, PrefixName(model_namespace, name));
  };

  for (auto& directive : directives.directives) {
    if (directive.add_model) {
      ModelInstanceInfo info;
      auto& model = *directive.add_model;
      const std::string name = PrefixName(model_namespace, model.name);
      drake::log()->debug("  add_model: {}\n    {}", name, model.file);
      const std::string file =
          ResolveModelDirectiveUri(model.file, parser->package_map());
      drake::multibody::ModelInstanceIndex child_model_instance_id =
          composite->AddModelFromFile(file, name);
      info.model_instance = child_model_instance_id;
      info.model_name = name;
      info.model_path = file;
      if (added_models) added_models->push_back(info);

    } else if (directive.add_model_instance) {
      auto& instance = *directive.add_model_instance;
      const std::string name = PrefixName(model_namespace, instance.name);
      drake::log()->debug("  add_model_instance: {}", name);
      plant->AddModelInstance(name);

    } else if (directive.add_frame) {
      auto& frame = *directive.add_frame;
      drake::log()->debug("  add_frame: {}", frame.name);
      if (!frame.X_PF.base_frame) {
        // This would be caught elsewhere, but it is clearer to throw here.
        throw std::logic_error(
            "add_frame directive with empty base frame is ambiguous");
      }
      // Only override instance if scope is explicitly specified.
      std::optional<ModelInstanceIndex> instance;
      ScopedName parsed = ParseScopedName(frame.name);
      if (!parsed.instance_name.empty()) {
        parsed.instance_name = PrefixName(
            model_namespace, parsed.instance_name);
        instance = plant->GetModelInstanceByName(parsed.instance_name);
      }
      auto& added = plant->AddFrame(make_unique<FixedOffsetFrame<double>>(
          parsed.name, get_scoped_frame(*frame.X_PF.base_frame),
          frame.X_PF.GetDeterministicValue(), instance));
      const std::string resolved_name = PrefixName(
          GetInstanceScopeName(*plant, added.model_instance()), added.name());
      drake::log()->debug("    resolved_name: {}", resolved_name);

    } else if (directive.add_weld) {
      AddWeld(
          get_scoped_frame(directive.add_weld->parent),
          get_scoped_frame(directive.add_weld->child),
          plant, added_models);

    } else if (directive.add_collision_filter_group) {
      // Find the model instance index that corresponds to model_namespace, if
      // the name is non-empty.
      std::optional<ModelInstanceIndex> model_instance;
      if (!model_namespace.empty()) {
        DRAKE_DEMAND(plant->HasModelInstanceNamed(model_namespace));
        model_instance = plant->GetModelInstanceByName(model_namespace);
      }

      auto& resolver = composite->collision_resolver();
      auto& group = *directive.add_collision_filter_group;
      drake::log()->debug("  add_collision_filter_group: {}", group.name);
      std::set<std::string> member_set(group.members.begin(),
                                       group.members.end());
      // TODO(rpoyner-tri) obey parser policy? Improve error location clues?
      drake::internal::DiagnosticPolicy d;
      resolver.AddGroup(d, group.name, member_set, model_instance);
      for (const auto& ignored_group : group.ignored_collision_filter_groups) {
        resolver.AddPair(d, group.name, ignored_group, model_instance);
      }

    } else {
      // Recurse.
      auto& sub = *directive.add_directives;
      std::string new_model_namespace = PrefixName(
          model_namespace, sub.model_namespace.value_or(""));
      // Ensure we have a model instance for this namespace.
      drake::log()->debug("  add_directives: {}", sub.file);
      drake::log()->debug("    new_model_namespace: {}", new_model_namespace);
      if (!new_model_namespace.empty() &&
          !plant->HasModelInstanceNamed(new_model_namespace)) {
        throw std::runtime_error(fmt::format(
            "Namespace '{}' does not exist as model instance",
            new_model_namespace));
      }
      auto sub_directives =
          LoadModelDirectives(
              ResolveModelDirectiveUri(sub.file, parser->package_map()));
      ProcessModelDirectivesImpl(sub_directives, plant, added_models, parser,
                                 composite, new_model_namespace);
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
    drake::multibody::MultibodyPlant<double>* plant,
    std::vector<ModelInstanceInfo>* added_models,
    drake::multibody::Parser* parser) {
  auto tmp_parser = ConstructIfNullAndReassign<Parser>(&parser, plant);
  auto composite = CompositeParse::MakeCompositeParse(parser);
  auto tmp_added_model =
      ConstructIfNullAndReassign<std::vector<ModelInstanceInfo>>(&added_models);
  const std::string model_namespace = "";
  ProcessModelDirectivesImpl(directives, plant, added_models, parser,
                             composite.get(), model_namespace);
}

std::vector<ModelInstanceInfo> ProcessModelDirectives(
    const ModelDirectives& directives,
    drake::multibody::Parser* parser) {
  DRAKE_THROW_UNLESS(parser != nullptr);
  std::vector<ModelInstanceInfo> added_models;
  ProcessModelDirectives(directives, &parser->plant(), &added_models, parser);
  return added_models;
}

ModelDirectives LoadModelDirectives(const std::string& filename) {
  drake::log()->debug("LoadModelDirectives: {}", filename);

  if (!fs::exists({filename})) {
    throw std::runtime_error(fmt::format(
        "No such file {} during LoadModelDirectives", filename));
  }

  // Even though the 'defaults' we use to start parsing here are empty, by
  // providing any defaults at all, the effect during parsing will be that
  // any of the users' ModelDirective structs and sub-structs will _also_
  // start from their default values and allow for overwriting only a subset
  // of the data fields instead of requiring that the user provide them all.
  const ModelDirectives defaults;
  const auto directives = LoadYamlFile<ModelDirectives>(
      filename, std::nullopt /* child_name */, defaults);
  DRAKE_DEMAND(directives.IsValid());
  return directives;
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
