#include "drake/multibody/parsing/process_model_directives.h"

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/schema/transform.h"
#include "drake/common/yaml/yaml_io.h"
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
//
// If @p error_func is provided (non-empty), use it to compute an offset X_PCe
// for the weld.
void AddWeldWithOptionalError(
    const Frame<double>& parent_frame,
    const Frame<double>& child_frame,
    ModelWeldErrorFunctionToBeDeprecated error_func,
    MultibodyPlant<double>* plant,
    std::vector<ModelInstanceInfo>* added_models) {
  // TODO(#14084): This hack really shouldn't belong in model
  // directives. Instead, it should live externally as a transformation on
  // ModelDirectives (either flattened or recursive).
  std::string parent_full_name =
      PrefixName(GetInstanceScopeName(*plant, parent_frame.model_instance()),
                 parent_frame.name());
  std::string child_full_name =
      PrefixName(GetInstanceScopeName(*plant, child_frame.model_instance()),
                 child_frame.name());
  std::optional<RigidTransformd> X_PCe =
      error_func ? error_func(parent_full_name, child_full_name) : std::nullopt;
  if (X_PCe.has_value()) {
    // N.B. Since this will belong in the child_frame's model instance, we
    // shouldn't worry about name collisions.
    const std::string weld_error_name =
        parent_frame.name() + "_weld_error_to_" + child_frame.name();
    drake::log()->debug("ProcessAddWeld adding error frame {}",
                        weld_error_name);
    const auto& error_frame =
        plant->AddFrame(make_unique<FixedOffsetFrame<double>>(
            weld_error_name, parent_frame, *X_PCe,
            child_frame.model_instance()));
    plant->WeldFrames(error_frame, child_frame);
  } else {
    plant->WeldFrames(parent_frame, child_frame);
  }
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
    const std::string& model_namespace,
    ModelWeldErrorFunctionToBeDeprecated error_func) {
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
          parser->AddModelFromFile(file, name);
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
      AddWeldWithOptionalError(
          get_scoped_frame(directive.add_weld->parent),
          get_scoped_frame(directive.add_weld->child),
          error_func, plant, added_models);

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
      ProcessModelDirectivesImpl(
          sub_directives, plant, added_models, parser, new_model_namespace,
          error_func);
    }
  }
}

}  // namespace

std::string ResolveModelDirectiveUri(const std::string& uri,
                                     const PackageMap& package_map) {
  const std::string scheme_separator = "://";
  const size_t scheme_end = uri.find(scheme_separator);
  if (scheme_end == std::string::npos) {
    drake::log()->error("Model resource '{}' is not a valid URI.",
                        uri);
    std::abort();
  }

  const std::string scheme = uri.substr(0, scheme_end);
  const size_t package_end = uri.find("/", scheme_end + 3);
  if (package_end == std::string::npos) {
    drake::log()->error("Model resource '{}' has no path in package.",
                        uri);
    std::abort();
  }

  const std::string package_name =
      uri.substr(scheme_end + scheme_separator.size(),
                 package_end - scheme_end - scheme_separator.size());
  const std::string path_in_package = uri.substr(package_end + 1);

  DRAKE_DEMAND(scheme == "package");  // No other schemes supported for now.
  if (!package_map.Contains(package_name)) {
    drake::log()->error(
        "Unable to resolve package '{}' for URI '{}' using package map: '{}'",
        package_name, uri, package_map);
    std::abort();
  }
  return package_map.GetPath(package_name) + "/" + path_in_package;
}

void ProcessModelDirectives(
    const ModelDirectives& directives,
    drake::multibody::MultibodyPlant<double>* plant,
    std::vector<ModelInstanceInfo>* added_models,
    drake::multibody::Parser* parser) {
  // Once 2022-02-01 comes around, we can fold these two overloads back down
  // to just one function.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  ProcessModelDirectives(directives, plant, added_models, parser, {});
#pragma GCC diagnostic pop
}

void ProcessModelDirectives(
    const ModelDirectives& directives, MultibodyPlant<double>* plant,
    std::vector<ModelInstanceInfo>* added_models, Parser* parser,
    ModelWeldErrorFunctionToBeDeprecated error_func) {
  auto tmp_parser = ConstructIfNullAndReassign<Parser>(&parser, plant);
  auto tmp_added_model =
      ConstructIfNullAndReassign<std::vector<ModelInstanceInfo>>(&added_models);
  const std::string model_namespace = "";
  ProcessModelDirectivesImpl(
      directives, plant, added_models, parser, model_namespace, error_func);
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
