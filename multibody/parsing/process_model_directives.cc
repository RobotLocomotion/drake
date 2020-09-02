#include "drake/multibody/parsing/dev/process_model_directives.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/schema/transform.h"
#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace multibody {
namespace parsing {

using std::make_unique;
using Eigen::Isometry3d;

namespace fs = drake::filesystem;
using drake::FindResourceOrThrow;
using drake::math::RigidTransformd;
using drake::multibody::FixedOffsetFrame;
using drake::multibody::Frame;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::PackageMap;
using drake::multibody::Parser;
using drake::yaml::YamlReadArchive;

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

namespace internal {

ScopedName ParseScopedName(const std::string& full_name) {
  const std::string delim = "::";
  size_t pos = full_name.rfind(delim);
  ScopedName result;
  if (pos == std::string::npos) {
    result.name = full_name;
  } else {
    result.instance_name = full_name.substr(0, pos);
    // "Global scope" (e.g. "::my_frame") not supported.
    DRAKE_DEMAND(!result.instance_name.empty());
    result.name = full_name.substr(pos + delim.size());
  }
  return result;
}

std::string PrefixName(const std::string& namespace_, const std::string& name) {
  if (namespace_.empty())
    return name;
  else if (name.empty())
    return namespace_;
  else
    return namespace_ + "::" + name;
}

std::string GetInstanceScopeName(
    const MultibodyPlant<double>& plant,
    ModelInstanceIndex instance) {
  if (instance != plant.world_body().model_instance()) {
    return plant.GetModelInstanceName(instance);
  } else {
    return "";
  }
}

// Add a new weld joint to @p plant from @p parent_frame as indicated by @p
// weld (as resolved relative to @p model_namespace) and update the @p info
// for the child model accordingly.
//
// If @p error_func is provided (non-empty), use it to compute an offset X_PCe
// for the weld.
void AddWeldWithOptionalError(
    const Frame<double>& parent_frame,
    const Frame<double>& child_frame,
    ModelWeldErrorFunction error_func,
    MultibodyPlant<double>* plant,
    std::vector<ModelInfo>* added_models) {
  // TODO(eric.cousineau): This hack really shouldn't belong in model
  // directives. Instead, it should live externally as a transformation on
  // ModelDirectives (either flattened or recursive).
  std::optional<RigidTransformd> X_PCe =
      error_func ? error_func(parent_frame, child_frame) : std::nullopt;
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
    // Record weld info into crappy ModelInfo struct.
    bool found = false;
    for (auto& info : *added_models) {
      if (info.model_instance == child_frame.model_instance()) {
        found = true;
        // See warning in ModelInfo about these members.
        info.parent_frame_name = parent_frame.name();
        info.child_frame_name = child_frame.name();
      }
    }
    DRAKE_DEMAND(found);
  }
}

void ProcessModelDirectivesImpl(
    const ModelDirectives& directives, MultibodyPlant<double>* plant,
    std::vector<ModelInfo>* added_models, Parser* parser,
    const std::string& model_namespace,
    ModelWeldErrorFunction error_func) {
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

  // Add `package://jaco_description` URIs even though that package lacks a
  // `package.xml` file.
  // TODO(ggould) ...which is bad and wrong.
  if (!parser->package_map().Contains("jaco_description")) {
    const fs::path path_inside_jaco = drake::FindResourceOrThrow(
        "drake/manipulation/models/jaco_description/LICENSE.TXT");
    parser->package_map().Add("jaco_description",
                              path_inside_jaco.parent_path().string());
  }

  for (auto& directive : directives.directives) {
    if (directive.add_model) {
      ModelInfo info;
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

    } else if (directive.add_package_path) {
      auto& package_path = *directive.add_package_path;
      drake::log()->debug("  add_package_path: {}", package_path.name);
      const fs::path abspath_xml =
          drake::FindResourceOrThrow(package_path.path + "/package.xml");
      const std::string path = abspath_xml.parent_path().string();

      // It is possible to get the same `add_package_path` directive twice,
      // e.g. by including the same directives yaml twice.  That's fine so
      // long as they agree.
      if (parser->package_map().Contains(package_path.name)) {
        DRAKE_DEMAND(parser->package_map().GetPath(package_path.name) == path);
      } else {
        parser->package_map().Add(package_path.name, path);
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
      ProcessModelDirectivesImpl(
          sub_directives, plant, added_models, parser, new_model_namespace,
          error_func);
    }
  }
}

}  // namespace internal

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

const drake::multibody::Frame<double>*
GetScopedFrameByNameMaybe(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& full_name) {
  if (full_name == "world")
    return &plant.world_frame();
  auto result = internal::ParseScopedName(full_name);
  if (!result.instance_name.empty()) {
    auto instance = plant.GetModelInstanceByName(result.instance_name);
    if (plant.HasFrameNamed(result.name, instance)) {
      return &plant.GetFrameByName(result.name, instance);
    }
  } else if (plant.HasFrameNamed(result.name)) {
    return &plant.GetFrameByName(result.name);
  }
  return nullptr;
}

const std::string GetScopedFrameName(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::multibody::Frame<double>& frame) {
  if (&frame == &plant.world_frame())
    return "world";
  return internal::PrefixName(internal::GetInstanceScopeName(
      plant, frame.model_instance()), frame.name());
}

void ProcessModelDirectives(
    const ModelDirectives& directives, MultibodyPlant<double>* plant,
    std::vector<ModelInfo>* added_models, Parser* parser,
    ModelWeldErrorFunction error_func) {
  auto tmp_parser = ConstructIfNullAndReassign<Parser>(&parser, plant);
  auto tmp_added_model =
      ConstructIfNullAndReassign<std::vector<ModelInfo>>(&added_models);
  const std::string model_namespace = "";
  internal::ProcessModelDirectivesImpl(
      directives, plant, added_models, parser, model_namespace, error_func);
}

ModelDirectives LoadModelDirectives(const std::string& filename) {
  drake::log()->debug("LoadModelDirectives: {}", filename);

  // TODO(ggould-tri) This should use the YamlLoadWithDefaults mechanism
  // instead once that is ported to drake.
  ModelDirectives directives;
  YAML::Node root = YAML::LoadFile(filename);
  drake::yaml::YamlReadArchive::Options options;
  options.allow_cpp_with_no_yaml = true;
  drake::yaml::YamlReadArchive(root, options).Accept(&directives);

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

ModelInfo MakeModelInfo(const std::string& model_name,
                        const std::string& model_path,
                        const std::string& parent_frame_name,
                        const std::string& child_frame_name,
                        const drake::math::RigidTransformd& X_PC) {
  ModelInfo info;
  info.model_name = model_name;
  info.model_path = model_path;
  info.parent_frame_name = parent_frame_name;
  info.child_frame_name = child_frame_name;
  info.X_PC = X_PC;
  return info;
}

// TODO(eric.cousineau): Do we *really* need this function? This seems like
// it'd be better handled as an MBP subgraph.
ModelDirectives MakeModelsAttachedToFrameDirectives(
    const std::vector<ModelInfo>& models_to_add) {
  ModelDirectives directives;

  // One for add frame, one for add model, one for add weld.
  directives.directives.resize(models_to_add.size() * 3);

  int index = 0;
  for (size_t i = 0; i < models_to_add.size(); i++) {
    const ModelInfo& model_to_add = models_to_add.at(i);
    std::string attachment_frame_name = model_to_add.parent_frame_name;

    // Add frame first.
    if (!model_to_add.X_PC.IsExactlyIdentity()) {
      AddFrame frame_dir;
      attachment_frame_name = model_to_add.model_name + "_attachment_frame";
      frame_dir.name = attachment_frame_name;
      frame_dir.X_PF.base_frame = model_to_add.parent_frame_name;
      frame_dir.X_PF.translation =
          drake::Vector<double, 3>(model_to_add.X_PC.translation());
      frame_dir.X_PF.rotation =
          drake::schema::Rotation{model_to_add.X_PC.rotation()};

      directives.directives.at(index++).add_frame = frame_dir;
    }

    // Add model
    AddModel model_dir;
    model_dir.file = model_to_add.model_path;
    model_dir.name = model_to_add.model_name;

    directives.directives.at(index++).add_model = model_dir;

    AddWeld weld_dir;
    weld_dir.parent = attachment_frame_name;
    weld_dir.child =
        model_to_add.model_name + "::" + model_to_add.child_frame_name;
    directives.directives.at(index++).add_weld = weld_dir;
  }
  directives.directives.resize(index);

  return directives;
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
