#include "drake/multibody/parsing/detail_dmd_parser.h"

#include <memory>
#include <set>

#include "drake/common/filesystem.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/scoped_names.h"

namespace drake {
namespace multibody {
namespace internal {

namespace fs = drake::filesystem;

using drake::yaml::LoadYamlFile;
using parsing::ModelDirectives;
using parsing::ModelInstanceInfo;
using parsing::GetInstanceScopeName;
using parsing::GetScopedFrameByName;
using parsing::ParseScopedName;
using parsing::PrefixName;
using parsing::ScopedName;

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
    const ModelDirectives& directives,
    const std::string& model_namespace,
    const ParsingWorkspace& workspace,
    std::vector<ModelInstanceInfo>* added_models) {
  drake::log()->debug("ProcessModelDirectives(MultibodyPlant)");
  DRAKE_DEMAND(added_models != nullptr);
  auto& [package_map, diagnostic, plant,
         collision_resolver, parser_selector] = workspace;
  DRAKE_DEMAND(plant != nullptr);
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
          ResolveUri(diagnostic, model.file, package_map, {});
      std::optional<ModelInstanceIndex> child_model_instance_id =
          parser_selector(diagnostic, file)->AddModel(
              {DataSource::kFilename, &file},
              model.name, model_namespace, workspace);
      info.model_instance = *child_model_instance_id;
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
      auto& added = plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
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

      auto& group = *directive.add_collision_filter_group;
      drake::log()->debug("  add_collision_filter_group: {}", group.name);
      std::set<std::string> member_set(group.members.begin(),
                                       group.members.end());
      collision_resolver->AddGroup(
          diagnostic, group.name, member_set, model_instance);
      for (const auto& ignored_group : group.ignored_collision_filter_groups) {
        collision_resolver->AddPair(
            diagnostic, group.name, ignored_group, model_instance);
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
      auto sub_directives = LoadModelDirectives(
          ResolveUri(diagnostic, sub.file, package_map, {}));
      ProcessModelDirectivesImpl(sub_directives, new_model_namespace, workspace,
                                 added_models);
    }
  }
}

}  // namespace

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

std::vector<ModelInstanceInfo> ProcessModelDirectives(
    const ModelDirectives& directives,
    const std::optional<std::string>& scope_name,
    const ParsingWorkspace& workspace) {
  std::vector<ModelInstanceInfo> added_models;
  ProcessModelDirectivesImpl(directives, scope_name.value_or(""), workspace,
                             &added_models);
  return added_models;
}

DmdParser::DmdParser() {}
DmdParser::~DmdParser() {}
std::optional<ModelInstanceIndex> DmdParser::AddModel(
    const DataSource& data_source, const std::string&,
    const std::optional<std::string>& scope_name,
    const ParsingWorkspace& workspace) {
  // Error(model_name is ignored)
  auto indices = AddAllModels(data_source, scope_name, workspace);
  if (indices.empty()) { return {}; }
  // Error(too many indices; use AddAllModels instead).
  return indices.at(0);
}

std::vector<ModelInstanceIndex> DmdParser::AddAllModels(
    const DataSource& data_source,
    const std::optional<std::string>& scope_name,
    const ParsingWorkspace& workspace) {
  // TODO(rpoyner-tri): diagnostic policy?
  DRAKE_DEMAND(data_source.IsFilename());
  // TODO(rpoyner-tri): diagnostic policy?
  auto directives = LoadModelDirectives(data_source.filename());
  // TODO(rpoyner-tri): diagnostic policy?
  auto infos = ProcessModelDirectives(directives, scope_name, workspace);
  std::vector<ModelInstanceIndex> results(infos.size());
  for (const auto& info : infos) {
    results.push_back(info.model_instance);
  }
  return results;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

