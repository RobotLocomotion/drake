#include "drake/multibody/parsing/detail_dmd_parser.h"

#include <filesystem>
#include <memory>
#include <set>

#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/scoped_names.h"

namespace drake {
namespace multibody {
namespace internal {

using parsing::ModelDirectives;
using parsing::ModelInstanceInfo;
using parsing::GetInstanceScopeName;
using parsing::GetScopedFrameByName;
using parsing::ParseScopedName;
using parsing::PrefixName;
using parsing::ScopedName;

namespace {

// Adds a new weld joint to @p plant from @p parent_frame as indicated by @p
// weld (as resolved relative to @p model_namespace) and update the @p info for
// the child model accordingly.
void AddWeld(
    const Frame<double>& parent_frame,
    const Frame<double>& child_frame,
    const math::RigidTransform<double>& X_PC,
    MultibodyPlant<double>* plant,
    std::vector<ModelInstanceInfo>* added_models) {
  plant->WeldFrames(parent_frame, child_frame, X_PC);
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

void ParseModelDirectivesImpl(
    const ModelDirectives& directives,
    const std::string& model_namespace,
    const ParsingWorkspace& workspace,
    std::vector<ModelInstanceInfo>* added_models) {
  drake::log()->debug("ParseModelDirectivesImpl(MultibodyPlant)");
  DRAKE_DEMAND(added_models != nullptr);
  auto& [package_map, diagnostic, plant,
         collision_resolver, parser_selector] = workspace;
  DRAKE_DEMAND(plant != nullptr);
  auto get_scoped_frame = [plant = plant, &model_namespace](
                              const std::string& name) -> const Frame<double>& {
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
          parser_selector(diagnostic, file).AddModel(
              {DataSource::kFilename, &file},
              model.name, model_namespace, workspace);
      if (!child_model_instance_id.has_value()) {
        // Error should have already been emitted.
        continue;
      }
      for (const auto& [joint_name, positions] :
               directive.add_model->default_joint_positions) {
        plant->GetMutableJointByName(joint_name, child_model_instance_id)
            .set_default_positions(positions);
      }
      for (const auto& [body_name, pose] :
               directive.add_model->default_free_body_pose) {
        plant->SetDefaultFreeBodyPose(
            plant->GetBodyByName(body_name, *child_model_instance_id),
            pose.GetDeterministicValue());
      }
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
      math::RigidTransform<double> X_PC{};
      if (directive.add_weld->X_PC) {
        X_PC = directive.add_weld->X_PC->GetDeterministicValue();
      }
      AddWeld(
          get_scoped_frame(directive.add_weld->parent),
          get_scoped_frame(directive.add_weld->child),
          X_PC, plant, added_models);

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
      std::string filename = ResolveUri(diagnostic, sub.file, package_map, {});
      auto sub_directives =
          LoadModelDirectives({DataSource::kFilename, &filename});
      ParseModelDirectivesImpl(sub_directives, new_model_namespace, workspace,
                               added_models);
    }
  }
}

}  // namespace

ModelDirectives LoadModelDirectives(const DataSource& data_source) {
  // Even though the 'defaults' we use to start parsing here are empty, by
  // providing any defaults at all, the effect during parsing will be that any
  // of the users' ModelDirective structs and sub-structs will _also_ start
  // from their default values and allow for overwriting only a subset of the
  // data fields instead of requiring that the user provide them all.
  const ModelDirectives defaults;

  ModelDirectives directives;
  if (data_source.IsFilename()) {
    const std::string& filename = data_source.filename();
    drake::log()->debug("LoadModelDirectives: {}", filename);

    if (!std::filesystem::exists({filename})) {
      throw std::runtime_error(
          fmt::format("No such file {} during LoadModelDirectives", filename));
    }

    directives = yaml::LoadYamlFile<ModelDirectives>(
        filename, std::nullopt /* child_name */, defaults);
  } else {
    DRAKE_DEMAND(data_source.IsContents());
    directives = yaml::LoadYamlString<ModelDirectives>(
        data_source.contents(), std::nullopt /* child_name */, defaults);
  }
  DRAKE_DEMAND(directives.IsValid());
  return directives;
}

std::vector<ModelInstanceInfo> ParseModelDirectives(
    const ModelDirectives& directives,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  std::vector<ModelInstanceInfo> added_models;
  ParseModelDirectivesImpl(directives, parent_model_name.value_or(""),
                           workspace, &added_models);
  return added_models;
}

DmdParserWrapper::DmdParserWrapper() {}

DmdParserWrapper::~DmdParserWrapper() {}

std::optional<ModelInstanceIndex> DmdParserWrapper::AddModel(
    const DataSource& data_source, const std::string&,
    const std::optional<std::string>&,
    const ParsingWorkspace& workspace) {
  std::string display_source =
      data_source.IsFilename()
      ? data_source.GetAbsolutePath()
      : data_source.GetStem() + ".dmd.yaml";
  workspace.diagnostic.Error(fmt::format(
      "'{}' is a model directives data source; it is always an error to pass"
      " a model directives source to a single-model parser method. Use"
      " AddModels() instead.", display_source));
  return {};
}

std::vector<ModelInstanceIndex> DmdParserWrapper::AddAllModels(
    const DataSource& data_source,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  // TODO(#18052): diagnostic policy?
  ModelDirectives directives = LoadModelDirectives(data_source);
  // TODO(#18052): diagnostic policy?
  const std::vector<ModelInstanceInfo> infos =
      ParseModelDirectives(directives, parent_model_name, workspace);
  std::vector<ModelInstanceIndex> results;
  results.reserve(infos.size());
  for (const auto& info : infos) {
    results.push_back(info.model_instance);
  }
  return results;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

