#include "drake/multibody/parsing/detail_dmd_parser.h"

#include <filesystem>
#include <memory>
#include <set>

#include "drake/common/text_logging.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/detail_make_model_name.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/scoped_name.h"

namespace drake {
namespace multibody {
namespace internal {

using parsing::GetScopedFrameByName;
using parsing::ModelDirectives;
using parsing::ModelInstanceInfo;

namespace {

// Adds a new weld joint to @p plant from @p parent_frame as indicated by @p
// weld (as resolved relative to @p model_namespace) and update the @p info for
// the child model accordingly.
void AddWeld(const Frame<double>& parent_frame,
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
        info.X_PC = X_PC;
      }
    }
    DRAKE_THROW_UNLESS(found);
  }
}

// If the given model contains a single body, returns its body frame.
// Otherwise, triggers a parse error.
const Frame<double>& GetUniqueBodyFrame(const ParsingWorkspace& workspace,
                                        ModelInstanceIndex model_instance) {
  const auto& plant = *workspace.plant;
  const auto& body_indices = plant.GetBodyIndices(model_instance);
  if (body_indices.size() == 1) {
    return plant.get_body(body_indices[0]).body_frame();
  }
  workspace.diagnostic.Error(fmt::format(
      "Model instance '{}' cannot use an empty frame name for its "
      "default_free_body_pose in add_model because it has {} (not 1) bodies.",
      plant.GetModelInstanceName(model_instance), body_indices.size()));
  return plant.world_frame();
}

// Implements the AddModel::default_free_body_pose logic (for one entry of that
// std::map).
// This handles both regular rigid bodies and deformable bodies. For deformable
// bodies, the pose can only be specified relative to the world frame, and it
// will use DeformableBody::set_default_pose to set the default pose. For rigid
// bodies, it will either set the default free body pose or add a quaternion
// floating joint depending on the parent frame and offset.
void ApplyAddModelDefaultFreeBodyPose(
    const ParsingWorkspace& workspace, ModelInstanceIndex model_instance,
    const Frame<double>& parent_frame, const std::string& child_name,
    const math::RigidTransform<double>& X_PC) {
  auto& plant = *workspace.plant;
  // First determine if the child is a deformable body.
  if (plant.deformable_model().HasBodyNamed(child_name, model_instance)) {
    const auto& deformable_body =
        plant.deformable_model().GetBodyByName(child_name, model_instance);
    if (&parent_frame != &plant.world_frame()) {
      workspace.diagnostic.Error(fmt::format(
          "Default pose for deformable body '{}' can only be specified "
          "relative to the world frame.",
          child_name));
      return;
    }
    // Set the default pose for the deformable body.
    plant.mutable_deformable_model()
        .GetMutableBody(deformable_body.body_id())
        .set_default_pose(X_PC);
    return;
  }
  // The body is a rigid body.
  const Frame<double>& child_frame =
      child_name.empty() ? GetUniqueBodyFrame(workspace, model_instance)
                         : plant.GetFrameByName(child_name, model_instance);
  const math::RigidTransform<double> child_offset =
      child_frame.GetFixedPoseInBodyFrame();
  if ((&parent_frame == &plant.world_frame()) &&
      child_offset.IsExactlyIdentity()) {
    // If the parent frame is the world and the child frame is coincident with
    // the body frame, then we can use the function to posture a body without
    // first adding a joint. (Note that this is a possible source of surprise
    // for users -- if they really did want the default pose to be taken with
    // respect to a child fixed offset frame that for now is the identity but
    // might later be re-postured by changing its parameter for the offset
    // transform after the fact, then using the body frame as child joint would
    // not suit their need. In that case, instead of using the sugar for
    // `default_free_body_pose`, the user can add whatever specific joint they
    // want with whatever frames and default posture they want, outside of
    // dmd. Our preference for using the body frame here is motivated by
    // backwards compatibility, so we could reconsider this decision if it
    // becomes a pain point.)
    plant.SetDefaultFloatingBaseBodyPose(child_frame.body(), X_PC);
  } else {
    // Add a floating joint so that we can set a default posture.
    // TODO(SeanCurtis-TRI): When the new multibody graph code lands,
    // update this code to test to see if there is already a joint between
    // the two bodies.
    // Note: the logic for generating the joint name is borrowed from
    // MultibodyTree::CreateJointImplementations().
    std::string joint_name = child_frame.body().name();
    while (plant.HasJointNamed(joint_name, model_instance)) {
      joint_name = "_" + joint_name;
    }
    const auto& joint =
        plant.AddJoint(std::make_unique<QuaternionFloatingJoint<double>>(
            joint_name, parent_frame, child_frame));
    plant.get_mutable_joint(joint.index()).SetDefaultPose(X_PC);
  }
}

void ParseModelDirectivesImpl(const ModelDirectives& directives,
                              const std::string& model_namespace,
                              const ParsingWorkspace& workspace,
                              std::vector<ModelInstanceInfo>* added_models) {
  drake::log()->debug("ParseModelDirectivesImpl(MultibodyPlant)");
  DRAKE_DEMAND(added_models != nullptr);
  auto& [options, package_map, diagnostic, builder, plant, scene_graph,
         collision_resolver, parser_selector] = workspace;
  DRAKE_DEMAND(plant != nullptr);
  auto get_scoped_frame = [_plant = plant, &model_namespace](
                              const std::string& name) -> const Frame<double>& {
    return GetScopedFrameByName(
        *_plant, DmdScopedNameJoin(model_namespace, name).to_string());
  };

  for (auto& directive : directives.directives) {
    if (directive.add_model) {
      ModelInstanceInfo info;
      auto& model = *directive.add_model;
      const std::string name =
          ScopedName::Join(model_namespace, model.name).to_string();
      drake::log()->debug("  add_model: {}\n    {}", name, model.file);
      const ResolveUriResult resolved =
          ResolveUri(diagnostic, model.file, package_map, {});
      if (!resolved.exists) {
        // ResolveUri already emitted an error message.
        continue;
      }
      const std::string file = resolved.full_path.string();
      std::optional<ModelInstanceIndex> child_model_instance_id =
          parser_selector(diagnostic, file)
              .AddModel({DataSource::kFilename, &file}, model.name,
                        model_namespace, workspace);
      if (!child_model_instance_id.has_value()) {
        // Error should have already been emitted.
        continue;
      }
      for (const auto& [joint_name, positions] :
           directive.add_model->default_joint_positions) {
        plant->GetMutableJointByName(joint_name, child_model_instance_id)
            .set_default_positions(positions);
      }
      for (const auto& [child_name, pose] :
           directive.add_model->default_free_body_pose) {
        const std::string parent_frame_name = pose.base_frame.value_or("world");
        const Frame<double>& parent_frame =
            (parent_frame_name == "world")
                ? plant->world_frame()
                : get_scoped_frame(parent_frame_name);
        const math::RigidTransform<double> X_PC = pose.GetDeterministicValue();
        ApplyAddModelDefaultFreeBodyPose(workspace, *child_model_instance_id,
                                         parent_frame, child_name, X_PC);
      }
      info.model_instance = *child_model_instance_id;
      info.model_name = name;
      info.model_path = file;
      if (added_models) added_models->push_back(info);

    } else if (directive.add_model_instance) {
      auto& instance = *directive.add_model_instance;
      const std::string name =
          MakeModelName(instance.name, model_namespace, workspace);
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
      ScopedName parsed = ScopedName::Parse(frame.name);
      if (!parsed.get_namespace().empty()) {
        // N.B. Here we're using Join on two namespaces to make a new namespace,
        // with no "element" involved. This is an atypical use of ScopedName,
        // but effective for our needs.
        parsed.set_namespace(
            ScopedName::Join(model_namespace, parsed.get_namespace())
                .get_full());
        instance = plant->GetModelInstanceByName(parsed.get_namespace());
      }
      auto& added = plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          std::string{parsed.get_element()},
          get_scoped_frame(*frame.X_PF.base_frame),
          frame.X_PF.GetDeterministicValue(), instance));
      const std::string resolved_name =
          ScopedName::Join(plant->GetModelInstanceName(added.model_instance()),
                           added.name())
              .to_string();
      drake::log()->debug("    resolved_name: {}", resolved_name);

    } else if (directive.add_weld) {
      math::RigidTransform<double> X_PC{};
      if (directive.add_weld->X_PC) {
        X_PC = directive.add_weld->X_PC->GetDeterministicValue();
      }
      AddWeld(get_scoped_frame(directive.add_weld->parent),
              get_scoped_frame(directive.add_weld->child), X_PC, plant,
              added_models);

    } else if (directive.add_collision_filter_group) {
      // If there's no geometry registered, there's nothing to be done with
      // collision filtering.  Trying to proceed will just trigger an error
      // eventually.
      if (!plant->geometry_source_is_registered()) {
        continue;
      }
      auto& group = *directive.add_collision_filter_group;

      // Find the model instance index that corresponds to model_namespace, if
      // the name is non-empty.
      std::optional<ModelInstanceIndex> model_instance;
      if (!model_namespace.empty()) {
        model_instance = plant->GetModelInstanceByName(model_namespace);
      } else if (group.model_namespace.has_value()) {
        model_instance = plant->GetModelInstanceByName(*group.model_namespace);
      }

      drake::log()->debug("  add_collision_filter_group: {}", group.name);
      std::set<std::string> member_set(group.members.begin(),
                                       group.members.end());
      std::set<std::string> member_groups_set(group.member_groups.begin(),
                                              group.member_groups.end());
      collision_resolver->AddGroup(diagnostic, group.name, member_set,
                                   member_groups_set, model_instance);
      for (const auto& ignored_group : group.ignored_collision_filter_groups) {
        collision_resolver->AddPair(diagnostic, group.name, ignored_group,
                                    model_instance);
      }

    } else {
      // Recurse.
      auto& sub = *directive.add_directives;
      std::string new_model_namespace =
          ScopedName::Join(model_namespace, sub.model_namespace.value_or(""))
              .to_string();
      // Ensure we have a model instance for this namespace.
      drake::log()->debug("  add_directives: {}", sub.file);
      drake::log()->debug("    new_model_namespace: {}", new_model_namespace);
      if (!new_model_namespace.empty() &&
          !plant->HasModelInstanceNamed(new_model_namespace)) {
        throw std::runtime_error(
            fmt::format("Namespace '{}' does not exist as model instance",
                        new_model_namespace));
      }
      const ResolveUriResult resolved =
          ResolveUri(diagnostic, sub.file, package_map, {});
      if (!resolved.exists) {
        // ResolveUri already emitted an error message.
        continue;
      }
      const std::string filename = resolved.full_path.string();
      auto sub_directives =
          LoadModelDirectives({DataSource::kFilename, &filename});
      ParseModelDirectivesImpl(sub_directives, new_model_namespace, workspace,
                               added_models);
    }
  }
}

}  // namespace

ScopedName DmdScopedNameJoin(const std::string& namespace_name,
                             const std::string& element_name) {
  if (element_name == "world") return ScopedName("", element_name);
  return ScopedName::Join(namespace_name, element_name);
}

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
  DRAKE_THROW_UNLESS(directives.IsValid());
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
    const std::optional<std::string>&, const ParsingWorkspace& workspace) {
  std::string display_source = data_source.IsFilename()
                                   ? data_source.GetAbsolutePath()
                                   : data_source.GetStem() + ".dmd.yaml";
  workspace.diagnostic.Error(fmt::format(
      "'{}' is a model directives data source; it is always an error to pass"
      " a model directives source to a single-model parser method. Use"
      " AddModels() instead.",
      display_source));
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
