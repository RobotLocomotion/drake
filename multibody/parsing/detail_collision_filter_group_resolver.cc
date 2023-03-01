#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"

#include "drake/common/unused.h"
#include "drake/multibody/tree/scoped_name.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticPolicy;
using geometry::GeometrySet;

CollisionFilterGroupResolver::CollisionFilterGroupResolver(
    MultibodyPlant<double>* plant)
    : plant_(plant) {
  DRAKE_DEMAND(plant != nullptr);
  minimum_model_instance_index_ =
      ModelInstanceIndex(plant->num_model_instances());

  // The scoped name for the world instance would require special handling,
  // but we never expect to be given a `model_instance` that refers to it.
  const ModelInstanceIndex world = plant->world_body().model_instance();
  DRAKE_DEMAND(minimum_model_instance_index_ > world);
}

CollisionFilterGroupResolver::~CollisionFilterGroupResolver() {}

void CollisionFilterGroupResolver::AddGroup(
    const DiagnosticPolicy& diagnostic,
    const std::string& group_name,
    const std::set<std::string>& body_names,
    std::optional<ModelInstanceIndex> model_instance) {
  if (model_instance) {
    DRAKE_DEMAND(*model_instance < plant_->num_model_instances());
    DRAKE_DEMAND(*model_instance >= minimum_model_instance_index_);
  }
  DRAKE_DEMAND(!group_name.empty());
  const std::string full_group_name = FullyQualify(group_name, model_instance);
  if (!ScopedName::Parse(group_name).get_namespace().empty()) {
    diagnostic.Error(fmt::format("group name '{}' cannot be a scoped name",
                                 full_group_name));
    return;
  }
  if (body_names.empty()) {
    diagnostic.Error(fmt::format("group '{}' has no members", full_group_name));
    return;
  }

  geometry::GeometrySet geometry_set;
  for (const auto& body_name : body_names) {
    DRAKE_DEMAND(!body_name.empty());
    const ScopedName scoped_body_name =
        ScopedName::Parse(FullyQualify(body_name, model_instance));

    const Body<double>* body{};
    if (plant_->HasModelInstanceNamed(scoped_body_name.get_namespace())) {
      ModelInstanceIndex body_model =
          plant_->GetModelInstanceByName(scoped_body_name.get_namespace());
      if (body_model < minimum_model_instance_index_) {
        diagnostic.Error(fmt::format("body name '{}' refers to a model outside"
                                     " the current parse", scoped_body_name));
        continue;
      }
      body = FindBody(scoped_body_name.get_element(), body_model);
    }
    if (!body) {
      diagnostic.Error(fmt::format("body with name '{}' not found",
                                   scoped_body_name));
      continue;
    }

    geometry_set.Add(plant_->GetBodyFrameIdOrThrow(body->index()));
  }
  groups_.insert({FullyQualify(group_name, model_instance), geometry_set});
}

void CollisionFilterGroupResolver::AddPair(
    const DiagnosticPolicy& diagnostic,
    const std::string& group_name_a,
    const std::string& group_name_b,
    std::optional<ModelInstanceIndex> model_instance) {
  unused(diagnostic);

  DRAKE_DEMAND(!group_name_a.empty());
  DRAKE_DEMAND(!group_name_b.empty());
  if (model_instance) {
    DRAKE_DEMAND(*model_instance < plant_->num_model_instances());
    DRAKE_DEMAND(*model_instance >= minimum_model_instance_index_);
  }

  // Store group pairs by fully qualified name. The groups don't need to
  // actually be defined until Resolve() time.
  const std::string name_a = FullyQualify(group_name_a, model_instance);
  const std::string name_b = FullyQualify(group_name_b, model_instance);

  // These two group names are allowed to be identical, which means the
  // bodies inside this collision filter group should be collision excluded
  // among each other.
  pairs_.insert({name_a, name_b});
}

void CollisionFilterGroupResolver::Resolve(const DiagnosticPolicy& diagnostic) {
  DRAKE_DEMAND(!is_resolved_);
  is_resolved_ = true;

  for (const auto& [name_a, name_b] : pairs_) {
    const GeometrySet* set_a = FindGroup(diagnostic, name_a);
    const GeometrySet* set_b = FindGroup(diagnostic, name_b);
    if (set_a == nullptr || set_b == nullptr) {
      continue;
    }
    plant_->ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
        {name_a, *set_a}, {name_b, *set_b});
  }
}

std::string CollisionFilterGroupResolver::FullyQualify(
    const std::string& name,
    std::optional<ModelInstanceIndex> model_instance) const {
  if (!model_instance) {
    // Names found in global scope are just themselves.
    return name;
  }
  const std::string& model_name = plant_->GetModelInstanceName(*model_instance);
  return ScopedName::Join(model_name, name).to_string();
}

const GeometrySet* CollisionFilterGroupResolver::FindGroup(
    const DiagnosticPolicy& diagnostic, const std::string& group_name) const {
  auto iter = groups_.find(group_name);
  if (iter == groups_.end()) {
    diagnostic.Error(
        fmt::format("collision filter group with name '{}' not found",
                    group_name));
    return nullptr;
  }
  return &iter->second;
}

const Body<double>* CollisionFilterGroupResolver::FindBody(
    std::string_view name,
    ModelInstanceIndex model_instance) {
  if (plant_->HasBodyNamed(name, model_instance)) {
    return &plant_->GetBodyByName(name, model_instance);
  }
  return {};
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
