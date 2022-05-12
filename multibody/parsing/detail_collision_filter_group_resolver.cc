#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"

#include "drake/multibody/parsing/scoped_names.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticPolicy;
using parsing::GetInstanceScopeName;
using parsing::ParseScopedName;
using parsing::PrefixName;
using parsing::ScopedName;

CollisionFilterGroupResolver::CollisionFilterGroupResolver(
    MultibodyPlant<double>* plant) : plant_(plant) {
  DRAKE_DEMAND(plant != nullptr);
}

void CollisionFilterGroupResolver::AddGroup(
    const DiagnosticPolicy& diagnostic,
    const std::string& group_name,
    std::set<std::string> body_names,
    std::optional<ModelInstanceIndex> model_instance) {
  if (model_instance) {
    DRAKE_DEMAND(*model_instance < plant_->num_model_instances());
  }
  DRAKE_DEMAND(!group_name.empty());
  if (!CheckLegalName(diagnostic, group_name, "group name")) { return; }
  ScopedName scoped_group_name = ParseScopedName(group_name);
  if (!scoped_group_name.instance_name.empty()) {
    diagnostic.Error(fmt::format("group name '{}' cannot be a scoped name",
                                 FullyQualify(group_name, model_instance)));
    return;
  }
  if (body_names.empty()) {
    diagnostic.Error(fmt::format("group '{}' has no members",
                                 FullyQualify(group_name, model_instance)));
    return;
  }

  geometry::GeometrySet geometry_set;
  for (const auto& body_name : body_names) {
    DRAKE_DEMAND(!body_name.empty());
    if (!CheckLegalName(diagnostic, body_name, "body name")) { continue; }
    const std::string qualified = FullyQualify(body_name, model_instance);
    ScopedName scoped_name = ParseScopedName(qualified);

    std::optional<ModelInstanceIndex> body_model;
    if (scoped_name.instance_name.empty()) {
      body_model = model_instance;
    } else if (plant_->HasModelInstanceNamed(scoped_name.instance_name)) {
      body_model = plant_->GetModelInstanceByName(scoped_name.instance_name);
    }

    const Body<double>* body = FindBody(scoped_name.name, body_model);
    if (!body) {
      diagnostic.Error(fmt::format("body with name '{}' not found", qualified));
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
  DRAKE_DEMAND(!group_name_a.empty());
  DRAKE_DEMAND(!group_name_b.empty());
  if (model_instance) {
    DRAKE_DEMAND(*model_instance < plant_->num_model_instances());
  }
  bool a_ok = CheckLegalName(diagnostic, group_name_a, "group name");
  bool b_ok = CheckLegalName(diagnostic, group_name_b, "group name");
  if (!(a_ok && b_ok)) {
    return;
  }

  // Store group pairs by fully qualified name. The groups don't need to
  // actually be defined until Resolve() time.
  const std::string name_a = FullyQualify(group_name_a, model_instance);
  const std::string name_b = FullyQualify(group_name_b, model_instance);

  // These two group names are allowed to be identical, which means the
  // bodies inside this collision filter group should be collision excluded
  // among each other.
  pairs_.insert({name_a.c_str(), name_b.c_str()});
}

void CollisionFilterGroupResolver::Resolve(const DiagnosticPolicy& diagnostic) {
  for (const auto& [name_a, name_b] : pairs_) {
    bool a_ok = IsGroupDefined(diagnostic, name_a);
    bool b_ok = IsGroupDefined(diagnostic, name_b);
    if (!(a_ok && b_ok)) {
      continue;
    }
    plant_->ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
        {name_a, groups_.find(name_a)->second},
        {name_b, groups_.find(name_b)->second});
  }
  groups_.clear();
  pairs_.clear();
}

bool CollisionFilterGroupResolver::CheckLegalName(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::string& name,
    const std::string& description) const {
  using parsing::internal::kScopedNameDelim;
  // The main objective here is to avoid aborting with a dubious message in
  // ParseScope(). There are numerous other degenerate name cases, but those
  // likely won't abort in ParseScope() and will be caught later on by full
  // qualification and lookup.
  bool legal = (name[0] != kScopedNameDelim[0]) &&
               (name[name.size() - 1] !=
                kScopedNameDelim[std::string(kScopedNameDelim).size() - 1]);
  if (!legal) {
    diagnostic.Error(fmt::format("{} '{}' can neither begin nor end with '{}'",
                                 description, name, kScopedNameDelim));
  }
  return legal;
}

std::string CollisionFilterGroupResolver::FullyQualify(
    const std::string& name,
    std::optional<ModelInstanceIndex> model_instance) const {
  if (!model_instance) {
    // Names found in global scope are just themselves.
    return name;
  }

  // Treat the name as relative and just prefix it with the model instance
  // name. Misuses of scoping will fail in lookup.
  return PrefixName(GetInstanceScopeName(*plant_, *model_instance), name);
}

bool CollisionFilterGroupResolver::IsGroupDefined(
    const DiagnosticPolicy& diagnostic, const std::string& group_name) const {
  bool defined = groups_.count(group_name) > 0;
  if (!defined) {
    diagnostic.Error(
        fmt::format("collision filter group with name '{}' not found",
                    group_name));
  }
  return defined;
}

const Body<double>* CollisionFilterGroupResolver::FindBody(
    const std::string& name,
    std::optional<ModelInstanceIndex> model_instance) {
  ModelInstanceIndex model = model_instance.value_or(default_model_instance());
  if (plant_->HasBodyNamed(name, model)) {
    return &plant_->GetBodyByName(name, model);
  }
  return {};
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
