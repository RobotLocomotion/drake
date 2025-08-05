#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"

#include <unordered_set>
#include <vector>

#include "drake/common/unused.h"
#include "drake/multibody/parsing/detail_strongly_connected_components.h"
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
    const DiagnosticPolicy& diagnostic, const std::string& group_name,
    const std::set<std::string>& body_names,
    const std::set<std::string>& member_group_names,
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
  if (groups_.contains(full_group_name)) {
    diagnostic.Error(
        fmt::format("group '{}' has already been defined", full_group_name));
    return;
  }
  if (body_names.empty() && member_group_names.empty()) {
    diagnostic.Error(fmt::format("group '{}' has no members", full_group_name));
    return;
  }

  geometry::GeometrySet geometry_set;
  std::set<std::string> full_names;
  for (const auto& body_name : body_names) {
    DRAKE_DEMAND(!body_name.empty());
    const ScopedName scoped_body_name =
        ScopedName::Parse(FullyQualify(body_name, model_instance));

    // The body name may refer to a rigid body or a deformable body. If the same
    // scoped name exists as rigid and deformable in the model, then this code
    // simply selects the rigid body.
    const RigidBody<double>* body{};
    const DeformableBody<double>* deformable_body{};
    if (plant_->HasModelInstanceNamed(scoped_body_name.get_namespace())) {
      ModelInstanceIndex body_model =
          plant_->GetModelInstanceByName(scoped_body_name.get_namespace());
      if (body_model < minimum_model_instance_index_) {
        diagnostic.Error(fmt::format(
            "body name '{}' refers to a model outside the current parse",
            scoped_body_name));
        continue;
      }
      body = FindBody(scoped_body_name.get_element(), body_model);
      deformable_body = FindDeformableBody(
          std::string(scoped_body_name.get_element()), body_model);
    }

    if (body && deformable_body) {
      diagnostic.Error(
          fmt::format("body name '{}' is ambiguous, refers to both a rigid and "
                      "deformable body",
                      scoped_body_name));
      continue;
    } else if (!body && !deformable_body) {
      diagnostic.Error(
          fmt::format("body with name '{}' not found", scoped_body_name));
      continue;
    }

    full_names.insert(std::string(scoped_body_name.get_full()));
    if (body) {
      geometry_set.Add(plant_->GetBodyFrameIdOrThrow(body->index()));
    } else if (deformable_body) {
      geometry_set.Add(deformable_body->geometry_id());
    }
  }
  groups_.insert({full_group_name, {full_names, geometry_set}});

  // Group insertions get computed at resolution time. Here we build the
  // directed graph of insertions, where edges point from the source group (the
  // group whose members we should insert) to the destination group (the group
  // into which we should insert members).
  //
  // Note that all of the successors in this graph (destinations) are names
  // that we know to exist, because they have been built by this function. The
  // keys of the graph map are names that need to be checked at resolution
  // time.
  for (const auto& insertion_group : member_group_names) {
    const auto full_insertion_group{
        FullyQualify(insertion_group, model_instance)};
    if (!group_insertion_graph_.contains(full_insertion_group)) {
      group_insertion_graph_[full_insertion_group] = {};
    }
    group_insertion_graph_[full_insertion_group].insert(full_group_name);
  }
}

void CollisionFilterGroupResolver::AddPair(
    const DiagnosticPolicy& diagnostic, const std::string& group_name_a,
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

CollisionFilterGroupsImpl<InstancedName> CollisionFilterGroupResolver::Resolve(
    const DiagnosticPolicy& diagnostic) {
  DRAKE_DEMAND(!is_resolved_);
  is_resolved_ = true;

  // TODO(rpoyner-tri): The source-file locality for naming errors discovered
  // here is not great. Consider some scheme for remembering the source code
  // line (bundled in `diagnostic` passed to AddGroup/AddPair/etc.) together
  // with the name to be checked.

  // Resolve whole-group insertions, which requires analyzing the insertion
  // graph and editing the map of groups.

  // TODO(rpoyner-tri): what follows is a lot of graph math, based on strings
  // for node labels; consider swapping strings for purpose-built integer index
  // labels if this turns out to be too slow.

  // First check the insertion source group names, emit errors and remove any
  // broken ones. Check names in order so that the order of error messages is
  // stable.
  std::set<std::string> ordered_names;
  std::vector<std::string> bad_names;
  for (const auto& [name, member_groups] : group_insertion_graph_) {
    ordered_names.insert(name);
  }
  for (const auto& name : ordered_names) {
    if (!FindGroup(diagnostic, name)) {
      bad_names.push_back(name);
    }
  }
  for (const auto& name : bad_names) {
    group_insertion_graph_.erase(name);
  }

  // It is perfectly well-defined and useful to allow nested group insertions:
  // a=>b=>c. However, if we naively execute b=>c, then a=>c, we have failed to
  // do the transitive insertion a=>c.  In order to efficiently and correctly
  // execute the insertions, we need to have a topological sort of the
  // insertion graph. It is possible (if maybe unlikely) that the graph could
  // have cycles, or more generally, strongly connected components (SCCs). This
  // is fine; the result of such a graph is that all groups in an SCC receive
  // the union of all the groups' members. Happily, there is a linear-time
  // algorithm to construct both the topo-sort and find the SCCs.
  //
  // Compute the sequence of strongly connected components, in reverse of
  // insertion order.
  StronglyConnectedComponents<std::string> sccs =
      FindStronglyConnectedComponents(group_insertion_graph_);

  // Execute the insertions.
  for (auto it = sccs.rbegin(); it != sccs.rend(); ++it) {
    const auto& scc = *it;
    // The content to insert is the union of the SCC's members' contents.
    GroupData contents_union;
    for (const auto& node : scc) {
      contents_union.geometries.Add(groups_[node].geometries);
      const auto& names = groups_[node].body_names;
      contents_union.body_names.insert(names.begin(), names.end());
    }

    // The destinations are the the union of the SCC's' members' successors,
    // plus the membership of the SCC itself.
    std::unordered_set<std::string> destinations{scc};
    for (const auto& node : scc) {
      const auto& successors = group_insertion_graph_[node];
      destinations.insert(successors.begin(), successors.end());
    }

    // Do the insertions.
    for (const auto& destination : destinations) {
      groups_[destination].geometries.Add(contents_union.geometries);
      const auto& names = contents_union.body_names;
      groups_[destination].body_names.insert(names.begin(), names.end());
    }
  }

  // Now that the groups are complete, evaluate the pairs into plant
  // rules. Remove any pairs with broken names.
  std::set<SortedPair<std::string>> bad_pairs;
  for (const auto& pair : pairs_) {
    const auto& [name_a, name_b] = pair;
    const GroupData* set_a = FindGroup(diagnostic, name_a);
    const GroupData* set_b = FindGroup(diagnostic, name_b);
    if (set_a == nullptr || set_b == nullptr) {
      bad_pairs.insert(pair);
      continue;
    }
    plant_->ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
        {name_a, set_a->geometries}, {name_b, set_b->geometries});
  }
  for (const auto& pair : bad_pairs) {
    pairs_.erase(pair);
  }

  // Return the info, so the Parser can report back to users.
  CollisionFilterGroupsImpl<std::string> result;
  for (const auto& [name, members] : groups_) {
    result.AddGroup(name, members.body_names);
  }
  for (const auto& pair : pairs_) {
    result.AddExclusionPair(pair);
  }
  return result.template Convert<InstancedName>(
      [this](const std::string& input) {
        InstancedName instanced_name;
        auto scoped = ScopedName::Parse(input);
        if (plant_->HasModelInstanceNamed(scoped.get_namespace())) {
          instanced_name.index =
              plant_->GetModelInstanceByName(scoped.get_namespace());
        } else {
          // If we fail this demand, we forgot to remove bad data before
          // building the `result`.
          DRAKE_DEMAND(scoped.get_namespace().empty());
        }
        instanced_name.name = scoped.get_element();
        return instanced_name;
      });
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

const CollisionFilterGroupResolver::GroupData*
CollisionFilterGroupResolver::FindGroup(const DiagnosticPolicy& diagnostic,
                                        const std::string& group_name) const {
  auto iter = groups_.find(group_name);
  if (iter == groups_.end()) {
    diagnostic.Error(fmt::format(
        "collision filter group with name '{}' not found", group_name));
    return nullptr;
  }
  return &iter->second;
}

const RigidBody<double>* CollisionFilterGroupResolver::FindBody(
    std::string_view name, ModelInstanceIndex model_instance) const {
  if (plant_->HasBodyNamed(name, model_instance)) {
    return &plant_->GetBodyByName(name, model_instance);
  }
  return {};
}

const DeformableBody<double>* CollisionFilterGroupResolver::FindDeformableBody(
    std::string name, ModelInstanceIndex model_instance) const {
  const auto& deformable_model = plant_->deformable_model();

  if (deformable_model.HasBodyNamed(name, model_instance)) {
    return &deformable_model.GetBodyByName(name, model_instance);
  }
  return {};
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
