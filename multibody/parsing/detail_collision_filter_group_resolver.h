#pragma once

#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/geometry_set.h"
#include "drake/multibody/parsing/detail_collision_filter_groups_impl.h"
#include "drake/multibody/parsing/detail_strongly_connected_components.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

// Collects and resolves collision filter group information, potentially across
// multiple models. Uses the same scoped-name semantics as SDFormat and model
// directives to allow reference and resolution of names across models.
//
// All methods use the supplied MultibodyPlant as the source of truth for
// existence and naming of models and bodies.  The Resolve() step writes the
// finished collision filtering rules to the plant.
//
// All methods may emit warnings or errors to a passed diagnostic policy
// channel.
//
// @note model_instance: All methods use a model instance parameter to denote
// the current scope where information is found during parsing. Its name is
// used to expand the relative names found to absolute names for further
// internal processing. If a model instance value is not provided, names are
// evaluated with respect to the "root" of the plant.
//
// @note valid_model_instance The resolver rejects model instance parameters
// and scoped names that refer to models built prior to the "current parsing
// operation". The current parsing operation covers the processing of some
// outermost model definition file, and all of its nested or included
// models. During that processing, one or more new model instances will be
// added to represent the models in the input data.

// The collision filter group resolver expects to be constructed before the
// parsing operation begins, and be called to resolve the filters after the
// parsing operation ends.  To restrict references to models within the current
// parse, it remembers the plant's `num_model_instances()` at the time the
// resolver is constructed, calling it the
// `minimum_model_instance_index`. Hence, valid model instances must lie with
// in the range:
//
//  valid_model ∈  [minimum_model_instance_index(), num_model_instances())
//
// Passing model instance parameters out of that range is a programming
// error. Parsed names that refer to models out of range will generate
// error messages.
//
class CollisionFilterGroupResolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CollisionFilterGroupResolver);

  // The plant parameter is aliased, and must outlive the resolver.
  // @pre plant is not nullptr.
  explicit CollisionFilterGroupResolver(MultibodyPlant<double>* plant);

  ~CollisionFilterGroupResolver();

  // @returns the minimum model instance index in force for this resolver.
  ModelInstanceIndex minimum_model_instance_index() const {
    return minimum_model_instance_index_;
  }

  // Adds a collision filter group. Locates bodies by name immediately, and
  // emits errors for illegal or ambiguous names, empty groups, missing bodies,
  // or duplicate group definition attempts.
  //
  // @param diagnostic          The error-reporting channel.
  // @param group_name          The name of the group being defined.
  // @param body_names          The names of the bodies that are group members.
  // @param member_group_names  The names of groups whose member bodies are
  //                            group members.
  // @param model_instance      The current model, for scoping purposes.
  //                            @see model_instance note above.
  //
  // @pre group_name is not empty.
  // @pre no member strings of body_names are empty.
  // @pre model_instance has a valid index or else no value.
  //      @see valid_model_instance note above.
  void AddGroup(const drake::internal::DiagnosticPolicy& diagnostic,
                const std::string& group_name,
                const std::set<std::string>& body_names,
                const std::set<std::string>& member_group_names,
                std::optional<ModelInstanceIndex> model_instance);

  // Adds a group pair. Emits diagnostics for illegal names.  Two distinct
  // names will resolve to an exclude-between-groups rule; identical names will
  // resolve to an exclude-within-group rule. Group names may refer to a group
  // not defined until later; group existence will be checked in the Resolve()
  // step.
  //
  // @param diagnostic     The error-reporting channel.
  // @param group_name_a   The name of a defined group.
  // @param group_name_b   The name of a defined group.
  // @param model_instance The current model, for scoping purposes.
  //                       @see model_instance note above.
  //
  // @pre neither group_name_a nor group_name_b is empty.
  // @pre model_instance has a valid index or else no value.
  //      @see valid_model_instance note above.
  void AddPair(const drake::internal::DiagnosticPolicy& diagnostic,
               const std::string& group_name_a, const std::string& group_name_b,
               std::optional<ModelInstanceIndex> model_instance);

  // Resolves group pairs to rules. Emits diagnostics for undefined groups.
  // @pre cannot have been previously invoked on this instance.
  // @returns the collision filter groups found after resolution.
  CollisionFilterGroupsImpl<InstancedName> Resolve(
      const drake::internal::DiagnosticPolicy& diagnostic);

 private:
  struct GroupData {
    std::set<std::string> body_names;
    geometry::GeometrySet geometries;
  };

  std::string FullyQualify(
      const std::string& name,
      std::optional<ModelInstanceIndex> model_instance) const;

  const GroupData* FindGroup(
      const drake::internal::DiagnosticPolicy& diagnostic,
      const std::string& group_name) const;

  const RigidBody<double>* FindBody(std::string_view name,
                                    ModelInstanceIndex model_instance) const;
  const DeformableBody<double>* FindDeformableBody(
      std::string name, ModelInstanceIndex model_instance) const;

  MultibodyPlant<double>* const plant_;

  std::map<std::string, GroupData> groups_;
  std::set<SortedPair<std::string>> pairs_;

  DirectedGraph<std::string> group_insertion_graph_;
  bool is_resolved_{false};
  ModelInstanceIndex minimum_model_instance_index_{};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
