#pragma once

#include <map>
#include <optional>
#include <set>
#include <string>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/geometry_set.h"
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
// TODO(rpoyner-tri): clarify the roles of the world instance (0) and the
// default model instance (1).
//
class CollisionFilterGroupResolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CollisionFilterGroupResolver)

  // The @p plant parameter is aliased, and must outlive the resolver.
  // @pre plant is not nullptr.
  explicit CollisionFilterGroupResolver(MultibodyPlant<double>* plant);

  ~CollisionFilterGroupResolver();

  // Adds a collision filter group. Locates bodies by name immediately, and
  // emits errors for illegal names, empty groups, or missing bodies.
  //
  // @param diagnostic     The error-reporting channel.
  // @param group_name     The name of the group being defined.
  // @param body_names     The names of the bodies that are group members.
  // @param model_instance The current model, for scoping purposes.
  //                       @see model_instance note above.
  //
  // @pre group_name is not empty.
  // @pre no member strings of body_names are empty.
  // @pre model_instance has a valid index or else no value.
  void AddGroup(
      const drake::internal::DiagnosticPolicy& diagnostic,
      const std::string& group_name,
      const std::set<std::string>& body_names,
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
  void AddPair(
      const drake::internal::DiagnosticPolicy& diagnostic,
      const std::string& group_name_a,
      const std::string& group_name_b,
      std::optional<ModelInstanceIndex> model_instance);

  // Resolves group pairs to rules. Emits diagnostics for undefined groups.
  //
  // @pre cannot have be previously invoked on this instance.
  void Resolve(const drake::internal::DiagnosticPolicy& diagnostic);

 private:
  bool CheckLegalName(const drake::internal::DiagnosticPolicy& diagnostic,
                      std::string_view name,
                      const std::string& description) const;

  std::string FullyQualify(
      const std::string& name,
      std::optional<ModelInstanceIndex> model_instance) const;

  const geometry::GeometrySet* FindGroup(
      const drake::internal::DiagnosticPolicy& diagnostic,
      const std::string& group_name) const;

  const Body<double>* FindBody(
      const std::string& name,
      std::optional<ModelInstanceIndex> model_instance);

  MultibodyPlant<double>* const plant_;
  std::map<std::string, geometry::GeometrySet> groups_;
  std::set<SortedPair<std::string>> pairs_;
  bool is_resolved_{false};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
