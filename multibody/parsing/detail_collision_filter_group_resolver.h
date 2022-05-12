#pragma once

#include <map>
#include <optional>
#include <set>
#include <string>

#include "drake/common/diagnostic_policy.h"
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
class CollisionFilterGroupResolver {
 public:
  // The @p plant parameter is aliased, and must outlive the resolver.
  // @pre plant is not nullptr.
  explicit CollisionFilterGroupResolver(MultibodyPlant<double>* plant);

  // Adds a collision filter group. Locates bodies by name immediately, and
  // emits errors for illegal names, empty groups, or missing bodies.
  //
  // @pre group_name is not empty.
  // @pre no member strings of body_names are empty.
  // @pre model_instance has a valid index or else no value.
  void AddGroup(
      const drake::internal::DiagnosticPolicy& diagnostic,
      const std::string& group_name,
      const std::set<std::string> body_names,
      std::optional<ModelInstanceIndex> model_instance);

  // Adds a group pair. Emit diagnostics for illegal names.  Two distinct names
  // will resolve to an exclude-between-groups rule; identical names will
  // resolve to an exclude-within-group rule. Group names may refer to a group
  // not defined until later; group existence will be checked in the Resolve()
  // step.
  //
  // @pre neither group_name_a nor group_name_b is empty.
  // @pre model_instance has a valid index or else no value.
  void AddPair(
      const drake::internal::DiagnosticPolicy& diagnostic,
      const std::string& group_name_a,
      const std::string& group_name_b,
      std::optional<ModelInstanceIndex> model_instance);

  // Resolve group pairs to rules. Emit diagnostics for undefined groups.
  void Resolve(const drake::internal::DiagnosticPolicy& diagnostic);

 private:
  bool CheckLegalName(const drake::internal::DiagnosticPolicy& diagnostic,
                      const std::string& name,
                      const std::string& description) const;
  std::string FullyQualify(
      const std::string& name,
      std::optional<ModelInstanceIndex> model_instance) const;
  bool IsGroupDefined(const drake::internal::DiagnosticPolicy& diagnostic,
                      const std::string& group_name) const;
  const Body<double>* FindBody(
      const std::string& name,
      std::optional<ModelInstanceIndex> model_instance);
  MultibodyPlant<double>* plant_;
  std::map<std::string, geometry::GeometrySet> groups_;
  std::set<SortedPair<std::string>> pairs_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
