#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

struct ParsingWorkspace;

class ParserInterface {
 public:
  virtual ~ParserInterface() {}
  virtual std::optional<ModelInstanceIndex> AddModel(
      const DataSource& data_source, const std::string& model_name,
      const std::optional<std::string>& scope_name,
      const ParsingWorkspace& workspace) = 0;

  virtual std::vector<ModelInstanceIndex> AddAllModels(
      const DataSource& data_source,
      const std::optional<std::string>& scope_name,
      const ParsingWorkspace& workspace) = 0;
};
inline ParserInterface* NoSelect(const std::string&) { DRAKE_UNREACHABLE(); }

// ParsingWorkspace bundles the commonly-needed elements for parsing routines.
// It owns nothing; all members are references or pointers to objects owned
// elsewhere.
//
// Note that code using this struct may pass it via const-ref, but the
// indicated plant and collision resolver objects will still be mutable; only
// the pointer values within the struct are const.
struct ParsingWorkspace {
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ParsingWorkspace)

  // All parameters are aliased; they must have a lifetime greater than that of
  // this struct.
  ParsingWorkspace(
      const PackageMap& package_map_in,
      const drake::internal::DiagnosticPolicy& diagnostic_in,
      MultibodyPlant<double>* plant_in,
      internal::CollisionFilterGroupResolver* collision_resolver_in,
      std::function<ParserInterface*(const std::string&)> parser_selector_in)
      : package_map(package_map_in),
        diagnostic(diagnostic_in),
        plant(plant_in),
        collision_resolver(collision_resolver_in),
        parser_selector(parser_selector_in) {
    DRAKE_DEMAND(plant != nullptr);
    DRAKE_DEMAND(collision_resolver != nullptr);
    DRAKE_DEMAND(parser_selector != nullptr);
  }

  const PackageMap& package_map;
  const drake::internal::DiagnosticPolicy& diagnostic;
  MultibodyPlant<double>* const plant;
  internal::CollisionFilterGroupResolver* const collision_resolver;
  std::function<ParserInterface*(const std::string&)> const parser_selector;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
