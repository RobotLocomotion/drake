#pragma once

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

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
      internal::CollisionFilterGroupResolver* collision_resolver_in)
      : package_map(package_map_in),
        diagnostic(diagnostic_in),
        plant(plant_in),
        collision_resolver(collision_resolver_in) {
    DRAKE_DEMAND(plant != nullptr);
    DRAKE_DEMAND(collision_resolver != nullptr);
  }

  const PackageMap& package_map;
  const drake::internal::DiagnosticPolicy& diagnostic;
  MultibodyPlant<double>* const plant;
  internal::CollisionFilterGroupResolver* const collision_resolver;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
