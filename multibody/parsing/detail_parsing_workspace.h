#pragma once

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

// ParsingWorkspace bundles the commonly-needed elements for parsing
// routines. It owns nothing; all members are references or pointers to object
// owned elsewhere.
struct ParsingWorkspace {
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ParsingWorkspace)

  // All parameters are aliased; they must have a lifetime greater than that of
  // this struct.
  ParsingWorkspace(
    const PackageMap& package_map_in,
    const drake::internal::DiagnosticPolicy& diagnostic_in,
    MultibodyPlant<double>* plant_in,
    geometry::SceneGraph<double>* scene_graph_in = nullptr)
      : package_map(package_map_in),
        diagnostic(diagnostic_in),
        plant(plant_in),
        scene_graph(scene_graph_in) {
    DRAKE_DEMAND(plant != nullptr);
  }

  const PackageMap& package_map;
  const drake::internal::DiagnosticPolicy& diagnostic;
  MultibodyPlant<double>* const plant;
  geometry::SceneGraph<double>* const scene_graph;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

