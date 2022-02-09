#pragma once

#include "drake/common/diagnostic_policy.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

struct ParsingWorkspace {
  ParsingWorkspace(
    const PackageMap& package_map_in,
    const drake::internal::DiagnosticPolicy& diagnostic_in,
    MultibodyPlant<double>* plant_in,
    geometry::SceneGraph<double>* scene_graph_in = nullptr)
      : package_map(package_map_in),
        diagnostic(diagnostic_in),
        plant(plant_in),
        scene_graph(scene_graph_in) {}

  const PackageMap& package_map;
  const drake::internal::DiagnosticPolicy& diagnostic;
  MultibodyPlant<double>* const plant;
  geometry::SceneGraph<double>* const scene_graph;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

