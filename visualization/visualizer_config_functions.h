#pragma once

#include <vector>

#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_buses.h"
#include "sim/common/drake_visualizer_config.h"

namespace anzu {
namespace sim {

/** Adds LCM visualization publishers to communicate to drake-visualizer.
@pre plant.is_finalized() */
void ApplyDrakeVisualizerConfig(
    const DrakeVisualizerConfig& config,
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::geometry::SceneGraph<double>& scene_graph,
    const drake::systems::lcm::LcmBuses& lcm,
    drake::systems::DiagramBuilder<double>* builder);

namespace internal {

// (For unit testing only.)
std::vector<drake::geometry::DrakeVisualizerParams>
ConvertDrakeVisualizerConfigToParams(
    const DrakeVisualizerConfig&);

}  // namespace internal
}  // namespace sim
}  // namespace anzu
