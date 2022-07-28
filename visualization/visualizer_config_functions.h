#pragma once

#include <vector>

#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_buses.h"
#include "drake/visualization/visualizer_config.h"

namespace drake {
namespace visualization {

// TODO(jwnimmer-tri) We should be able to drop the `scene_graph` argument,
// inferring it from the plant instead in most (all?) cases.

/** Adds LCM visualization publishers to communicate to drake-visualizer.

@experimental The exact function signature is subject to change as we polish
this new feature.

@pre plant.is_finalized() */
void ApplyVisualizerConfig(
    const VisualizerConfig& config,
    const multibody::MultibodyPlant<double>& plant,
    const geometry::SceneGraph<double>& scene_graph,
    const systems::lcm::LcmBuses& lcm_buses,
    systems::DiagramBuilder<double>* builder);

/** Adds LCM visualization publishers to communicate to drake-visualizer.

@experimental The exact function signature is subject to change as we polish
this new feature.

This overload always publishes to the given `lcm` instance, thus ignoring
the given `config.lcm_bus` setting.

@pre plant.is_finalized() */
void ApplyVisualizerConfig(
    const VisualizerConfig& config,
    const multibody::MultibodyPlant<double>& plant,
    const geometry::SceneGraph<double>& scene_graph,
    lcm::DrakeLcmInterface* lcm,
    systems::DiagramBuilder<double>* builder);

namespace internal {

// (For unit testing only.)
std::vector<geometry::DrakeVisualizerParams>
ConvertVisualizerConfigToParams(const VisualizerConfig&);

}  // namespace internal
}  // namespace visualization
}  // namespace drake
