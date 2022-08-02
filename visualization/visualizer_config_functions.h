#pragma once

#include <vector>

#include "drake/geometry/drake_visualizer_params.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_buses.h"
#include "drake/visualization/visualizer_config.h"

namespace drake {
namespace visualization {

/** Adds LCM visualization publishers to communicate to drake_visualizer
and/or meldis.

@experimental The exact function signature is subject to change as we polish
this new feature.

@param[in] config The visualization configuration.

@param[in,out] builder The diagram to add visuzliation systems into.

@param[in] lcm_buses (Optional) The available LCM buses to use for visualization
message publication. When not provided, uses the `lcm` interface if provided, or
else the `config.lcm_bus` must be set to "default" in which case an approriate
DrakeLcm object is constructed and used internally.

@param[in] plant (Optional) The MultibodyPlant to visualize.
When not provided, visualizes the system named "plant" in the given `builder`.

@param[in] scene_graph (Optional) The MultibodyPlant to visualize.
When not provided, visualizes the system named "scene_graph" in the given
`builder`.

@param[in] lcm (Optional) The LCM interface used for visualization message
publication. When not provided, uses the `config.lcm_bus` value to look up
the appropriate interface from `lcm_buses`.

@pre The `builder` is non-null.

@pre Either the given `builder` contains an LcmInterfaceSystem that matches the
`config.lcm_bus` or else the provided `lcm` is non-null; see
drake::systems::lcm::ApplyLcmBusConfig().

@pre Either the given `builder` contains a MultibodyPlant system named "plant"
or else the provided `plant` is non-null.

@pre Either the given `builder` contains a SceneGraph system named "scene_graph"
or else the provided `scene_graph` is non-null.

@pre The MultibodyPlant is already finalized, as in `plant.Finalize()`. */
void ApplyVisualizerConfig(
    const VisualizerConfig& config,
    systems::DiagramBuilder<double>* builder,
    const systems::lcm::LcmBuses* lcm_buses = nullptr,
    const multibody::MultibodyPlant<double>* plant = nullptr,
    const geometry::SceneGraph<double>* scene_graph = nullptr,
    lcm::DrakeLcmInterface* lcm = nullptr);

/** Adds LCM visualization publishers to communicate to drake_visualizer
and/or meldis, using all of the default configuration settings.

@pre AddMultibodyPlant() has already been called on the given `builder`.
@pre The MultibodyPlant in the given `builder` is already finalized, as in
     `plant.Finalize()`. */
void AddDefaultVisualizer(systems::DiagramBuilder<double>* builder);

namespace internal {

// (For unit testing only.)
std::vector<geometry::DrakeVisualizerParams>
ConvertVisualizerConfigToParams(const VisualizerConfig&);

}  // namespace internal
}  // namespace visualization
}  // namespace drake
