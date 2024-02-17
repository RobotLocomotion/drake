#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/drake_visualizer_params.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/meshcat/contact_visualizer_params.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_buses.h"
#include "drake/visualization/visualization_config.h"

namespace drake {
namespace visualization {

/** Adds LCM visualization publishers to communicate to Meshcat and/or Meldis.

<dl><dt>Example</dt><dd>
@code
// Create a builder.
DiagramBuilder<double> builder;

// Add the MultibodyPlant and SceneGraph.
const MultibodyPlantConfig plant_config = ...;
MultibodyPlant<double>& plant = AddMultibodyPlant(plant_config, &builder);
// ... populate the plant, e.g., with a Parser, ...
plant.Finalize();

// Add the visualization.
const VisualizationConfig vis_config = ...;
ApplyVisualizationConfig(vis_config, &builder);

// Simulate.
Simulator<double> simulator(builder.Build());
// ... etc ...
@endcode
</dd></dl>

@param[in] config The visualization configuration.

@param[in,out] builder The diagram to add visualization systems into.

@param[in] lcm_buses (Optional) The available LCM buses to use for visualization
message publication. When not provided, uses the `lcm` interface if provided, or
else the `config.lcm_bus` must be set to "default" in which case an appropriate
drake::lcm::DrakeLcm object is constructed and used internally.

@param[in] plant (Optional) The MultibodyPlant to visualize.
In the common case where a MultibodyPlant has already been added to `builder`
using either AddMultibodyPlant() or AddMultibodyPlantSceneGraph(), the default
value (nullptr) here is suitable and generally should be preferred.
When provided, it must be a System that's been added to the the given `builder`.
When not provided, visualizes the system named "plant" in the given `builder`.

@param[in] scene_graph (Optional) The SceneGraph to visualize.
In the common case where a SceneGraph has already been added to `builder` using
either AddMultibodyPlant() or AddMultibodyPlantSceneGraph(), the default value
(nullptr) here is suitable and generally should be preferred.
When provided, it must be a System that's been added to the the given `builder`.
When not provided, visualizes the system named "scene_graph" in the given
`builder`.

@param[in] meshcat (Optional) A Meshcat object for visualization message
publication. When not provided, a Meshcat object will be created unless
`config.enable_meshcat_creation` is set to false.

@param[in] lcm (Optional) The LCM interface used for visualization message
publication. When not provided, uses the `config.lcm_bus` value to look up
the appropriate interface from `lcm_buses`.

@pre The `builder` is non-null.

@pre Either the `config.lcm_bus` is set to "default", or else `lcm_buses` is
non-null and contains a bus named `config.lcm_bus`, or else `lcm` is non-null.

@pre Either the given `builder` contains a MultibodyPlant system named "plant"
or else the provided `plant` is non-null.

@pre Either the given `builder` contains a SceneGraph system named "scene_graph"
or else the provided `scene_graph` is non-null.

@pre The MultibodyPlant is already finalized, as in `plant.Finalize()`.

@see drake::visualization::AddDefaultVisualization()
@see drake::multibody::AddMultibodyPlant()
@see drake::systems::lcm::ApplyLcmBusConfig() */
void ApplyVisualizationConfig(
    const VisualizationConfig& config, systems::DiagramBuilder<double>* builder,
    const systems::lcm::LcmBuses* lcm_buses = nullptr,
    const multibody::MultibodyPlant<double>* plant = nullptr,
    geometry::SceneGraph<double>* scene_graph = nullptr,
    std::shared_ptr<geometry::Meshcat> meshcat = nullptr,
    lcm::DrakeLcmInterface* lcm = nullptr);

/** Adds LCM visualization publishers to communicate to Meshcat and/or Meldis,
using all of the default configuration settings.

@param meshcat An optional existing Meshcat instance. (If nullptr, then a
meshcat instance will be created.)

<dl><dt>Example</dt><dd>
@code
// Create a builder.
DiagramBuilder<double> builder;

// Add the MultibodyPlant and SceneGraph.
const MultibodyPlantConfig plant_config = ...;
MultibodyPlant<double>& plant = AddMultibodyPlant(plant_config, &builder);
// ... populate the plant, e.g., with a Parser, ...
plant.Finalize();

// Add the visualization.
AddDefaultVisualization(&builder);

// Simulate.
Simulator<double> simulator(builder.Build());
// ... etc ...
@endcode
</dd></dl>

@pre AddMultibodyPlant() or AddMultibodyPlantSceneGraph() has already been
     called on the given `builder`.

@pre The MultibodyPlant in the given `builder` is already finalized, as in
     `plant.Finalize()`.

@see drake::visualization::ApplyVisualizationConfig()
@see drake::multibody::AddMultibodyPlant() */
void AddDefaultVisualization(
    systems::DiagramBuilder<double>* builder,
    std::shared_ptr<geometry::Meshcat> meshcat = nullptr);

namespace internal {

// (This function is declared in the header so that unit tests can call it.)
std::vector<geometry::DrakeVisualizerParams>
ConvertVisualizationConfigToDrakeParams(const VisualizationConfig&);

// (This function is declared in the header so that unit tests can call it.)
std::vector<geometry::MeshcatVisualizerParams>
ConvertVisualizationConfigToMeshcatParams(const VisualizationConfig&);

// (This function is declared in the header so that unit tests can call it.)
multibody::meshcat::ContactVisualizerParams
ConvertVisualizationConfigToMeshcatContactParams(const VisualizationConfig&);

}  // namespace internal
}  // namespace visualization
}  // namespace drake
