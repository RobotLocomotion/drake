#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_buses.h"
#include "drake/systems/sensors/camera_config.h"

namespace drake {
namespace systems {
namespace sensors {

/** Constructs a simulated camera sensor (rgbd sensor and publishing systems)
 within `builder`. As specified, the RGB, depth, and/or label images from the
 camera are published via `lcm` on the channel
 <tt>DRAKE_RGBD_CAMERA_IMAGES_{camera_config.name}</tt>.


@param[in] config The camera configuration.

@param[in,out] builder The diagram to add sensor and publishing systems into.

@param[in] lcm_buses (Optional) The available LCM buses to use for camera
message publication. When not provided, uses the `lcm` interface if provided, or
else the `config.lcm_bus` must be set to "default" in which case an appropriate
drake::lcm::DrakeLcm object is constructed and used internally.

@param[in] plant (Optional) The MultibodyPlant to use for kinematics.
In the common case where a MultibodyPlant has already been added to `builder`
using either AddMultibodyPlant() or AddMultibodyPlantSceneGraph(), the default
value (nullptr) here is suitable and generally should be preferred.
When provided, it must be a System that's been added to the given `builder`.
When not provided, uses the system named "plant" in the given `builder`.

@param[in] scene_graph (Optional) The SceneGraph to use for rendering.
In the common case where a SceneGraph has already been added to `builder` using
either AddMultibodyPlant() or AddMultibodyPlantSceneGraph(), the default value
(nullptr) here is suitable and generally should be preferred.
When provided, it must be a System that's been added to the given `builder`.
When not provided, uses the system named "scene_graph" in the given `builder`.

@param[in] lcm (Optional) The LCM interface used for visualization message
publication. When not provided, uses the `config.lcm_bus` value to look up
the appropriate interface from `lcm_buses`.

@throws std::exception if camera_config contains invalid values.

@pre The `builder` is non-null.

@pre Either the `config.lcm_bus` is set to "default", or else `lcm_buses` is
non-null and contains a bus named `config.lcm_bus`, or else `lcm` is non-null.

@pre Either the given `builder` contains a MultibodyPlant system named "plant"
or else the provided `plant` is non-null.

@pre Either the given `builder` contains a SceneGraph system named "scene_graph"
or else the provided `scene_graph` is non-null.

@see drake::multibody::AddMultibodyPlant()
@see drake::systems::lcm::ApplyLcmBusConfig() */
void ApplyCameraConfig(const CameraConfig& config,
                       DiagramBuilder<double>* builder,
                       const systems::lcm::LcmBuses* lcm_buses = nullptr,
                       const multibody::MultibodyPlant<double>* plant = nullptr,
                       geometry::SceneGraph<double>* scene_graph = nullptr,
                       drake::lcm::DrakeLcmInterface* lcm = nullptr);

}  // namespace sensors
}  // namespace systems
}  // namespace drake
