#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "sim/common/camera_config.h"

namespace anzu {
namespace sim {

/** Constructs a camera network (rgbd sensor and publishing systems) connected
 to `scene_graph`. As specified, the RGB and/or depth images from the camera are
 published via `lcm` on the channel
 <tt>DRAKE_RGBD_CAMERA_IMAGES_{camera_config.name}</tt>.

 @throws std::exception if camera_config contains invalid values. */
void DrakeApplyCameraConfig(const CameraConfig& camera_config,
                            drake::multibody::MultibodyPlant<double>* sim_plant,
                            drake::systems::DiagramBuilder<double>* builder,
                            drake::geometry::SceneGraph<double>* scene_graph,
                            drake::lcm::DrakeLcmInterface* lcm);

/** Constructs a camera network (rgbd sensor and publishing systems) connected
 to `scene_graph`. As specified, the RGB and/or depth images from the camera are
 published via `lcm` on the channel
 <tt>DRAKE_RGBD_CAMERA_IMAGES_{camera_config.name}</tt>.

 @throws std::exception if camera_config contains invalid values. */
void ApplyCameraConfig(const CameraConfig& camera_config,
                       drake::multibody::MultibodyPlant<double>* sim_plant,
                       drake::systems::DiagramBuilder<double>* builder,
                       drake::geometry::SceneGraph<double>* scene_graph,
                       drake::lcm::DrakeLcmInterface* lcm);

}  // namespace sim
}  // namespace anzu
