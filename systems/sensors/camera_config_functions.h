#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/camera_config.h"

namespace drake {
namespace systems {
namespace sensors {

/** Constructs a camera network (rgbd sensor and publishing systems) connected
 to `scene_graph`. As specified, the RGB and/or depth images from the camera are
 published via `lcm` on the channel
 <tt>DRAKE_RGBD_CAMERA_IMAGES_{camera_config.name}</tt>.

 @pre `plant` and `scene_graph` both belong to the `builder`.
 @throws std::exception if camera_config contains invalid values. */
void ApplyCameraConfig(const CameraConfig& config,
                       multibody::MultibodyPlant<double>* plant,
                       DiagramBuilder<double>* builder,
                       geometry::SceneGraph<double>* scene_graph,
                       drake::lcm::DrakeLcmInterface* lcm);

}  // namespace sensors
}  // namespace systems
}  // namespace drake
