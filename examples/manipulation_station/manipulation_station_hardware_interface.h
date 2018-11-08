#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace examples {
namespace manipulation_station {

/// A System that connects via message-passing to the hardware manipulation
/// station.
///
/// Note: Users must call Connect() after initialization.
///
/// @{
///
/// @system{ ManipulationStationHardwareInterface,
///   @input_port{iiwa_position}
///   @input_port{iiwa_feedforward_torque}
///   @input_port{wsg_position}
///   @input_port{wsg_force_limit},
///   @output_port{iiwa_position_commanded}
///   @output_port{iiwa_position_measured}
///   @output_port{iiwa_velocity_estimated}
///   @output_port{iiwa_torque_commanded}
///   @output_port{iiwa_torque_measured}
///   @output_port{iiwa_torque_external}
///   @output_port{wsg_state_measured}
///   @output_port{wsg_force_measured}
///   @output_port{camera0_rgb_image}
///   @output_port{camera0_depth_image}
///   @output_port{...}
///   @output_port{cameraN_rgb_image}
///   @output_port{cameraN_depth_image}
/// }
///
/// @ingroup manipulation_station_systems
/// @}
///
class ManipulationStationHardwareInterface : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManipulationStationHardwareInterface)

  /// Subscribes to an incoming camera message on the channel
  ///   DRAKE_RGBD_CAMERA_IMAGES_<camera_id>
  /// where @p camera_ids contains the ids, typically serial numbers, and
  /// declares the output ports camera%d_rgb_image and camera%d_depth_image,
  /// where %d is the camera number starting at zero.
  ManipulationStationHardwareInterface(
      const std::vector<std::string> camera_ids = {});

  /// Starts a thread to receive network messages, and blocks execution until
  /// the first messages have been received.
  void Connect(bool wait_for_cameras = true);

  /// For parity with ManipulationStation, we maintain a MultibodyPlant of
  /// the IIWA arm, with the lumped-mass equivalent spatial inertia of the
  /// Schunk WSG gripper.
  // TODO(russt): Actually add the equivalent mass of the WSG.
  const multibody::multibody_plant::MultibodyPlant<double>&
  get_controller_plant() const {
    return *owned_controller_plant_;
  }

 private:
  std::unique_ptr<multibody::multibody_plant::MultibodyPlant<double>>
      owned_controller_plant_;
  std::unique_ptr<lcm::DrakeLcm> owned_lcm_;
  systems::lcm::LcmSubscriberSystem* wsg_status_subscriber_;
  systems::lcm::LcmSubscriberSystem* iiwa_status_subscriber_;
  std::vector<systems::lcm::LcmSubscriberSystem*> camera_subscribers_;
};

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
