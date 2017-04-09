#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/sensors/depth_sensor_specification.h"


namespace drake {
namespace systems {
namespace sensors {

/// A DepthSensorVis takes as input a DepthSensorOutput and publishes
/// `bot_core::pointcloud_t` LCM messages for visualizing the depth readings
/// contained within the DepthSensorOutput. The messages are published on LCM
/// channel "DRAKE_POINTCLOUD_[sensor name]" where [sensor name] is the name
/// provided to the constructor.
///
class DepthSensorVis : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DepthSensorVis)

  /// A %DepthSensorVis constructor.
  ///
  /// @param name The name of the depth sensor being visualized. This is
  /// appended to the LCM channel name. See class description for details.
  ///
  /// @param spec The specification of the depth sensor whose output is being
  /// visualized by this system.
  ///
  /// @param lcm The LCM interface for publishing LCM messages. The lifespan of
  /// this parameter must exceed that of this object.
  DepthSensorVis(const std::string& name, const DepthSensorSpecification& spec,
      lcm::DrakeLcmInterface* lcm);

  /// Returns a descriptor of the input port containing a DepthSensorOutput.
  const InputPortDescriptor<double>& depth_readings_input_port()
      const;

 protected:
  /// This system doesn't output anything.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override {}

  /// See class description for what this system publishes.
  void DoPublish(const Context<double>& context) const override;

 private:
  const std::string name_;
  const DepthSensorSpecification& spec_;
  lcm::DrakeLcmInterface* lcm_{};
  int depth_readings_input_port_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
