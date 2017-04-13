#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/sensors/depth_sensor_specification.h"

namespace drake {
namespace systems {
namespace sensors {

/// A DepthSensorToLcmPointCloudMessage takes as input a DepthSensorOutput and
/// outputs an AbstractValue containing a `Value<bot_core::pointcloud_t>` LCM
/// message. This message can then be sent to `drake-visualizer` using
/// LcmPublisherSystem for visualizing the depth readings contained within the
/// DepthSensorOutput.
class DepthSensorToLcmPointCloudMessage : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DepthSensorToLcmPointCloudMessage)

  /// A %DepthSensorToLcmPointCloudMessage constructor.
  ///
  /// @param spec The specification of the depth sensor whose output is being
  /// visualized.
  explicit DepthSensorToLcmPointCloudMessage(
      const DepthSensorSpecification& spec);

  /// Returns a descriptor of the input port containing a DepthSensorOutput.
  const InputPortDescriptor<double>& depth_readings_input_port() const;

  /// Returns a descriptor of the abstract valued output port that contains a
  /// `Value<bot_core::pointcloud_t>`.
  const OutputPortDescriptor<double>& pointcloud_message_output_port() const;

 protected:
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  const DepthSensorSpecification& spec_;

  // See class description for the semantics of the input and output ports.
  int input_port_index_{};
  int output_port_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
