#pragma once

#include "bot_core/pointcloud_t.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/depth_sensor_specification.h"

namespace drake {
namespace systems {
namespace sensors {

/// A DepthSensorToLcmPointCloudMessage takes as input a DepthSensorOutput and
/// the pose of the depth sensor in the world (`X_WS`). If the input port
/// containing `X_WS` is unconnected, a std::runtime_error will be thrown
/// while evaluating the output of this system. This system outputs an
/// AbstractValue containing a `Value<bot_core::pointcloud_t>` LCM message that
/// defines a point cloud in the world frame. This message can then be sent to
/// `drake-visualizer` using LcmPublisherSystem for visualizing the depth
/// readings contained within the inputted DepthSensorOutput.
class DepthSensorToLcmPointCloudMessage : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DepthSensorToLcmPointCloudMessage)

  /// A %DepthSensorToLcmPointCloudMessage constructor.
  ///
  /// @param spec The specification of the depth sensor whose output is being
  /// visualized.
  explicit DepthSensorToLcmPointCloudMessage(
      const DepthSensorSpecification& spec);

  /// Returns the input port containing a DepthSensorOutput.
  const InputPort<double>& depth_readings_input_port() const;

  /// Returns the input port containing `X_WS`.
  const InputPort<double>& pose_input_port() const;

  /// Returns the abstract valued output port that contains a
  /// `Value<bot_core::pointcloud_t>`.
  const OutputPort<double>& pointcloud_message_output_port() const;

 private:
  // This is the calculator method for the output port.
  void CalcPointCloudMessage(const systems::Context<double>& context,
                             bot_core::pointcloud_t* output) const;

  const DepthSensorSpecification& spec_;

  // See class description for the semantics of the input and output ports.
  int depth_readings_input_port_index_{};
  int pose_input_port_index_{};
  int output_port_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
