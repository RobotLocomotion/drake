#include "drake/systems/sensors/depth_sensor_to_lcm_point_cloud_message.h"

#include <string>
#include <vector>

#include "bot_core/pointcloud_t.hpp"

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/sensors/depth_sensor_output.h"

namespace drake {
namespace systems {
namespace sensors {

DepthSensorToLcmPointCloudMessage::DepthSensorToLcmPointCloudMessage(
      const DepthSensorSpecification& spec) : spec_(spec) {
  input_port_index_ =
      DeclareVectorInputPort(DepthSensorOutput<double>(spec_)).get_index();
  output_port_index_ =
      DeclareAbstractOutputPort(systems::Value<bot_core::pointcloud_t>())
          .get_index();
}

const InputPortDescriptor<double>&
DepthSensorToLcmPointCloudMessage::depth_readings_input_port() const {
  return this->get_input_port(input_port_index_);
}

const OutputPortDescriptor<double>&
DepthSensorToLcmPointCloudMessage::pointcloud_message_output_port() const {
  return System<double>::get_output_port(output_port_index_);
}

void DepthSensorToLcmPointCloudMessage::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Obtains the input.
  const DepthSensorOutput<double>* depth_data =
      this->template EvalVectorInput<DepthSensorOutput>(context,
          input_port_index_);
  const Eigen::Matrix3Xd point_cloud = depth_data->GetPointCloud();

  // Obtains the output.
  bot_core::pointcloud_t& message =
      output->GetMutableData(output_port_index_)->
        GetMutableValue<bot_core::pointcloud_t>();

  message.frame_id = std::string(RigidBodyTreeConstants::kWorldName);
  message.n_points = point_cloud.cols();
  message.points.clear();
  for (int i = 0; i < point_cloud.cols(); ++i) {
    const auto& point = point_cloud.col(i);
    message.points.push_back(std::vector<float>{
        static_cast<float>(point(0)),
        static_cast<float>(point(1)),
        static_cast<float>(point(2))});
  }
  message.n_channels = 0;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
