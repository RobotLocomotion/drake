#include "drake/systems/sensors/depth_sensor_to_lcm_point_cloud_message.h"

#include <string>
#include <vector>

#include "bot_core/pointcloud_t.hpp"

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/depth_sensor_output.h"

namespace drake {
namespace systems {

using rendering::PoseVector;

namespace sensors {

DepthSensorToLcmPointCloudMessage::DepthSensorToLcmPointCloudMessage(
    const DepthSensorSpecification& spec)
    : spec_(spec) {
  depth_readings_input_port_index_ =
      DeclareVectorInputPort(DepthSensorOutput<double>(spec_)).get_index();
  pose_input_port_index_ =
      DeclareVectorInputPort(PoseVector<double>()).get_index();
  output_port_index_ =
      DeclareAbstractOutputPort(
          &DepthSensorToLcmPointCloudMessage::CalcPointCloudMessage)
          .get_index();
}

const InputPortDescriptor<double>&
DepthSensorToLcmPointCloudMessage::depth_readings_input_port() const {
  return this->get_input_port(depth_readings_input_port_index_);
}

const InputPortDescriptor<double>&
DepthSensorToLcmPointCloudMessage::pose_input_port() const {
  return this->get_input_port(pose_input_port_index_);
}

const OutputPort<double>&
DepthSensorToLcmPointCloudMessage::pointcloud_message_output_port() const {
  return System<double>::get_output_port(output_port_index_);
}

void DepthSensorToLcmPointCloudMessage::CalcPointCloudMessage(
    const systems::Context<double>& context,
    bot_core::pointcloud_t* output) const {
  // Obtains the input.
  const DepthSensorOutput<double>* depth_data =
      this->template EvalVectorInput<DepthSensorOutput>(context,
          depth_readings_input_port_index_);
  const PoseVector<double>* X_WS =
      this->template EvalVectorInput<PoseVector>(context,
          pose_input_port_index_);

  // Handles the scenario where the pose input port is unconnected.
  if (X_WS == nullptr) {
    throw std::runtime_error("DepthSensorToLcmPointCloudMessage: ERROR: Pose "
        "input port unconnected.");
  }

  // Obtains the point cloud in the sensor's frame (S).
  const Eigen::Matrix3Xd point_cloud_S = depth_data->GetPointCloud();

  bot_core::pointcloud_t& message = *output;
  message.frame_id = std::string(RigidBodyTreeConstants::kWorldName);
  message.n_points = point_cloud_S.cols();
  message.points.clear();
  for (int i = 0; i < point_cloud_S.cols(); ++i) {
    const auto& point_S = point_cloud_S.col(i);
    Eigen::Vector3d point_W = X_WS->get_isometry() * point_S;
    message.points.push_back(std::vector<float>{
        static_cast<float>(point_W(0)),
        static_cast<float>(point_W(1)),
        static_cast<float>(point_W(2))});
  }
  message.n_channels = 0;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
