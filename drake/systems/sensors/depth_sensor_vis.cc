#include "drake/systems/sensors/depth_sensor_vis.h"

#include <vector>

#include "bot_core/pointcloud_t.hpp"

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/sensors/depth_sensor_output.h"

namespace drake {
namespace systems {
namespace sensors {

DepthSensorVis::DepthSensorVis(const std::string& name,
    const DepthSensorSpecification& spec, lcm::DrakeLcmInterface* lcm)
        : name_(name), spec_(spec), lcm_(lcm) {
  depth_readings_input_port_index_ =
      DeclareVectorInputPort(DepthSensorOutput<double>(spec_)).get_index();
}

const InputPortDescriptor<double>&
DepthSensorVis::depth_readings_input_port() const {
  return this->get_input_port(depth_readings_input_port_index_);
}

void DepthSensorVis::DoPublish(const Context<double>& context) const {
  // Obtains the input.
  const DepthSensorOutput<double>* depth_data =
      this->template EvalVectorInput<DepthSensorOutput>(context,
          depth_readings_input_port_index_);
  const Eigen::Matrix3Xd point_cloud = depth_data->GetPointCloud();

  bot_core::pointcloud_t message;
  message.frame_id = std::string(RigidBodyTreeConstants::kWorldName);
  message.n_points = point_cloud.cols();
  for (int i = 0; i < point_cloud.cols(); ++i) {
    const auto& point = point_cloud.col(i);
    message.points.push_back(std::vector<float>{
        static_cast<float>(point(0)),
        static_cast<float>(point(1)),
        static_cast<float>(point(2))});
  }
  message.n_channels = 0;

  const int num_bytes = message.getEncodedSize();
  std::vector<uint8_t> message_bytes(num_bytes);
  const int num_bytes_encoded =
      message.encode(message_bytes.data(), 0, num_bytes);
  DRAKE_ASSERT(num_bytes_encoded == num_bytes);

  lcm_->Publish("DRAKE_POINTCLOUD_" + name_, message_bytes.data(), num_bytes);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
