#include "drake/multibody/sensors/rigid_body_tree_sensor.h"

#include <string>

namespace drake {
namespace multibody {

RigidBodyTreeSensor::RigidBodyTreeSensor(const RigidBodyTree<double>& tree,
                                         const std::string& sensor_name,
                                         const RigidBodyFrame<double>& frame)
    : tree_(tree), sensor_name_(sensor_name), frame_(frame) {}

const std::string& RigidBodyTreeSensor::get_name() const {
  return sensor_name_;
}

const RigidBodyTree<double>& RigidBodyTreeSensor::get_tree() const {
  return tree_;
}

const RigidBodyFrame<double>& RigidBodyTreeSensor::get_frame() const {
  return frame_;
}

}  // namespace multibody
}  // namespace drake
