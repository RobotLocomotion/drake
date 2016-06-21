#pragma once

#include <string>
#include <Eigen/Core>

#include "lcmtypes/drake/lcmt_iiwa_status.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// Models the Drake::LCMVector concept
template <typename ScalarType = double>
class IiwaStatus {
 public:
  IiwaStatus()
      : value_(Eigen::Matrix<ScalarType, num_joints, 1>::Zero()) {}

  typedef drake::lcmt_iiwa_status LCMMessageType;
  // TODO(sam.creasey) stop duplicating this
  static std::string channel() { return "IIWA_STATUS"; }

  // TODO(sam.creasey) stop duplicating this
  static const int num_joints = 7;
  static const int RowsAtCompileTime = Eigen::Dynamic;
  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;
  size_t size() const { return num_joints; }

  const EigenType& value() const { return value_; }
  void set_value(const EigenType& value_in) { value_ = value_in; }

  /// Magic conversion specialization back to Eigen.
  friend EigenType toEigen(const IiwaStatus<ScalarType>& vec) {
    return vec.value_;
  }

 private:
  EigenType value_;
};

template <typename ScalarType>
bool encode(const double& t, const IiwaStatus<ScalarType>& wrap,
            // NOLINTNEXTLINE(runtime/references)
            drake::lcmt_iiwa_status& msg) {
  msg.timestamp = static_cast<int64_t>(t * 1000);
  msg.num_joints = wrap.num_joints;
  msg.joint_position_measured.resize(msg.num_joints);
  for (int i = 0; i <  msg.num_joints; i++) {
    msg.joint_position_measured[i] = wrap.value()[i];
  }
  msg.joint_position_commanded.resize(msg.num_joints, 0);
  msg.joint_position_ipo.resize(msg.num_joints, 0);
  msg.joint_torque_measured.resize(msg.num_joints, 0);
  msg.joint_torque_commanded.resize(msg.num_joints, 0);
  msg.joint_torque_external.resize(msg.num_joints, 0);
  return true;
}

template <typename ScalarType>
bool decode(const drake::lcmt_iiwa_status& msg,
            // NOLINTNEXTLINE(runtime/references)
            double& t,
            // NOLINTNEXTLINE(runtime/references)
            IiwaStatus<ScalarType>& wrap) {
  t = static_cast<double>(msg.timestamp) / 1000.0;
  typename IiwaStatus<ScalarType>::EigenType new_value;
  new_value.resize(msg.num_joints);
  for (int i = 0; i < msg.num_joints; i++) {
    new_value[i] = msg.joint_position_measured[i];
  }
  wrap.set_value(new_value);
  return true;
}


}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
