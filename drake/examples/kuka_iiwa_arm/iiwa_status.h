#pragma once

#include <string>
#include <Eigen/Core>

#include "drake/lcmt_iiwa_status.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// This class holds various information about the joint positions of
/// the IIWA robot (intended for use as a state vector to pass into
/// BotVisualizer).  As a generic receiver of the `lcmt_iiwa_status`
/// message, it could in the future be expanded to hold additional
/// data from that message as needed.  Models the `drake::LCMVector`
/// concept.
template <typename ScalarType = double>
class IiwaStatus {
 public:
  IiwaStatus()
      : joint_position_values_(
            Eigen::Matrix<ScalarType, kNumJoints, 1>::Zero()) {}

  typedef drake::lcmt_iiwa_status LCMMessageType;
  // TODO(sam.creasey) stop duplicating this
  static std::string channel() { return "IIWA_STATUS"; }

  // TODO(sam.creasey) stop duplicating this
  static const int kNumJoints = 7;
  static const int RowsAtCompileTime = Eigen::Dynamic;
  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;
  size_t size() const { return kNumJoints; }

  const EigenType& value() const { return joint_position_values_; }
  void set_value(const EigenType& joint_position_values_in) {
    joint_position_values_ = joint_position_values_in;
  }

  /// Magic conversion specialization back to Eigen.
  friend EigenType toEigen(const IiwaStatus<ScalarType>& vec) {
    return vec.joint_position_values_;
  }

 private:
  EigenType joint_position_values_;
};

/// Implemented per the `drake::LCMVector` concept for the encode
/// function.  Currently only handles measured joint positions (since
/// that's all `IiwaStatus` does).
template <typename ScalarType>
bool encode(const double& t, const IiwaStatus<ScalarType>& wrap,
            // NOLINTNEXTLINE(runtime/references)
            drake::lcmt_iiwa_status& msg) {
  msg.timestamp = static_cast<int64_t>(t * 1000);
  msg.num_joints = wrap.kNumJoints;
  msg.joint_position_measured.resize(msg.num_joints);
  for (int i = 0; i < msg.num_joints; i++) {
    msg.joint_position_measured[i] = wrap.value()[i];
  }
  msg.joint_position_commanded.resize(msg.num_joints, 0);
  msg.joint_position_ipo.resize(msg.num_joints, 0);
  msg.joint_torque_measured.resize(msg.num_joints, 0);
  msg.joint_torque_commanded.resize(msg.num_joints, 0);
  msg.joint_torque_external.resize(msg.num_joints, 0);
  return true;
}

/// Implemented per the `drake::LCMVector` concept for the decode
/// function.  Currently only handles measured joint position (since
/// that's all `IiwaStatus` does).
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
