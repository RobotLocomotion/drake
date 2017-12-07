#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lcmtypes/bot_core/robot_state_t.hpp"

#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/controllers/side.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

// TODO(siyuan.feng): I am hard coding force torque between certain bodies
// with only the "world", not the environment, because the current
// implementation of collision groups doesn't work properly.

/// Assembles information from various input ports into a robot_state_t LCM
/// message, presented on an output port.
class RobotStateEncoder final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotStateEncoder)

  static const size_t kTorqueXIndex = 0;
  static const size_t kTorqueYIndex = 1;
  static const size_t kForceZIndex = 5;

  RobotStateEncoder(const RigidBodyTree<double>& tree,
                    const std::vector<RigidBodyFrame<double>>& ft_sensor_info);

  ~RobotStateEncoder() override;

  /// Returns the output port on which the LCM message is presented.
  const OutputPort<double>& lcm_message_port() const;

  /// Returns descriptor of kinematics result input port.
  const InputPortDescriptor<double>& kinematics_results_port() const;

  /// Returns descriptor of contact results input port.
  const InputPortDescriptor<double>& contact_results_port() const;

  /// Returns descriptor of effort input port corresponding to @param actuator.
  const InputPortDescriptor<double>& effort_port(
      const RigidBodyActuator& actuator) const;

 private:
  // This is the method used by the allocator for the output port.
  bot_core::robot_state_t MakeRobotState() const;

  // This is the calculator method for the output port.
  void OutputRobotState(const Context<double>& context,
                        bot_core::robot_state_t* output) const;

  std::map<const RigidBodyActuator*, int> DeclareEffortInputPorts();

  std::map<Side, int> DeclareWrenchInputPorts();

  void SetStateAndEfforts(const KinematicsResults<double>& kinematics_results,
                          const Context<double>& context,
                          bot_core::robot_state_t* message) const;

  void SetForceTorque(const KinematicsResults<double>& kinematics_results,
                      const ContactResults<double>& contact_results,
                      bot_core::robot_state_t* message) const;

  // Computes the spatial force applied at the origin of Body1 by Body2,
  // expressed in Body1's local frame.
  SpatialForce<double> GetSpatialForceActingOnBody1ByBody2InBody1Frame(
      const KinematicsResults<double>& kinematics_results,
      const ContactResults<double>& contact_results,
      const RigidBody<double>& body1, const RigidBody<double>& body2) const;

  const manipulation::RobotStateLcmMessageTranslator translator_;

  // Output port.
  const int lcm_message_port_index_;

  // Input ports.
  const int kinematics_results_port_index_;
  const int contact_results_port_index_;
  const std::map<const RigidBodyActuator*, int> effort_port_indices_;

  // These are used for force torque sensors.
  // The first part is the body that the sensor is attached to, and the second
  // part is local offset within the body frame.
  const std::vector<RigidBodyFrame<double>> force_torque_sensor_info_;

  // Index into force_torque_sensor_info_
  int l_foot_ft_sensor_idx_ = -1;
  int r_foot_ft_sensor_idx_ = -1;
  int l_hand_ft_sensor_idx_ = -1;
  int r_hand_ft_sensor_idx_ = -1;
};

}  // namespace systems
}  // namespace drake
