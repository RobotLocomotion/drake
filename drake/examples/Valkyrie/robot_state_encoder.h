#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lcmtypes/bot_core/robot_state_t.hpp"

#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/robotInterfaces/Side.h"

namespace drake {
namespace systems {

// TODO(siyuan.feng): I am hard coding force torque between certain bodies
// with only the "world", not the environment, because the current
// implementation of collision groups doesn't work properly.

/// Assembles information from various input ports into a robot_state_t LCM
/// message, presented on an output port.
class RobotStateEncoder final : public LeafSystem<double> {
 public:
  static const size_t kTorqueXIndex = 0;
  static const size_t kTorqueYIndex = 1;
  static const size_t kForceZIndex = 5;

  RobotStateEncoder(const RigidBodyTree<double>& tree,
                    const std::vector<RigidBodyFrame<double>>& ft_sensor_info);

  ~RobotStateEncoder() override;

  // Disable copy and assign.
  RobotStateEncoder(const RobotStateEncoder&) = delete;

  RobotStateEncoder& operator=(const RobotStateEncoder&) = delete;

  /// Returns descriptor of output port on which the LCM message is presented.
  const OutputPortDescriptor<double>& lcm_message_port() const;

  /// Returns descriptor of kinematics result input port.
  const InputPortDescriptor<double>& kinematics_results_port() const;

  /// Returns descriptor of contact results input port.
  const InputPortDescriptor<double>& contact_results_port() const;

  /// Returns descriptor of effort input port corresponding to @param actuator.
  const InputPortDescriptor<double>& effort_port(
      const RigidBodyActuator& actuator) const;

 protected:
  std::unique_ptr<AbstractValue> AllocateOutputAbstract(
      const OutputPortDescriptor<double>& descriptor) const override;

 private:
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override;

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

  // Tree to which message corresponds.
  const RigidBodyTree<double>& tree_;

  // Pointer to the body in @p tree_ that is attached to the world with a
  // floating joint. Null if there is no such body.
  const RigidBody<double>* const floating_body_;

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
