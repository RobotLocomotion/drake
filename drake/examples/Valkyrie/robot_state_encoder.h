#pragma once

#include <map>

#include "lcmtypes/bot_core/robot_state_t.hpp"

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/robotInterfaces/Side.h"

namespace drake {
namespace systems {

/// Assembles information from various input ports into a robot_state_t LCM
/// message, presented on an output port.
class RobotStateEncoder final : public LeafSystem<double> {
 public:
  static const size_t kTorqueXIndex = 0;
  static const size_t kTorqueYIndex = 1;
  static const size_t kForceZIndex = 5;

  explicit RobotStateEncoder(const RigidBodyTree<double>& tree);

  ~RobotStateEncoder() override;

  // Disable copy and assign.
  RobotStateEncoder(const RobotStateEncoder&) = delete;

  RobotStateEncoder& operator=(const RobotStateEncoder&) = delete;

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override;

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override;

  /// Returns descriptor of output port on which the LCM message is presented.
  const SystemPortDescriptor<double>& lcm_message_port() const;

  /// Returns descriptor of kinematics result input port.
  const SystemPortDescriptor<double>& kinematics_results_port() const;

  /// Returns descriptor of effort input port corresponding to @param actuator.
  const SystemPortDescriptor<double>& effort_port(
      const RigidBodyActuator& actuator) const;

  /// Returns descriptor of foot wrench input port for side @param side.
  const SystemPortDescriptor<double>& foot_contact_wrench_port(
      const Side& side) const;

  /// Returns descriptor of hand wrench input port for side @param side.
  const SystemPortDescriptor<double>& hand_contact_wrench_port(
      const Side& side) const;

 private:
  std::map<const RigidBodyActuator*, int> DeclareEffortInputPorts();

  std::map<Side, int> DeclareWrenchInputPorts();

  void SetStateAndEfforts(const Context<double>& context,
                          bot_core::robot_state_t* message) const;

  void SetForceTorque(const Context<double>& context,
                      bot_core::robot_state_t* message) const;

  // Tree to which message corresponds.
  const RigidBodyTree<double>& tree_;

  // Pointer to the body in @p tree_ that is attached to the world with a
  // floating joint. Null if there is no such body.
  const RigidBody<double>* const floating_body_;

  // Output port.
  const int lcm_message_port_index_;

  // Input ports.
  const int kinematics_results_port_index_;
  const std::map<const RigidBodyActuator*, int> effort_port_indices_;
  const std::map<Side, int> foot_wrench_port_indices_;
  const std::map<Side, int> hand_wrench_port_indices_;
};

}  // namespace systems
}  // namespace drake
