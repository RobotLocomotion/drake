#pragma once

#include <map>
#include <string>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

// TODO(tkoolen): currently doesn't do anything with the efforts or wrenches in
// the robot_state_t message.

/**
 * Converts a robot_state_t LCM message into a KinematicsCache object.
 *
 * Note that a RobotStateDecoder simply ignores state information for joints
 * that it doesn't know about.
 */
class RobotStateDecoder : public LeafSystem<double> {
 public:
  explicit RobotStateDecoder(const RigidBodyTree<double>& tree);

  ~RobotStateDecoder() override {}

  // Disable copy and assign.
  RobotStateDecoder(const RobotStateDecoder&) = delete;

  RobotStateDecoder& operator=(const RobotStateDecoder&) = delete;

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override;

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override;

 private:
  std::map<std::string, const RigidBody<double>*> CreateJointNameToBodyMap(
      const RigidBodyTree<double>& tree);

  const RigidBodyTree<double>& tree_;
  const RigidBody<double>* const floating_body_;
  const int robot_state_message_port_index_;
  const int kinematics_cache_port_index_;
  const std::map<std::string, const RigidBody<double>*> joint_name_to_body_;
};

}  // namespace systems
}  // namespace drake
