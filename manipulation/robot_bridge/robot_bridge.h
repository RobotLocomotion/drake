#pragma once

#include <string>
#include <tuple>
#include <unordered_map>

#include "drake/manipulation/robot_bridge/motion_primitive.h"
#include "drake/manipulation/robot_bridge/motion_primitive_selector.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/port_switch.h"

namespace drake {
namespace manipulation {
namespace robot_bridge {

/**
 * This is basically a diagram of a bunch of MotionPrimitives attached to a
 * MotionPrimitiveSelector.
 * The input ports:
 * - state
 * - all additional inputs required by the primtives
 * Output ports:
 * - position
 * - torque
 * - motion summary
 */
class RobotBridge : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotBridge)

  explicit RobotBridge(const multibody::MultibodyPlant<double>* plant,
                       const multibody::Frame<double>* tool_frame,
                       double timestep);

  void Initialize(const Eigen::VectorXd& x0,
                  systems::Context<double>* context) const;

 private:
  std::unordered_map<std::string, MotionPrimitive*> primitives_;
  MotionPrimitiveSelector* primitive_selector_{};
  systems::PortSwitch<double>* position_switch_{};
  systems::PortSwitch<double>* torque_switch_{};
  systems::PortSwitch<double>* motion_summary_switch_{};
};

}  // namespace robot_bridge
}  // namespace manipulation
}  // namespace drake
