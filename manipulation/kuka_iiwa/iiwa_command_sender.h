#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

/**
Creates and outputs lcmt_iiwa_command messages.

Note that this system does not actually send the message on an LCM channel. To
send the message, the output of this system should be connected to a
systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_command>().

This system has three vector-valued input ports:

- one for the commanded position, which must be connected if position mode is
  specified,
- one for commanded torque, which is optional if position and torque mode is
  specified (for backwards compatibility), but must be connected if it is
  torque only, and
- one for the time to use, in seconds, for the message timestamp, which is
  optional.

If position and torque mode is specified, the torque input port can remain
unconnected; the message will contain torque values of size zero. If position
mode is not specified, the message will contain position values of size zero.

If the time input port is not connected, the context time will be used for
message timestamp.

This system has one abstract-valued output port of type lcmt_iiwa_command.

@system
name: IiwaCommandSender
input_ports:
- position (required if using position mode)
- torque (optional if using position mode, required in torque mode)
- time (optional)
output_ports:
- lcmt_iiwa_command
@endsystem

@see `lcmt_iiwa_command.lcm` for additional documentation.
*/
class IiwaCommandSender final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaCommandSender)

  explicit IiwaCommandSender(
      int num_joints = kIiwaArmNumJoints,
      IiwaControlMode control_mode = IiwaControlMode::kPositionAndTorque);
  ~IiwaCommandSender() final;

  /** @name Named accessors for this System's input and output ports. */
  //@{
  const systems::InputPort<double>& get_time_input_port() const;
  const systems::InputPort<double>& get_position_input_port() const;
  const systems::InputPort<double>& get_torque_input_port() const;
  //@}

 private:
  void CalcOutput(const systems::Context<double>&, lcmt_iiwa_command*) const;

  const int num_joints_;
  const IiwaControlMode control_mode_{IiwaControlMode::kPositionAndTorque};

  const drake::systems::InputPort<double>* position_input_port_{};
  const drake::systems::InputPort<double>* torque_input_port_{};
  const drake::systems::InputPort<double>* time_input_port_{};
};

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
