#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_panda_command.hpp"
#include "drake/manipulation/franka_panda/panda_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace franka_panda {

/// Creates and outputs lcmt_panda_command messages.
///
/// Note that this system does not actually send the message an LCM channel. To
/// send the message, the output of this system should be connected to a
/// LcmPublisherSystem::Make<lcmt_panda_command>().
///
/// This system has vector-valued input ports, each one of size num_joints.
///
/// This system has one abstract-valued output port of type lcmt_panda_command.
///
/// @system
/// name: PandaCommandSender
/// input_ports:
/// - position (*)
/// - velocity (*)
/// - torque (*)
/// output_ports:
/// - lcmt_panda_command
/// @endsystem
///
/// (*) Each input port is present if it's relevant to the control_mode passed
/// to the constructor.
///
/// @see `lcmt_panda_command.lcm` for additional documentation.
class PandaCommandSender final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PandaCommandSender);

  /// @param control_mode is a bitset of one or more control mode constants
  /// defined in PandaControlModes namespace. Use bitwise OR to combine modes.
  PandaCommandSender(int num_joints, PandaControlMode control_mode);
  ~PandaCommandSender() final;

  /// @name Named accessors for this System's input and output ports.
  //@{
  const drake::systems::InputPort<double>& get_position_input_port() const;
  const drake::systems::InputPort<double>& get_velocity_input_port() const;
  const drake::systems::InputPort<double>& get_torque_input_port() const;
  //@}

 private:
  void CalcOutput(const drake::systems::Context<double>&,
                  drake::lcmt_panda_command*) const;

  const int num_joints_;
  const PandaControlMode control_mode_;

  const drake::systems::InputPort<double>* position_input_port_{};
  const drake::systems::InputPort<double>* velocity_input_port_{};
  const drake::systems::InputPort<double>* torque_input_port_{};
};

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
