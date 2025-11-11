#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/lcmt_panda_command.hpp"
#include "drake/manipulation/franka_panda/panda_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace franka_panda {

/// Handles lcmt_panda_command message from a LcmSubscriberSystem.
///
/// Note that this system does not actually subscribe to an LCM channel. To
/// receive the message, the input of this system should be connected to a
/// LcmSubscriberSystem::Make<drake::lcmt_panda_command>().
///
/// It has one required input port, "lcmt_panda_command".
/// It has several output ports, each one of size num_joints.
///
/// @system
/// name: PandaCommandReceiver
/// input_ports:
/// - lcmt_panda_command
/// - position_measured
/// output_ports:
/// - position (*)
/// - velocity (*)
/// - torque   (*)
/// @endsystem
///
/// (*) Each output port is present iff the control_mode passed to the
/// constructor set the corresponding CONTROL_MODE bit.
///
/// Prior to receiving a valid lcmt_panda_command message, the "position" output
/// (if present) initially feeds through from the "position_measured" input, and
/// both the "velocity" and "torque" outputs (if present) are zero.
///
/// If discrete update events are enabled (e.g., during simulation), the system
/// latches the "position_measured" input into state during the first event,
/// and the "position" output (if present) comes from the latched state, not the
/// input.
///
/// The lcmt_panda_command input must match the num_joints and control_mode that
/// were passed to the constructor: the message's control_mode_expected must be
/// set to the same value as the constructor's control_mode, and the message's
/// position, velocity, torque vectors must be sized to match the constructor's
/// num_joint iff the corresponding bit of the control_mode is set or must be
/// zero-sized otherwise.
/// TODO(jeremy.nimmer) The control_mode_expected is not actually validated yet
/// at runtime, but will be in the future (once we fix our code to stop setting
/// it incorrectly).
class PandaCommandReceiver final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PandaCommandReceiver);

  /// @param control_mode is a bitset of one or more control mode constants
  /// defined in PandaControlModes namespace. Use bitwise OR to combine modes.
  PandaCommandReceiver(int num_joints, PandaControlMode control_mode);
  ~PandaCommandReceiver() final;

  /// (Advanced.) Copies the current "position_measured" input
  /// into Context state, and changes the behavior of the "position"
  /// output to produce the latched state if no message has been received yet.
  /// The latching already happens automatically during the first discrete
  /// update event (e.g., when using a Simulator); this method exists for use
  /// when not already using a Simulator or other special cases.
  /// @pre the control_mode has the POSITION bit set
  void LatchInitialPosition(drake::systems::Context<double>*) const;

  const drake::systems::InputPort<double>& get_message_input_port() const {
    return *message_input_;
  }

  const drake::systems::InputPort<double>& get_position_measured_input_port()
      const {
    return *position_measured_input_;
  }

  const drake::systems::OutputPort<double>& get_commanded_position_output_port()
      const;

  const drake::systems::OutputPort<double>& get_commanded_velocity_output_port()
      const;

  const drake::systems::OutputPort<double>& get_commanded_torque_output_port()
      const;

 private:
  void LatchInitialPosition(const drake::systems::Context<double>&,
                            drake::systems::DiscreteValues<double>*) const;
  void DoCalcNextUpdateTime(const drake::systems::Context<double>&,
                            drake::systems::CompositeEventCollection<double>*,
                            double*) const;
  void CalcDefaultCommand(const drake::systems::Context<double>&,
                          drake::lcmt_panda_command*) const;
  void CalcMessageInputOrDefault(const drake::systems::Context<double>&,
                                 drake::lcmt_panda_command*) const;
  void CalcPositionOutput(const drake::systems::Context<double>&,
                          drake::systems::BasicVector<double>*) const;
  void CalcVelocityOutput(const drake::systems::Context<double>&,
                          drake::systems::BasicVector<double>*) const;
  void CalcTorqueOutput(const drake::systems::Context<double>&,
                        drake::systems::BasicVector<double>*) const;

  const int num_joints_;
  const PandaControlMode control_mode_;
  const drake::systems::InputPort<double>* message_input_{};
  const drake::systems::InputPort<double>* position_measured_input_{};
  drake::systems::DiscreteStateIndex latched_position_measured_is_set_;
  drake::systems::DiscreteStateIndex latched_position_measured_;
  const drake::systems::CacheEntry* default_command_{};
  const drake::systems::CacheEntry* message_input_or_default_{};
  const drake::systems::OutputPort<double>* commanded_position_output_{};
  const drake::systems::OutputPort<double>* commanded_velocity_output_{};
  const drake::systems::OutputPort<double>* commanded_torque_output_{};
};

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
