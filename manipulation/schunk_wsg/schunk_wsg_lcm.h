#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the Schunk WSG gripper.

#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

/// Handles lcmt_schunk_wsg_command messages from a LcmSubscriberSystem.  Has
/// two output ports: one for the commanded finger position represented as the
/// desired distance from the center (zero) position in meters, and one for
/// the commanded force limit.  The commanded position and force limit are
/// scalars (BasicVector<double> of size 1).
///
/// @system{ SchunkWsgCommandReceiver,
///   @input_port{lcmt_schunk_wsg_command},
///   @output_port{commanded_position}
///   @output_port{force_limit} }
class SchunkWsgCommandReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgCommandReceiver)

  /// @param initial_position the commanded position to output if no LCM
  /// message has been received yet.
  ///
  /// @param initial_force the commanded force limit to output if no LCM
  /// message has been received yet.
  SchunkWsgCommandReceiver(double initial_position = 0.02,
                           double initial_force = 40);

  const systems::InputPort<double>& get_command_input_port() const {
    return this->get_input_port(0);
  }

  const systems::OutputPort<double>& get_commanded_position_output_port()
      const {
    return this->get_output_port(commanded_position_output_port_);
  }

  const systems::OutputPort<double>& get_force_limit_output_port()
      const {
    return this->get_output_port(force_limit_output_port_);
  }

 private:
  void CalcCommandedPositionOutput(
      const systems::Context<double>& context,
      systems::BasicVector<double>* output) const;

  void CalcForceLimitOutput(
      const systems::Context<double>& context,
      systems::BasicVector<double>* output) const;

 private:
  const double initial_position_{};
  const double initial_force_{};
  const systems::OutputPortIndex commanded_position_output_port_{};
  const systems::OutputPortIndex force_limit_output_port_{};
};

/// Sends lcmt_schunk_wsg_status messages for a Schunk WSG.  This
/// system has one input port for the current state of the WSG, and one
/// optional input port for the measured gripping force.
///
/// @system{ SchunkStatusSender,
///          @input_port{state}
///          @input_port{force},
///          @output_port{lcmt_schunk_wsg_status}
/// }
///
/// The state input is a BasicVector<double> of size 2 -- with one position
/// and one velocity -- representing the *positive* position of the fingers
/// from the middle/zero position.
///
/// @ingroup manipulation_systems
class SchunkWsgStatusSender : public systems::LeafSystem<double> {
 public:

  DRAKE_DEPRECATED("Don't use this constructor.  Use the default constructor "
                   "and just wire in the two-dimensional state input.  Note "
                   "that the *sign* of the expected input has also changed --"
                   " positive position means open.")
  SchunkWsgStatusSender(int input_state_size, int input_torque_size,
                        int position_index, int velocity_index);

  DRAKE_DEPRECATED("Use get_state_input_port() instead which takes a "
                   "two-dimensional BasicVector<double>.")
  const systems::InputPort<double>& get_input_port_wsg_state() const {
    DRAKE_DEMAND(input_port_wsg_state_ != -1);
    return this->get_input_port(input_port_wsg_state_);
  }

  DRAKE_DEPRECTED("Use get_force_input_port() instead.")
  const systems::InputPort<double>& get_input_port_measured_torque() {
    return this->get_input_port(force_input_port_);
  }

  SchunkWsgStatusSender();

  const systems::InputPort<double>& get_state_input_port() const {
    DRAKE_DEMAND(state_input_port_ != -1);
    return this->get_input_port(state_input_port_);
  }

  const systems::InputPort<double>& get_force_input_port()
      const {
    return this->get_input_port(force_input_port_);
  }

 private:
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_schunk_wsg_status* output) const;

  systems::InputPortIndex state_input_port_{-1};
  systems::InputPortIndex force_input_port_{};

  // TODO(russt): Remove this port after the deprecation timeline.
  systems::InputPortIndex input_port_wsg_state_{-1};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
