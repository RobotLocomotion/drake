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
/// system has one input port for the current state of the simulated
/// WSG (probably a RigidBodyPlant), and one optional input port for the
/// measured gripping force.
/// @ingroup manipulation_systems
class SchunkWsgStatusSender : public systems::LeafSystem<double> {
 public:
  SchunkWsgStatusSender(int input_state_size, int input_torque_size,
                        int position_index, int velocity_index);

  const systems::InputPort<double>& get_input_port_wsg_state() const {
    return this->get_input_port(input_port_wsg_state_);
  }

  const systems::InputPort<double>& get_input_port_measured_torque()
      const {
    return this->get_input_port(input_port_measured_torque_);
  }

 private:
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_schunk_wsg_status* output) const;

  const int position_index_{};
  const int velocity_index_{};
  systems::InputPortIndex input_port_measured_torque_{};
  systems::InputPortIndex input_port_wsg_state_{};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
