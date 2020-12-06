#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the Schunk WSG gripper.

#include <memory>
#include <vector>

#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

/// Handles the command for the Schunk WSG gripper from a LcmSubscriberSystem.
///
/// It has one input port: "command_message" for lcmt_schunk_wsg_command
/// abstract values.
///
/// It has two output ports: one for the commanded finger position represented
/// as the desired distance between the fingers in meters, and one for the
/// commanded force limit.  The commanded position and force limit are scalars
/// (BasicVector<double> of size 1).
///
/// @system
/// name: SchunkWsgCommandReceiver
/// input_ports:
/// - command_message
/// output_ports:
/// - position
/// - force_limit
/// @endsystem
///
/// @ingroup manipulation_systems
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

  const systems::OutputPort<double>& get_position_output_port() const {
    return this->GetOutputPort("position");
  }

  const systems::OutputPort<double>& get_force_limit_output_port() const {
    return this->GetOutputPort("force_limit");
  }

 private:
  void CalcPositionOutput(const systems::Context<double>& context,
                          systems::BasicVector<double>* output) const;

  void CalcForceLimitOutput(const systems::Context<double>& context,
                            systems::BasicVector<double>* output) const;

  const double initial_position_{};
  const double initial_force_{};
};


/// Send lcmt_schunk_wsg_command messages for a Schunk WSG gripper.  Has
/// two input ports: one for the commanded finger position represented as the
/// desired signed distance between the fingers in meters, and one for the
/// commanded force limit.  The commanded position and force limit are
/// scalars (BasicVector<double> of size 1).
///
/// @system
/// name: SchunkWsgCommandSender
/// input_ports:
/// - position
/// - force_limit (optional)
/// output_ports:
/// - lcmt_schunk_wsg_command
/// @endsystem
///
/// @ingroup manipulation_systems
class SchunkWsgCommandSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgCommandSender)

  explicit SchunkWsgCommandSender(double default_force_limit = 40.0);

  const systems::InputPort<double>& get_position_input_port()
  const {
    return this->get_input_port(position_input_port_);
  }

  const systems::InputPort<double>& get_force_limit_input_port()
  const {
    return this->get_input_port(force_limit_input_port_);
  }

  const systems::OutputPort<double>& get_command_output_port() const {
    return this->get_output_port(0);
  }

 private:
  void CalcCommandOutput(
      const systems::Context<double>& context,
      lcmt_schunk_wsg_command* output) const;

 private:
  const systems::InputPortIndex position_input_port_{};
  const systems::InputPortIndex force_limit_input_port_{};
  const double default_force_limit_;
};


/// Handles lcmt_schunk_wsg_status messages from a LcmSubscriberSystem.  Has
/// two output ports: one for the measured state of the gripper, represented as
/// the signed distance between the fingers in meters and its corresponding
/// velocity, and one for the measured force.
///
/// @system
/// name: SchunkWsgStatusReceiver
/// input_ports:
/// - lcmt_schunk_wsg_status
/// output_ports:
/// - state
/// - force
/// @endsystem
///
/// @ingroup manipulation_systems
class SchunkWsgStatusReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgStatusReceiver)

  SchunkWsgStatusReceiver();

  const systems::InputPort<double>& get_status_input_port() const {
    return this->get_input_port(0);
  }

  const systems::OutputPort<double>& get_state_output_port()
  const {
    return this->get_output_port(state_output_port_);
  }

  const systems::OutputPort<double>& get_force_output_port()
  const {
    return this->get_output_port(force_output_port_);
  }

 private:
  void CopyStateOut(
      const systems::Context<double>& context,
      systems::BasicVector<double>* output) const;

  void CopyForceOut(
      const systems::Context<double>& context,
      systems::BasicVector<double>* output) const;

 private:
  const systems::OutputPortIndex state_output_port_{};
  const systems::OutputPortIndex force_output_port_{};
};


/// Sends lcmt_schunk_wsg_status messages for a Schunk WSG.  This
/// system has one input port for the current state of the WSG, and one
/// optional input port for the measured gripping force.
///
/// @system
/// name: SchunkStatusSender
/// input_ports:
/// - state
/// - force
/// output_ports:
/// - lcmt_schunk_wsg_status
/// @endsystem
///
/// The state input is a BasicVector<double> of size 2 -- with one position
/// and one velocity -- representing the distance between the fingers (positive
/// implies non-penetration).
///
/// @ingroup manipulation_systems
class SchunkWsgStatusSender : public systems::LeafSystem<double> {
 public:
  SchunkWsgStatusSender();

  const systems::InputPort<double>& get_state_input_port() const {
    DRAKE_DEMAND(state_input_port_.is_valid());
    return this->get_input_port(state_input_port_);
  }

  const systems::InputPort<double>& get_force_input_port() const {
    return this->get_input_port(force_input_port_);
  }

 private:
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_schunk_wsg_status* output) const;

  systems::InputPortIndex state_input_port_{};
  systems::InputPortIndex force_input_port_{};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
