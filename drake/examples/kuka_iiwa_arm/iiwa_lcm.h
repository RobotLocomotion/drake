#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the iiwa arm.

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// Handles lcmt_iiwa_command messages from a LcmSubscriberSystem.
/// Has a single output port which publishes the commanded position
/// for each joint.  All values on the output port will be zero if no
/// LCM message has arrived with a command yet.
class IiwaCommandReceiver : public systems::LeafSystem<double> {
 public:
  IiwaCommandReceiver();

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;
};

/// Sends lcmt_iiwa_status messages.  This system has two input ports,
/// one for the current state of the plant and one for the most
/// recently received command.  The input port for the state of the
/// plant will be twice the size of the command (position and velocity
/// state for each joint vs. commanded position for each joint).
class IiwaStatusSender : public systems::LeafSystem<double> {
 public:
  IiwaStatusSender();

  const systems::SystemPortDescriptor<double>& get_command_input_port() const {
    return this->get_input_port(0);
  }

  const systems::SystemPortDescriptor<double>& get_state_input_port() const {
    return this->get_input_port(1);
  }

  std::unique_ptr<systems::SystemOutput<double>> AllocateOutput(
      const systems::Context<double>& context) const override;

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
