#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the iiwa arm.

#include <memory>

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

/// Creates and outputs lcmt_iiwa_status messages.
///
/// This system has two vector-valued input ports, one for the plant's current
/// state and one for the most recently received command. The input port
/// containing the plant's state must be twice the size of the input port
/// containing the most recently received command. This is because there should
/// be a joint position state and velocity state for each commanded joint.
///
/// This system has one abstract valued output port that contains a
/// systems::Value object templated on type `lcmt_iiwa_status`. Note that this
/// system does not actually send this message on an LCM channel. To send the
/// message, the output of this system should be connected to an input port of
/// a systems::lcm::LcmPublisherSystem that accepts a
/// systems::Value object templated on type `lcmt_iiwa_status`. For an example
/// of this, see iiwa_swg_simulation.cc.
class IiwaStatusSender : public systems::LeafSystem<double> {
 public:
  IiwaStatusSender();

  const systems::InputPortDescriptor<double>& get_command_input_port() const {
    return this->get_input_port(0);
  }

  const systems::InputPortDescriptor<double>& get_state_input_port() const {
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
