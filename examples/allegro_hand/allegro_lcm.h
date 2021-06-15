#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the allegro hand.

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/lcmt_allegro_command.hpp"
#include "drake/lcmt_allegro_status.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace allegro_hand {

const double kHardwareStatusPeriod = 0.003;

/// Handles lcmt_allegro_command messages from a LcmSubscriberSystem.
/// Has two output ports: one for the commanded position for each joint along
/// with a zero velocity for each joint, and another for commanded additional
/// feedforward joint torque. The joint torque command is currently not used.
class AllegroCommandReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AllegroCommandReceiver)

  AllegroCommandReceiver(int num_joints = kAllegroNumJoints,
                         double lcm_period = kHardwareStatusPeriod);

  /// Sets the initial position of the controlled hand prior to any
  /// commands being received.  @param x contains the starting position.
  /// This position will be the commanded position (with zero
  /// velocity) until a position message is received.  If this
  /// function is not called, the open hand pose will be the zero
  /// configuration.
  void set_initial_position(systems::Context<double>* context,
                            const Eigen::Ref<const VectorX<double>>& x) const;

  const systems::OutputPort<double>& get_commanded_state_output_port() const {
    return this->get_output_port(state_output_port_);
  }

  const systems::OutputPort<double>& get_commanded_torque_output_port() const {
    return this->get_output_port(torque_output_port_);
  }

  double lcm_period() const {
    return lcm_period_;
  }

 private:
  void CopyStateToOutput(const systems::Context<double>& context, int start_idx,
                         int length,
                         systems::BasicVector<double>* output) const;

  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
      systems::DiscreteValues<double>* discrete_state) const override;

 private:
  int state_output_port_ = 0;
  int torque_output_port_ = 0;
  const int num_joints_ = 16;
  const double lcm_period_;
};

/// Creates and outputs lcmt_allegro_status messages.
///
/// This system has three vector-valued input ports, one for the plant's
/// current state, one for the most recently received position command, and one
/// for the most recently received joint torque command.
/// The state and command ports contain a position and velocity for each joint,
/// which is supposed to be in the order of thumb(4)-index(4)-middle(4)-ring(4).
///
/// This system has one abstract valued output port that contains a
/// Value object templated on type `lcmt_allegro_status`. Note that
/// this system does not actually send this message on an LCM channel. To send
/// the message, the output of this system should be connected to an input port
/// of a systems::lcm::LcmPublisherSystem that accepts a Value object
/// templated on type `lcmt_allegro_status`.
///
/// This system is presently only used in simulation.
class AllegroStatusSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AllegroStatusSender)

  explicit AllegroStatusSender(int num_joints = kAllegroNumJoints);

  const systems::InputPort<double>& get_command_input_port() const {
    return this->get_input_port(command_input_port_);
  }

  const systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

  const systems::InputPort<double>& get_commanded_torque_input_port() const {
    return this->get_input_port(command_torque_input_port_);
  }

 private:
  // This is the calculator method for the output port.
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_allegro_status* output) const;

  int command_input_port_ = 0;
  int state_input_port_ = 0;
  int command_torque_input_port_ = 0;

  const int num_joints_ = 16;
};

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
