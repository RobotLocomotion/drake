#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the iiwa arm.

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// Handles lcmt_iiwa_command messages from a LcmSubscriberSystem.
/// Has a single output port which publishes the commanded position
/// for each joint along with an estimate of the commanded velocity
/// for each joint.
class IiwaCommandReceiver : public systems::LeafSystem<double> {
 public:
  IiwaCommandReceiver();

  /// Sets the initial position of the controlled iiwa prior to any
  /// commands being received.  @p x contains the starting position.
  /// This position will be the commanded position (with zero
  /// velocity) until a position message is received.  If this
  /// function is not called, the starting position will be the zero
  /// configuration.
  void set_initial_position(
      systems::Context<double>* context,
      const Eigen::Ref<const VectorX<double>> x) const;

 protected:
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      systems::DiscreteState<double>* discrete_state) const override;
};

/// Creates and outputs lcmt_iiwa_status messages.
///
/// This system has two vector-valued input ports, one for the plant's
/// current state and one for the most recently received command.
/// Both ports contain a position and velocity for each joint
/// (velocity is unused, this is done to be more readily compatible
/// with the outputs from IiwaCommandReceiver and RigidBodyPlant).
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
