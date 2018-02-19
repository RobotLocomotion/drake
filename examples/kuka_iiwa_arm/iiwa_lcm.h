#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the iiwa arm.

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

extern const double kIiwaLcmStatusPeriod;

/// Handles lcmt_iiwa_command messages from a LcmSubscriberSystem.
/// Has two output ports: one for the commanded position for each joint along
/// with an estimate of the commanded velocity for each joint, and another for
/// commanded additional feedforward joint torque.
class IiwaCommandReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaCommandReceiver)

  explicit IiwaCommandReceiver(int num_joints = kIiwaArmNumJoints);

  /// Sets the initial position of the controlled iiwa prior to any
  /// commands being received.  @p x contains the starting position.
  /// This position will be the commanded position (with zero
  /// velocity) until a position message is received.  If this
  /// function is not called, the starting position will be the zero
  /// configuration.
  void set_initial_position(systems::Context<double>* context,
                            const Eigen::Ref<const VectorX<double>> x) const;

  const systems::OutputPort<double>& get_commanded_state_input_port()
      const {
    return this->get_output_port(0);
  }

  const systems::OutputPort<double>& get_commanded_torque_input_port()
      const {
    return this->get_output_port(1);
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
  const int num_joints_;
};

/// Creates and outputs lcmt_iiwa_command messages
///
/// This system has two vector-valued input ports, one for the
/// commanded position (which must be connected) and one for commanded
/// torque (which is optional).  If the torque input port is not
/// connected, then no torque values will be emitted in the resulting
/// message.
///
/// This system has one abstract valued output port that contains a
/// systems::Value object templated on type `lcmt_iiwa_command`. Note that this
/// system does not actually send this message on an LCM channel. To send the
/// message, the output of this system should be connected to an input port of
/// a systems::lcm::LcmPublisherSystem that accepts a
/// systems::Value object templated on type `lcmt_iiwa_command`.
class IiwaCommandSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaCommandSender)

  explicit IiwaCommandSender(int num_joints = kIiwaArmNumJoints);

  const systems::InputPortDescriptor<double>& get_position_input_port() const {
    return this->get_input_port(position_input_port_);
  }

  const systems::InputPortDescriptor<double>& get_torque_input_port() const {
    return this->get_input_port(torque_input_port_);
  }

 private:
  void OutputCommand(const systems::Context<double>& context,
                     lcmt_iiwa_command* output) const;

  const int num_joints_;
  const int position_input_port_{};
  const int torque_input_port_{};
};

// TODO(sam.creasey) Add output for torques once we have a system
// which needs them.

/// Handles lcmt_iiwa_status messages from a LcmSubscriberSystem.  Has
/// the following output ports:
///
/// * Measured position and estimated velocity for each joint
/// * Last commanded position for each joint
///
/// All ports will continue to output their initial state (typically
/// zero) until a message is received.
class IiwaStatusReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaStatusReceiver)

  explicit IiwaStatusReceiver(int num_joints = kIiwaArmNumJoints);

  const systems::OutputPort<double>& get_measured_position_output_port() const {
    return this->get_output_port(measured_position_output_port_);
  }

  const systems::OutputPort<double>& get_commanded_position_output_port()
      const {
    return this->get_output_port(commanded_position_output_port_);
  }

 private:
  void OutputMeasuredPosition(const systems::Context<double>& context,
                              systems::BasicVector<double>* output) const;
  void OutputCommandedPosition(const systems::Context<double>& context,
                               systems::BasicVector<double>* output) const;

  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
      systems::DiscreteValues<double>* discrete_state) const override;

  const int num_joints_;
  const int measured_position_output_port_{};
  const int commanded_position_output_port_{};
};

/// Creates and outputs lcmt_iiwa_status messages.
///
/// This system has five vector-valued input ports, one for the plant's
/// current state, one for the most recently received position command, one
/// for the most recently received joint torque command, one for the plant's
/// measured joint torque, and one for the plant's external joint torque. The
/// last two inputs are optional. If left unconnected, the measured joint torque
/// field in the output message will be identical to the commanded joint torque,
/// and external torque will be filled with zeros.
/// The state and command ports contain a position and velocity for each joint
/// (velocity is unused, this is done to be more readily compatible with the
/// outputs from IiwaCommandReceiver and RigidBodyPlant). The torque related
/// ports contain a single torque for each joint.
///
/// This system has one abstract valued output port that contains a
/// systems::Value object templated on type `lcmt_iiwa_status`. Note that this
/// system does not actually send this message on an LCM channel. To send the
/// message, the output of this system should be connected to an input port of
/// a systems::lcm::LcmPublisherSystem that accepts a
/// systems::Value object templated on type `lcmt_iiwa_status`. For an example
/// of this, see iiwa_wsg_simulation.cc.
///
/// This system is presently only used in simulation. The robot hardware drivers
/// publish directly to LCM and do not make use of this system.
class IiwaStatusSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaStatusSender)

  explicit IiwaStatusSender(int num_joints = kIiwaArmNumJoints);

  const systems::InputPortDescriptor<double>& get_command_input_port() const {
    return this->get_input_port(0);
  }

  const systems::InputPortDescriptor<double>& get_state_input_port() const {
    return this->get_input_port(1);
  }

  const systems::InputPortDescriptor<double>& get_commanded_torque_input_port()
      const {
    return this->get_input_port(2);
  }

  /**
   * Optional input port. If not connected, the joint_torque_measured field in
   * the output message will be identical to the joint_torque_commanded field.
   */
  const systems::InputPortDescriptor<double>& get_measured_torque_input_port()
      const {
    return this->get_input_port(3);
  }

  /**
   * Optional input port. If not connected, the joint_torque_external field in
   * the output message will be zeros.
   */
  const systems::InputPortDescriptor<double>& get_external_torque_input_port()
      const {
    return this->get_input_port(4);
  }

 private:
  // This is the method to use for the output port allocator.
  lcmt_iiwa_status MakeOutputStatus() const;

  // This is the calculator method for the output port.
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_iiwa_status* output) const;

  const int num_joints_;
};

/**
 * A translator class that converts the contact force field in
 * systems::ContactResults to external joint torque for a set of specified
 * model instances in RigidBodyTree. The input, systems::ContactResults, is
 * assumed to be generated using the same RigidBodyTree. This class also assumes
 * that contact force is the only cause of external joint torque, no other
 * effects such as friction is considered.
 */
class IiwaContactResultsToExternalTorque : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaContactResultsToExternalTorque)

  /**
   * Constructor.
   * @param tree const RigidBodyTree reference that generates ContactResults.
   * @param model_instance_ids A set of model instances in @p tree, whose
   * corresponding generalized contact forces will be stacked and output. Order
   * of @p model_instance_ids does not matter.
   */
  IiwaContactResultsToExternalTorque(
      const RigidBodyTree<double>& tree,
      const std::vector<int>& model_instance_ids);

 private:
  const int num_joints_;
  // Maps model instance ids to velocity indices and number of
  // velocity states in the RigidBodyTree.  Values are stored as a
  // pair of (index, count).
  std::vector<std::pair<int, int>> velocity_map_;

  void OutputExternalTorque(const systems::Context<double>& context,
                            systems::BasicVector<double>* output) const;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
