#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the iiwa arm.

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

extern const double kIiwaLcmStatusPeriod;

/**
 * A vectorized representation of lcmt_iiwa_command.
 */
// TODO(siyuan.feng@tri.global) remove this when #10149 is resolved.
template <typename T>
class IiwaCommand : public systems::BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaCommand)

  static constexpr double kUnitializedTime = 0;

  /**
   * The dimension of this will be 2 * @p num_joints + 1, which are timestamp,
   * position and torque per each joint.
   */
  explicit IiwaCommand(int num_joints);

  T utime() const;

  Eigen::VectorBlock<const VectorX<T>> joint_position() const;

  Eigen::VectorBlock<const VectorX<T>> joint_torque() const;

  void set_utime(T utime);

  /**
   * @throws if the dimension of @p q does not match num_joints at construction
   * time.
   */
  void set_joint_position(const VectorX<T>& q);

  /**
   * @throws if the dimension of @p q does not match num_joints at construction
   * time.
   */
  void set_joint_torque(const VectorX<T>& torque);

 private:
  IiwaCommand<T>* DoClone() const override {
    return new IiwaCommand<T>(num_joints_);
  }

  const int num_joints_;
};

/**
 * A translator between the LCM message type lcmt_iiwa_command and its
 * vectorized representation, IiwaCommand<double>. This is intended to be used
 * with systems::lcm::LcmPublisherSystem and systems::lcm::LcmSubscriberSystem.
 */
// TODO(siyuan.feng@tri.global) remove this when #10149 is resolved.
class IiwaCommandTranslator : public systems::lcm::LcmAndVectorBaseTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaCommandTranslator)

  /**
   * Constructs a IiwaCommandTranslator.
   * @param num_joints Number of joints of the IIWA command.
   */
  explicit IiwaCommandTranslator(int num_joints = kIiwaArmNumJoints);

  std::unique_ptr<systems::BasicVector<double>> AllocateOutputVector()
      const override;

  /**
   * Translates @p lcm_message_bytes into @p vector_base. Assumes that the
   * size of `joint_position` field in the decoded messages matches the
   * declared number of joints at construction time. If the decoded
   * `joint_torque` field is empty, the torque part of @p vector_base will
   * be filled by zeros.
   * Throws if
   * - @p lcm_message_bytes cannot be decoded as a lcmt_iiwa_command.
   * - @p vector_base is not a IiwaCommand<double>.
   * - The decoded `joint_position` in @p lcm_message_bytes has a different
   *   size.
   * - The decoded `joint_torque` in @p lcm_message_bytes has a different size.
   */
  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
                   systems::VectorBase<double>* vector_base) const override;

  /**
   * Not implemented.
   * @throws std::runtime_error.
   */
  void Serialize(double time, const systems::VectorBase<double>& vector_base,
                 std::vector<uint8_t>* lcm_message_bytes) const override;

 private:
  const int num_joints_;
};

/// Handles IIWA commands from a LcmSubscriberSystem. It has two input ports:
/// one for lcmt_iiwa_command messages and the other for the vectorized
/// version (IiwaCommand). Only one of the inputs should be connected. However,
/// if both are connected, the message one will be ignored.
/// It has two output ports: one for the commanded position for each joint along
/// with an estimate of the commanded velocity for each joint, and another for
/// commanded additional feedforward joint torque.
///
/// @system {
///   @input_port{command_message}
///   @input_port{command_vector}
///   @output_port{state}
///   @output_port{feedforward_torque}
/// }
// TODO(siyuan.feng@tri.global) remove the vector input version after #10149 is
// resolved.
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

  const systems::OutputPort<double>& get_commanded_state_output_port()
      const {
    return this->GetOutputPort("state");
  }

  const systems::OutputPort<double>& get_commanded_torque_output_port()
      const {
    return this->GetOutputPort("feedforward_torque");
  }

 private:
  void CopyStateToOutput(const systems::Context<double>& context, int start_idx,
                         int length,
                         systems::BasicVector<double>* output) const;

  // TODO(russt): This system should NOT have any state.
  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
      systems::DiscreteValues<double>* discrete_state) const override;

 private:
  const int num_joints_;
  const IiwaCommandTranslator translator_;
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

  const systems::InputPort<double>& get_position_input_port() const {
    return this->get_input_port(position_input_port_);
  }

  const systems::InputPort<double>& get_torque_input_port() const {
    return this->get_input_port(torque_input_port_);
  }

 private:
  void OutputCommand(const systems::Context<double>& context,
                     lcmt_iiwa_command* output) const;

  const int num_joints_;
  const int position_input_port_{};
  const int torque_input_port_{};
};


/// Handles lcmt_iiwa_status messages from a LcmSubscriberSystem.
///
/// @system{ IiwaStatusReceiver,
///   @input_port{lcmt_iiwa_status},
///   @output_port{position_commanded}
///   @output_port{position_measured}
///   @output_port{velocity_estimated}
///   @output_port{torque_commanded}
///   @output_port{torque_measured}
///   @output_port{torque_external} }
///
/// All ports will output all zeros until a message is received.
///
/// @see `lcmt_iiwa_status.lcm` for additional documentation.
class IiwaStatusReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaStatusReceiver)

  explicit IiwaStatusReceiver(int num_joints = kIiwaArmNumJoints);

  const systems::OutputPort<double>& get_position_commanded_output_port()
  const {
    return this->get_output_port(position_commanded_output_port_);
  }

  const systems::OutputPort<double>& get_position_measured_output_port()
  const {
    return this->get_output_port(position_measured_output_port_);
  }

  const systems::OutputPort<double>& get_velocity_estimated_output_port()
  const {
    return this->get_output_port(velocity_estimated_output_port_);
  }

  const systems::OutputPort<double>& get_torque_commanded_output_port()
  const {
    return this->get_output_port(torque_commanded_output_port_);
  }

  const systems::OutputPort<double>& get_torque_measured_output_port()
  const {
    return this->get_output_port(torque_measured_output_port_);
  }

  const systems::OutputPort<double>& get_torque_external_output_port()
  const {
    return this->get_output_port(torque_external_output_port_);
  }

  const systems::OutputPort<double>& get_state_output_port() const {
    return this->get_output_port(state_output_port_);
  }

  DRAKE_DEPRECATED(
      "This output port is deprecated.  Use the state output port, or the "
      "position_measured, velocity_estimated, and position_commanded ports "
      "separately.")
  const systems::OutputPort<double>& get_measured_position_output_port() const {
    return this->get_output_port(deprecated_measured_position_output_port_);
  }

  DRAKE_DEPRECATED("This output port is deprecated.  Use position_commanded "
                   "as an exact replacement.")
  const systems::OutputPort<double>& get_commanded_position_output_port()
      const {
    return this->get_output_port(position_commanded_output_port_);
  }

 private:
  template <std::vector<double> drake::lcmt_iiwa_status::* field>
  void CopyLcmVectorOut(const systems::Context<double>& context,
                        systems::BasicVector<double>* output) const;

  void OutputState(const systems::Context<double>& context,
                   systems::BasicVector<double>* output) const;

  void OutputDeprecatedMeasuredPosition(
      const systems::Context<double>& context,
      systems::BasicVector<double>* output) const;

  const int num_joints_;

  const int position_commanded_output_port_{};
  const int position_measured_output_port_{};
  const int velocity_estimated_output_port_{};
  const int torque_commanded_output_port_{};
  const int torque_measured_output_port_{};
  const int torque_external_output_port_{};
  const int state_output_port_{};

  const int deprecated_measured_position_output_port_{};
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
///
/// @see `lcmt_iiwa_status.lcm` for additional documentation.
class IiwaStatusSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaStatusSender)

  explicit IiwaStatusSender(int num_joints = kIiwaArmNumJoints);

  const systems::InputPort<double>& get_command_input_port() const {
    return this->get_input_port(0);
  }

  const systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(1);
  }

  const systems::InputPort<double>& get_commanded_torque_input_port()
      const {
    return this->get_input_port(2);
  }

  /**
   * Optional input port. If not connected, the joint_torque_measured field in
   * the output message will be identical to the joint_torque_commanded field.
   */
  const systems::InputPort<double>& get_measured_torque_input_port()
      const {
    return this->get_input_port(3);
  }

  /**
   * Optional input port. If not connected, the joint_torque_external field in
   * the output message will be zeros.
   */
  const systems::InputPort<double>& get_external_torque_input_port()
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
