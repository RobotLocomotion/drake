#pragma once

/// @file
/// This file contains classes dealing with sending/receiving
/// LCM messages related to the planar gripper.
/// TODO(rcory) Create doxygen system diagrams for the classes below.
///
/// All (q, v) state vectors in this file are of the format
/// (joint_positions, joint_velocities).

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_planar_gripper_command.hpp"
#include "drake/lcmt_planar_gripper_status.hpp"
#include "drake/systems/framework/event_status.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace planar_gripper {

using systems::OutputPort;
using systems::InputPort;

// By default the planar gripper has 3 fingers.
constexpr int kGripperDefaultNumFingers = 3;

// This is rather arbitrary, for now.
// TODO(rcory) Refine this value once the planner comes online.
constexpr double kGripperLcmStatusPeriod = 0.010;

/// Handles lcmt_planar_gripper_command messages from a LcmSubscriberSystem.
///
/// This system has one abstract valued input port which expects a
/// Value object templated on type `lcmt_planar_gripper_command`.
///
/// This system has two output ports. The first reports the commanded position
/// and velocity for all joints, and the second reports the commanded joint
/// torques.
class GripperCommandDecoder : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GripperCommandDecoder)

  /// Constructor.
  /// @param num_fingers The total number of fingers used on the planar-gripper.
  explicit GripperCommandDecoder(int num_fingers = kGripperDefaultNumFingers);

  /// Sets the initial position of the controlled gripper prior to any
  /// commands being received.  @p x contains the starting position.
  /// This position will be the commanded position (with zero
  /// velocity) until a position message is received.  If this
  /// function is not called, the starting position will be the zero
  /// configuration.
  void set_initial_position(
      systems::Context<double>* context,
      const Eigen::Ref<const VectorX<double>> x) const;

  const systems::OutputPort<double>& get_state_output_port() const {
    DRAKE_DEMAND(state_output_port_ != nullptr);
    return *state_output_port_;
  }

  const systems::OutputPort<double>& get_torques_output_port() const {
    DRAKE_DEMAND(torques_output_port_ != nullptr);
    return *torques_output_port_;
  }

 private:
  void OutputStateCommand(const systems::Context<double>& context,
                     systems::BasicVector<double>* output) const;

  void OutputTorqueCommand(const systems::Context<double>& context,
                          systems::BasicVector<double>* output) const;

  /// Event handler of the periodic discrete state update.
  systems::EventStatus UpdateDiscreteState(
      const systems::Context<double>& context,
      systems::DiscreteValues<double>* discrete_state) const;

  const int num_fingers_;
  const int num_joints_;
  const OutputPort<double>* state_output_port_{};
  const OutputPort<double>* torques_output_port_{};
};

/// Creates and outputs lcmt_planar_gripper_command messages.
///
/// This system has two vector-valued input ports containing the
/// desired position and velocity in the first port, and commanded torque on the
/// second port.
///
/// This system has one abstract valued output port that contains a
/// Value object templated on type `lcmt_planar_gripper_command`. Note
/// that this system does not actually send this message on an LCM
/// channel. To send the message, the output of this system should be
/// connected to an input port of a systems::lcm::LcmPublisherSystem
/// that accepts a Value object templated on type
/// `lcmt_planar_gripper_command`.
class GripperCommandEncoder : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GripperCommandEncoder)

  /// Constructor.
  /// @param num_joints The total number of fingers used on the planar-gripper.
  explicit GripperCommandEncoder(int num_fingers = kGripperDefaultNumFingers);

  const systems::InputPort<double>& get_state_input_port() const {
    DRAKE_DEMAND(state_input_port_ != nullptr);
    return *state_input_port_;
  }

  const systems::InputPort<double>& get_torques_input_port() const {
    DRAKE_DEMAND(torques_input_port_ != nullptr);
    return *torques_input_port_;
  }

 private:
  void OutputCommand(const systems::Context<double>& context,
                     lcmt_planar_gripper_command* output) const;

  const int num_fingers_;
  const int num_joints_;
  const InputPort<double>* state_input_port_{};
  const InputPort<double>* torques_input_port_{};
};

/// Handles lcmt_planar_gripper_status messages from a LcmSubscriberSystem.
///
/// This system has one abstract valued input port which expects a
/// Value object templated on type `lcmt_planar_gripper_status`.
///
/// This system has two vector valued output ports which report
/// measured position and velocity (state) as well as fingertip forces (fy, fz).
///
/// All ports will continue to output their initial state (typically
/// zero) until a message is received.
class GripperStatusDecoder : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GripperStatusDecoder)

  /// Constructor.
  /// @param num_fingers The total number of fingers used on the planar-gripper.
  explicit GripperStatusDecoder(int num_fingers = kGripperDefaultNumFingers);

  const systems::OutputPort<double>& get_state_output_port() const {
    DRAKE_DEMAND(state_output_port_ != nullptr);
    return *state_output_port_;
  }

  const systems::OutputPort<double>& get_force_output_port() const {
    DRAKE_DEMAND(force_output_port_ != nullptr);
    return *force_output_port_;
  }

 private:
  void OutputStateStatus(const systems::Context<double>& context,
                    systems::BasicVector<double>* output) const;

  void OutputForceStatus(const systems::Context<double>& context,
                         systems::BasicVector<double>* output) const;

  /// Event handler of the periodic discrete state update.
  systems::EventStatus UpdateDiscreteState(
      const systems::Context<double>& context,
      systems::DiscreteValues<double>* discrete_state) const;

  const int num_fingers_;
  const int num_joints_;
  const int num_tip_forces_;
  const OutputPort<double>* state_output_port_{};
  const OutputPort<double>* force_output_port_{};
};

/// Creates and outputs lcmt_planar_gripper_status messages.
///
/// This system has two vector-valued input ports containing the
/// current position and velocity (state) as well as fingertip forces (fy, fz).
///
/// This system has one abstract valued output port that contains a
/// Value object templated on type `lcmt_planar_gripper_status`. Note that this
/// system does not actually send this message on an LCM channel. To send the
/// message, the output of this system should be connected to an input port of
/// a systems::lcm::LcmPublisherSystem that accepts a
/// Value object templated on type `lcmt_planar_gripper_status`.
class GripperStatusEncoder : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GripperStatusEncoder)

  /// Constructor.
  /// @param num_joints The total number of fingers used on the planar-gripper.
  explicit GripperStatusEncoder(int num_fingers = kGripperDefaultNumFingers);

  const systems::InputPort<double>& get_state_input_port() const {
    DRAKE_DEMAND(state_input_port_ != nullptr);
    return *state_input_port_;
  }

  const systems::InputPort<double>& get_force_input_port() const {
    DRAKE_DEMAND(force_input_port_ != nullptr);
    return *force_input_port_;
  }

 private:
  // This is the calculator method for the output port.
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_planar_gripper_status* output) const;

  const int num_fingers_;
  const int num_joints_;
  const int num_tip_forces_;
  const InputPort<double>* state_input_port_{};
  const InputPort<double>* force_input_port_{};
};

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
