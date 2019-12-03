#pragma once

/// @file
/// This file contains classes dealing with sending/receiving
/// LCM messages related to the planar gripper.
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

// By default the planar gripper has 3 fingers.
constexpr int kGripperDefaultNumFingers = 3;

// This is rather arbitrary, for now.
constexpr double kGripperLcmStatusPeriod = 0.010;

/// Handles lcmt_planar_gripper_command messages from a LcmSubscriberSystem.
///
/// This system has one abstract valued input port which expects a
/// Value object templated on type `lcmt_planar_gripper_command`.
///
/// This system has a single output port which reports the commanded position
/// and velocity for all joints.
class GripperCommandDecoder : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GripperCommandDecoder)

  /// @param num_joints The total number of joints across all fingers.
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
};

/// Creates and outputs lcmt_planar_gripper_command messages
///
/// This system has one vector-valued input port containing the
/// desired position and velocity or torque, depending on the mode of control.
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

  /// @param num_joints The total number of joints across all fingers.
  explicit GripperCommandEncoder(int num_fingers = kGripperDefaultNumFingers);

 private:
  void OutputCommand(const systems::Context<double>& context,
                     lcmt_planar_gripper_command* output) const;

  const int num_fingers_;
  const int num_joints_;
};

/// Handles lcmt_planar_gripper_status messages from a LcmSubscriberSystem.
///
/// This system has one abstract valued input port which expects a
/// Value object templated on type `lcmt_planar_gripper_status`.
///
/// This system has one vector valued output port which reports
/// measured position and velocity for each joint.
///
/// All ports will continue to output their initial state (typically
/// zero) until a message is received.
class GripperStatusDecoder : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GripperStatusDecoder)

  /// @param num_joints The total number of joints across all fingers.
  explicit GripperStatusDecoder(int num_fingers = kGripperDefaultNumFingers);

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
};

/// Creates and outputs lcmt_planar_gripper_status messages.
///
/// This system has one vector-valued input port containing the
/// current position and velocity.
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

  /// @param num_joints The total number of joints across all fingers.
  explicit GripperStatusEncoder(int num_fingers = kGripperDefaultNumFingers);

 private:
  // This is the method to use for the output port allocator.
  lcmt_planar_gripper_status MakeOutputStatus() const;

  // This is the calculator method for the output port.
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_planar_gripper_status* output) const;

  const int num_fingers_;
  const int num_joints_;
};

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
