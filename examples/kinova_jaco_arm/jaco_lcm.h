#pragma once

/// @file
/// This file contains classes dealing with sending/receiving
/// LCM messages related to the jaco arm.
///
/// All (q, v) state vectors in this file are of the format
/// (joint_positions, finger_positions, joint_velocities,
/// finger_velocities).

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_jaco_command.hpp"
#include "drake/lcmt_jaco_status.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace kinova_jaco_arm {

/// The LCM system classes for the Jaco default to a 7dof model with 3
/// fingers.  Different configurations are supported by passing the
/// proper arguments to the system constructors.
constexpr int kJacoDefaultArmNumJoints = 7;
constexpr int kJacoDefaultArmNumFingers = 3;

/// Kinova says 100Hz is the proper frequency for joint velocity
/// updates.  See
/// https://github.com/Kinovarobotics/kinova-ros#velocity-control-joint-space-and-cartesian-space
constexpr double kJacoLcmStatusPeriod = 0.010;

/// Handles lcmt_jaco_command messages from a LcmSubscriberSystem.
///
/// This system has one abstract valued input port which expects a
/// systems::Value object templated on type `lcmt_jaco_command`.
///
/// This system has a single output port which publishes the commanded
/// position and velocity for each joint.
class JacoCommandReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JacoCommandReceiver)

  /// @param num_joints The number of joints on the arm (typically 6
  /// or 7).
  ///
  /// @param num_fingers The number of fingers on the gripper
  /// (typically 2 or 3).
  explicit JacoCommandReceiver(int num_joints = kJacoDefaultArmNumJoints,
                               int num_fingers = kJacoDefaultArmNumFingers);

  /// Sets the initial position of the controlled jaco prior to any
  /// commands being received.  @p x contains the starting position.
  /// This position will be the commanded position (with zero
  /// velocity) until a position message is received.  If this
  /// function is not called, the starting position will be the zero
  /// configuration.
  void set_initial_position(
      systems::Context<double>* context,
      const Eigen::Ref<const VectorX<double>> x) const;

 private:
  void OutputCommand(const systems::Context<double>& context,
                     systems::BasicVector<double>* output) const;

  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
      systems::DiscreteValues<double>* discrete_state) const override;

 private:
  const int num_joints_;
  const int num_fingers_;
};

/// Creates and outputs lcmt_jaco_command messages
///
/// This system has one vector-valued input port containing the
/// desired position and velocity.
///
/// This system has one abstract valued output port that contains a
/// systems::Value object templated on type `lcmt_jaco_command`. Note
/// that this system does not actually send this message on an LCM
/// channel. To send the message, the output of this system should be
/// connected to an input port of a systems::lcm::LcmPublisherSystem
/// that accepts a systems::Value object templated on type
/// `lcmt_jaco_command`.
class JacoCommandSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JacoCommandSender)

  /// @param num_joints The number of joints on the arm (typically 6
  /// or 7).
  ///
  /// @param num_fingers The number of fingers on the gripper
  /// (typically 2 or 3).
  explicit JacoCommandSender(int num_joints = kJacoDefaultArmNumJoints,
                             int num_fingers = kJacoDefaultArmNumFingers);

 private:
  void OutputCommand(const systems::Context<double>& context,
                     lcmt_jaco_command* output) const;

  const int num_joints_;
  const int num_fingers_;
};

/// Handles lcmt_jaco_status messages from a LcmSubscriberSystem.
///
/// This system has one abstract valued input port which expects a
/// systems::Value object templated on type `lcmt_jaco_status`.
///
/// This system has one vector valued output port which reports
/// measured position and velocity for each joint and finger.
///
/// All ports will continue to output their initial state (typically
/// zero) until a message is received.
class JacoStatusReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JacoStatusReceiver)

  /// @param num_joints The number of joints on the arm (typically 6
  /// or 7).
  ///
  /// @param num_fingers The number of fingers on the gripper
  /// (typically 2 or 3).
  explicit JacoStatusReceiver(int num_joints = kJacoDefaultArmNumJoints,
                              int num_fingers = kJacoDefaultArmNumFingers);

 private:
  void OutputStatus(const systems::Context<double>& context,
                    systems::BasicVector<double>* output) const;

  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
      systems::DiscreteValues<double>* discrete_state) const override;

  const int num_joints_;
  const int num_fingers_;
};

/// Creates and outputs lcmt_jaco_status messages.
///
/// This system has one vector-valued input port containing the
/// current position and velocity.
///
/// This system has one abstract valued output port that contains a
/// systems::Value object templated on type `lcmt_jaco_status`. Note that this
/// system does not actually send this message on an LCM channel. To send the
/// message, the output of this system should be connected to an input port of
/// a systems::lcm::LcmPublisherSystem that accepts a
/// systems::Value object templated on type `lcmt_jaco_status`.
class JacoStatusSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JacoStatusSender)

  /// @param num_joints The number of joints on the arm (typically 6
  /// or 7).
  ///
  /// @param num_fingers The number of fingers on the gripper
  /// (typically 2 or 3).
  explicit JacoStatusSender(int num_joints = kJacoDefaultArmNumJoints,
                            int num_fingers = kJacoDefaultArmNumFingers);

 private:
  // This is the method to use for the output port allocator.
  lcmt_jaco_status MakeOutputStatus() const;

  // This is the calculator method for the output port.
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_jaco_status* output) const;

  const int num_joints_;
  const int num_fingers_;
};

}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake
