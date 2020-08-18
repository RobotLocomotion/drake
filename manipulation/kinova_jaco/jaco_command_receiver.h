#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_jaco_command.hpp"
#include "drake/manipulation/kinova_jaco/jaco_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {

/// Handles lcmt_jaco_command message from a LcmSubscriberSystem.
///
/// Note that this system does not actually subscribe to an LCM channel. To
/// receive the message, the input of this system should be connected to a
/// LcmSubscriberSystem::Make<drake::lcmt_jaco_command>().
///
/// It has one input port, "lcmt_jaco_command".
///
/// This system has a single output port which contains the commanded position
/// and velocity for each joint.  Finger velocities will be translated from
/// the values used by the Kinova SDK to values appropriate for the finger
/// joints in the Jaco description (see jaco_constants.h).
///
/// @system
/// name: JacoCommandReceiver
/// input_ports:
/// - lcmt_jaco_command
/// output_ports:
/// - state
/// @endsystem
class JacoCommandReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JacoCommandReceiver)

  JacoCommandReceiver(int num_joints = kJacoDefaultArmNumJoints,
                      int num_fingers = kJacoDefaultArmNumFingers);

  /// Sets the initial commanded position of the controlled jaco prior to any
  /// command messages being received.  If this function is not called, the
  /// starting position will be the zero configuration.  Finger positions
  /// should be specified as values appropriate for the Jaco description (see
  /// jaco_constants.h), not in Kinova SDK values.
  void set_initial_position(
      systems::Context<double>* context,
      const Eigen::Ref<const Eigen::VectorXd>& q) const;

 private:
  Eigen::VectorXd input_state(const systems::Context<double>&) const;
  void CalcInput(const systems::Context<double>&, lcmt_jaco_command*) const;

  const int num_joints_;
  const int num_fingers_;
  const systems::CacheEntry* groomed_input_{};
};

}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
