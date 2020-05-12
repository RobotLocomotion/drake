#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_jaco_command.hpp"
#include "drake/manipulation/kinova_jaco/jaco_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {

// TODO(sammy-tri) Add support for not sending finger commands.

/// Creates and outputs lcmt_jaco_command messages.
///
/// Note that this system does not actually send the message to an LCM
/// channel. To send the message, the output of this system should be
/// connected to a
/// systems::lcm::LcmPublisherSystem::Make<lcmt_jaco_command>().
///
/// This system has one vector-valued input port containing the desired
/// position and velocity.  Finger velocities will be translated to the values
/// used by the Kinova SDK from values appropriate for the finger joints in
/// the Jaco description (see jaco_constants.h).
///
/// This system has one abstract-valued output port of type lcmt_jaco_command.
///
/// @system {
///   @input_port{state}
///   @output_port{lcmt_jaco_command}
/// }
///
/// @see `lcmt_jaco_command.lcm` for additional documentation.
class JacoCommandSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JacoCommandSender)

  JacoCommandSender(int num_joints = kJacoDefaultArmNumJoints,
                    int num_fingers = kJacoDefaultArmNumFingers);

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_input_port() const;
  const systems::OutputPort<double>& get_output_port() const;
  //@}

 private:
  void CalcOutput(const systems::Context<double>&, lcmt_jaco_command*) const;

  const int num_joints_;
  const int num_fingers_;
};

}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
