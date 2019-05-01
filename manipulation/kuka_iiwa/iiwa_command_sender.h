#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

/// Creates and outputs lcmt_iiwa_command messages.
///
/// Note that this system does not actually send the message an LCM channel. To
/// send the message, the output of this system should be connected to a
/// systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_command>().
///
/// This system has two vector-valued input ports, one for the commanded
/// position (which must be connected) and one for commanded torque (which is
/// optional).  If the torque input port is not connected, then no torque
/// values will be emitted in the resulting message.
///
/// This system has one abstract-valued output port of type lcmt_iiwa_command.
///
/// @system {
///   @input_port{position}
///   @input_port{torque (optional)}
///   @output_port{lcmt_iiwa_command}
/// }
///
/// @see `lcmt_iiwa_command.lcm` for additional documentation.
class IiwaCommandSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaCommandSender)

  explicit IiwaCommandSender(int num_joints = kIiwaArmNumJoints);

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_position_input_port() const;
  const systems::InputPort<double>& get_torque_input_port() const;
  const systems::OutputPort<double>& get_output_port() const;
  //@}

 private:
  void CalcOutput(const systems::Context<double>&, lcmt_iiwa_command*) const;

  const int num_joints_;
};

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
