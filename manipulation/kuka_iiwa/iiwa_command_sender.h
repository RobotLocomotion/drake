#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

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

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
