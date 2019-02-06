#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

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

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
