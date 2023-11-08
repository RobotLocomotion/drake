#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

/// This class implements a controller for a Schunk WSG gripper.  It
/// has two input ports: lcmt_schunk_wsg_command message and the current
/// state, and an output port which emits the target force for the actuated
/// finger. Note, only one of the command input ports should be connected,
/// However, if both are connected, the message input will be ignored.
/// The internal implementation consists of a PID controller (which controls
/// the target position from the command message) combined with a saturation
/// block (which applies the force control from the command message).
///
/// @system
/// name: SchunkWsgController
/// input_ports:
/// - state
/// - command_message
/// output_ports:
/// - force
/// @endsystem
///
/// @ingroup manipulation_systems
class SchunkWsgController : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgController)

  // The gains here are somewhat arbitrary.  The goal is to make sure
  // that the maximum force is generated except when very close to the
  // target.
  explicit SchunkWsgController(double kp = 2000.0, double ki = 0.0,
                               double kd = 5.0);
};

/// This class implements a desired state controller for a Schunk WSG gripper.
/// It has two input ports: lcmt_schunk_wsg_command message and the current
/// state, and an output port which emits the desired state for the actuated
/// finger. The desired state output port is designed to be connect to the
/// desired state input port of MultibodyPlant for the model instance
/// corresponding to the controlled Schunk gripper. This controller is intended
/// for a Schunk gripper modeled with 1 actuated dof and 1 unactuated dof.
/// The unactuated dof constrained to follow the actuated DoF with a coupler
/// constraint, either added programmatically with
/// MultibodyPlant::AddCouplerConstraint() or in URDF through the use of the
/// <mimic> tag.
/// @see drake::multibody::MultibodyPlant::get_desired_state_input_port().
///
/// Note: This controller is intended only for position control, and will
///       thus ignore the `force` component of the command message.
///
/// @system
/// name: SchunkWsgDesiredStateController
/// input_ports:
/// - state
/// - command_message
/// output_ports:
/// - desired_state
/// @endsystem
///
/// @ingroup manipulation_systems
class SchunkWsgDesiredStateController : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgDesiredStateController)

  SchunkWsgDesiredStateController();
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
