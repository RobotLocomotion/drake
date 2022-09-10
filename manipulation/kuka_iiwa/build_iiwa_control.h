#pragma once

#include <optional>
#include <string>

#include <Eigen/Dense>

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"

/// @file iiwa controller and controller plant setup
///
/// This class implements the software stack of the Iiwa arm and the LCM-to-FRI
/// adapter installed in it.  The only control mode FRI exposes is
/// position-control with an optional feedforward torque command.  Torque
/// control mode, however, is not available.  The arm receives position commands
/// (`lcmt_iiwa_command`) and emits status (`lcmt_iiwa_status`);  the iiwa
/// controller built here takes care of translating the command into torques on
/// the iiwa joints and the iiwa measured positions and torques into those
/// status messages.  Note that only 7 DoF Iiwa arm is supported.
///
/// In order to do this, the controller maintains an entire separate iiwa
/// plant (*not* the simulated plant!) to perform inverse dynamics
/// computations.  These computations correspond to the servoing and gravity
/// compensation done on the real iiwa; disagreement between the controller
/// model and the simulated model (e.g., introduced via the weld joint error
/// mechanism) represents errors in the servo, end-effector, and
/// gravity-compensation configuration.

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

/// Given a @p plant (and associated @p ids) and a @p builder, install in that
/// builder the systems necessary to control and monitor an iiwa described by
/// @p controller_plant in that plant.
///
/// The installed plant will communicate over the LCM interface @p lcm.
///
/// The installed plant will connect itself to the actuation input port and
/// state output ports in `plant` corresponding to the iiwa model.
///
/// @p desired_iiwa_kp_gains is an optional argument to pass in gains
/// corresponding to the IIWA Dof (7) in the controller.  If no argument is
/// passed, the gains derived from hardware will be used instead (hardcoded
/// within the implementation of this method).
void BuildIiwaControl(
    const multibody::MultibodyPlant<double>& plant,
    const multibody::ModelInstanceIndex iiwa_instance,
    const multibody::MultibodyPlant<double>& controller_plant,
    lcm::DrakeLcmInterface* lcm, systems::DiagramBuilder<double>* builder,
    double ext_joint_filter_tau = 0.01,
    const std::optional<Eigen::VectorXd>& desired_iiwa_kp_gains = std::nullopt);

struct IiwaControlPorts {
  const systems::InputPort<double>* commanded_positions{};
  const systems::InputPort<double>* commanded_feedforward_torque{};
  const systems::OutputPort<double>* joint_torque{};
  const systems::OutputPort<double>* external_torque{};
};

/// A simplified Iiwa controller builder.
/// @sa BuildIiwaControl()
///
/// @return an IiwaControlPorts struct containing the commanded positions input
/// port of the installed controller as well as output ports for the joint and
/// external torques. If @p enable_feedforward_torque is true, the struct also
/// contains the feedforward torque input port.
IiwaControlPorts BuildSimplifiedIiwaControl(
    const multibody::MultibodyPlant<double>& plant,
    const multibody::ModelInstanceIndex iiwa_instance,
    const multibody::MultibodyPlant<double>& controller_plant,
    systems::DiagramBuilder<double>* builder,
    double ext_joint_filter_tau = 0.01,
    const std::optional<Eigen::VectorXd>& desired_iiwa_kp_gains = std::nullopt,
    bool enable_feedforward_torque = false);

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
