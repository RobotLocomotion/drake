#pragma once

#include <optional>
#include <string>

#include <Eigen/Dense>

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/multibody/plant/multibody_plant.h"

/// @file
/// Iiwa controller and controller plant setup
///
/// This class implements the software stack of the Iiwa arm and the LCM-to-FRI
/// adapter installed in it as it is exposed in
/// https://github.com/RobotLocomotion/drake-iiwa-driver.
/// The driver in this repository exposes three options:
///   - position-only control,
///   - position-and-torque control, where feedforward torque command is
///     optional, and
///   - torque-only control, added with nominal gravity compensation.
/// The arm receives position and/or torque commands (`lcmt_iiwa_command`) and
/// emits status (`lcmt_iiwa_status`); the Iiwa controller built here takes
/// care of translating the command into torques on the Iiwa joints and the
/// Iiwa measured positions and torques into those status messages.  Note that
/// only the 7 DoF Iiwa arm is supported.
///
/// A simulated controller maintains an entire separate Iiwa plant (*not* the
/// simulated plant!) to perform inverse dynamics computations.  These
/// computations correspond to the servoing and gravity compensation done on
/// the real Iiwa; disagreement between the controller model and the simulated
/// model represents errors in the servo, end-effector, and
/// gravity-compensation configuration.

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

/// Given a @p plant (and associated @p iiwa_instance) and a @p builder,
/// installs in that builder the systems necessary to control and monitor an
/// Iiwa described by @p controller_plant in that plant.
///
/// The installed plant will communicate over the LCM interface @p lcm.
///
/// The installed plant will connect itself to the actuation input port and
/// state output ports in `plant` corresponding to the Iiwa model.
///
/// @p desired_iiwa_kp_gains is an optional argument to pass in gains
/// corresponding to the Iiwa Dof (7) in the controller.  If no argument is
/// passed, the gains derived from hardware will be used instead (hardcoded
/// within the implementation of this function). These gains must be nullopt
/// if @p control_mode does not include position control.
///
/// @p control_mode the control mode for the controller.
///
/// Note: The Diagram will maintain an internal reference to `controller_plant`,
/// so you must ensure that `controller_plant` has a longer lifetime than the
/// Diagram.
void BuildIiwaControl(
    const multibody::MultibodyPlant<double>& plant,
    const multibody::ModelInstanceIndex iiwa_instance,
    const multibody::MultibodyPlant<double>& controller_plant,
    lcm::DrakeLcmInterface* lcm, systems::DiagramBuilder<double>* builder,
    double ext_joint_filter_tau = 0.01,
    const std::optional<Eigen::VectorXd>& desired_iiwa_kp_gains = std::nullopt,
    IiwaControlMode control_mode = IiwaControlMode::kPositionAndTorque);

/// The return type of BuildSimplifiedIiwaControl(). Depending on the
/// `control_mode`, some of the input ports might be null. The output ports are
/// never null.
struct IiwaControlPorts {
  /// This will be non-null iff the control_mode denotes commanded positions.
  const systems::InputPort<double>* commanded_positions{};
  /// This will be non-null iff the control_mode denotes commanded torques.
  const systems::InputPort<double>* commanded_torque{};

  const systems::OutputPort<double>* position_commanded{};
  const systems::OutputPort<double>* position_measured{};
  const systems::OutputPort<double>* velocity_estimated{};
  const systems::OutputPort<double>* joint_torque{};  // aka torque_commanded
  const systems::OutputPort<double>* torque_measured{};
  const systems::OutputPort<double>* external_torque{};  // aka torque_external
};

/// A simplified Iiwa controller builder to construct an
/// InverseDynamicsController without adding LCM I/O systems.
/// @sa BuildIiwaControl()
IiwaControlPorts BuildSimplifiedIiwaControl(
    const multibody::MultibodyPlant<double>& plant,
    const multibody::ModelInstanceIndex iiwa_instance,
    const multibody::MultibodyPlant<double>& controller_plant,
    systems::DiagramBuilder<double>* builder,
    double ext_joint_filter_tau = 0.01,
    const std::optional<Eigen::VectorXd>& desired_iiwa_kp_gains = std::nullopt,
    IiwaControlMode control_mode = IiwaControlMode::kPositionAndTorque);

namespace internal {

// Adds a SimIiwaDriver system to the given `builder` and connects its plant-
// related input and output ports to `plant`. Returns the newly-added System.
// This is indirectly unit tested via BuildSimplifiedIiwaControl.
const systems::System<double>& AddSimIiwaDriver(
    const multibody::MultibodyPlant<double>& plant,
    const multibody::ModelInstanceIndex iiwa_instance,
    const multibody::MultibodyPlant<double>& controller_plant,
    systems::DiagramBuilder<double>* builder, double ext_joint_filter_tau,
    const std::optional<Eigen::VectorXd>& desired_iiwa_kp_gains,
    IiwaControlMode control_mode);

}  // namespace internal
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
