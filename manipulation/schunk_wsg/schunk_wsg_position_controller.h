#pragma once

#include <vector>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/discrete_derivative.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

/// This class implements a controller for a Schunk WSG gripper in position
/// control mode.  It assumes that the gripper is modeled in the plant as two
/// independent prismatic joints for the fingers.
///
/// Note: This is intended as a simpler single-system implementation that can
/// replace the SchunkWsgController when using position control mode.  We
/// anticipate a single-system SchunkWsgForceController implementation to
/// (soon) replace the other mode, and then will deprecate SchunkWsgController.
///
/// Call the positions of the prismatic joints q₀ and q₁.  q₀ = q₁ = 0 is
/// the configuration where the fingers are touching in the center.  When the
/// gripper is open, q₀ < 0 and q₁ > 0.
///
/// The physical gripper mechanically imposes that -q₀ = q₁, and implements a
/// controller to track -q₀ = q₁ = q_d/2 (q_d is the desired position, which
/// is the signed distance between the two fingers).  We model that here with
/// two PD controllers -- one that implements the physical constraint
/// (keeping the fingers centered):
///   f₀+f₁ = -kp_constraint*(q₀+q₁) - kd_constraint*(v₀+v₁),
/// and another to implement the controller (opening/closing the fingers):
///   -f₀+f₁ = sat(kp_command*(q_d + q₀ - q₁) + kd_command*(v_d + v₀ - v₁)),
/// where sat() saturates the command to be in the range [-force_limit,
/// force_limit].  The expectation is that
///   kp_constraint ≫ kp_command.
///
/// @system{ SchunkWSGPdController,
///   @input_port{desired_state}
///   @input_port{force_limit}
///   @input_port{state},
///   @output_port{generalized_force}
///   @output_port{grip_force} }
///
/// The desired_state is a BasicVector<double> of size 2 (position and
/// velocity of the distance between the fingers).  The force_limit is a
/// scalar (BasicVector<double> of size 1).  The state is a
/// BasicVector<double> of size 4 (positions and velocities of the two
/// fingers).  The output generalized_force is a BasicVector<double> of size
/// 2 (generalized force inputs to the two fingers).  The output grip_force is
/// a scalar surrogate for the force measurement from the driver,
/// f = abs(f₀-f₁) which, like the gripper itself, only reports a positive
/// force.
///
class SchunkWsgPdController : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgPdController)

  /// Initialize the controller.  The gain parameters are set based
  /// limited tuning in simulation with a kuka picking up small objects.
  // Note: These default parameter values came from the previous version of
  // the controller, except that kp_command is decreased by 10x.  Setting
  // kd_constraint = 50.0 resulted in some numerical instability in existing
  // kuka tests.  They could be tuned in better with more simulation effort.
  SchunkWsgPdController(double kp_command = 200.0, double kd_command = 5.0,
                        double kp_constraint = 2000.0,
                        double kd_constraint = 5.0);

  const systems::InputPort<double>& get_desired_state_input_port() const {
    return get_input_port(desired_state_input_port_);
  }

  const systems::InputPort<double>& get_force_limit_input_port() const {
    return get_input_port(force_limit_input_port_);
  }

  const systems::InputPort<double>& get_state_input_port() const {
    return get_input_port(state_input_port_);
  }

  const systems::OutputPort<double>& get_generalized_force_output_port() const {
    return get_output_port(generalized_force_output_port_);
  }

  const systems::OutputPort<double>& get_grip_force_output_port() const {
    return get_output_port(grip_force_output_port_);
  }

 private:
  Eigen::Vector2d CalcGeneralizedForce(
      const systems::Context<double>& context) const;

  void CalcGeneralizedForceOutput(
      const systems::Context<double>& context,
      systems::BasicVector<double>* output_vector) const;

  void CalcGripForceOutput(const systems::Context<double>& context,
                           systems::BasicVector<double>* output_vector) const;

  const double kp_command_;
  const double kd_command_;
  const double kp_constraint_;
  const double kd_constraint_;

  systems::InputPortIndex desired_state_input_port_{};
  systems::InputPortIndex force_limit_input_port_{};
  systems::InputPortIndex state_input_port_{};
  systems::OutputPortIndex generalized_force_output_port_{};
  systems::OutputPortIndex grip_force_output_port_{};
};

/// This class implements a controller for a Schunk WSG gripper in position
/// control mode adding a discrete-derivative to estimate the desired
/// velocity from the desired position commands.  It is a thin wrapper
/// around SchunkWsgPdController.
///
/// @system{ SchunkWSGPositionController,
///   @input_port{desired_position}
///   @input_port{force_limit}
///   @input_port{state},
///   @output_port{generalized_force}
///   @output_port{grip_force} }
///
/// @see SchunkWsgPdController
class SchunkWsgPositionController : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgPositionController)

  /// Initialize the controller.  The default @p time_step is set to match
  /// the update rate of the wsg firmware.  The gain parameters are set based
  /// limited tuning in simulation with a kuka picking up small objects.
  /// @see SchunkWsgPdController::SchunkWsgPdController()
  SchunkWsgPositionController(double time_step = 0.05,
                              double kp_command = 200.0,
                              double kd_command = 5.0,
                              double kp_constraint = 2000.0,
                              double kd_constraint = 5.0);

  // The controller stores the last commanded desired position as state.
  // This is a helper method to reset that state.
  void set_initial_position(systems::State<double>* state,
                            double desired_position) const;

  // The controller stores the last commanded desired position as state.
  // This is a helper method to reset that state.
  void set_initial_position(systems::Context<double>* context,
                            double desired_position) const {
    set_initial_position(&context->get_mutable_state(), desired_position);
  }

  const systems::InputPort<double>& get_desired_position_input_port() const {
    return get_input_port(desired_position_input_port_);
  }

  const systems::InputPort<double>& get_force_limit_input_port() const {
    return get_input_port(force_limit_input_port_);
  }

  const systems::InputPort<double>& get_state_input_port() const {
    return get_input_port(state_input_port_);
  }

  const systems::OutputPort<double>& get_generalized_force_output_port() const {
    return get_output_port(generalized_force_output_port_);
  }

  const systems::OutputPort<double>& get_grip_force_output_port() const {
    return get_output_port(grip_force_output_port_);
  }

 private:
  systems::StateInterpolatorWithDiscreteDerivative<double>* state_interpolator_;

  systems::InputPortIndex desired_position_input_port_{};
  systems::InputPortIndex force_limit_input_port_{};
  systems::InputPortIndex state_input_port_{};
  systems::OutputPortIndex generalized_force_output_port_{};
  systems::OutputPortIndex grip_force_output_port_{};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
