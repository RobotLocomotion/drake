#pragma once

#include <limits>
#include <vector>

#include "drake/common/trajectories/piecewise_polynomial.h"
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

/**
 * This systems smoothes the position command for the Schunk WSG gripper.
 *
 * @system{ SchunkWsgPositionCommandInterpolator,
 *  @input_port{position_command}
 *  @input_port{state_command} }
 *
 * The `position_command` input port is a scalar, and the 'state_command'
 * output is vector of 2. This system is modeled as a discrete time system
 * that pulls the input port at a constant rate. The command trajectory
 * is stored in the discrete state. Whenever an input change larger some
 * threshold is detected in the discrete update, a new trajectory is
 * generated and stored in the discrete state. The output is generated by
 * interpolating the trajectory that's stored in the context.
 *
 * The trajectory follow a trapezoid profile. Depending on the distance that
 * needs to be covered, it can be either one of:
 * - max acceleration then max deceleration
 * - max acceleration, max velocity then max deceleration.
 */
class SchunkWsgPositionCommandInterpolator
    : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgPositionCommandInterpolator)

  /// The minimum change between the last received command and the
  /// current command to trigger a trajectory update.  Based on
  /// manually driving the actual gripper using the web interface, it
  /// appears that it will at least attempt to respond to commands as
  /// small as 0.1mm.
  static constexpr double kTargetEpsilon = 0.0001;

  /// The acceleration and velocity limits correspond to the maximum
  /// values available for manual control through the gripper's web
  /// interface. In SI units.
  static constexpr double kMaxVelocity = 0.42;
  static constexpr double kMaxAccel = 5.;

  /**
   * @param time_step Period for discrete update.
   */
  explicit SchunkWsgPositionCommandInterpolator(double time_step);

  /**
   * Set the internal state's trajectory a constant one at @p target.
   */
  void SetConstantTrajectory(systems::Context<double>* context,
                             double target) const;

  const systems::InputPort<double>& get_position_command_input_port() const {
    return GetInputPort("position_command");
  }

  const systems::OutputPort<double>&
  get_interpolated_state_command_output_port() const {
    return GetOutputPort("state_command");
  }

 private:
  // This is a very specific workaround #10149. What I really want to store in
  // the discrete state is the trajectory itself, but that's only possible
  // through UnrestrictedUpdate, which has severe performance penalty at the
  // moment for complex system (e.g. manipulation station). So instead, I am
  // hacking this around by storing the parameters that are used to generate
  // the trajectory in the discrete state, and reconstructing the trajectory
  // in the calc output functions.
  template <typename T>
  class TrapezoidTrajAsVector : public systems::BasicVector<T> {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrapezoidTrajAsVector)

    // The trajectory have 2 or 3 linear segments, and is represented by a
    // vector of 8, the first 4 numbers are dedicated to time (breaks), and
    // the last 4 are for knots. The 2 segments version represents a motion
    // profile of constant acceleration then constant deceleration, whereas
    // the 3 segments represents constant acceleration, constant speed then
    // constant deceleration. The two versions are distinguished by the 4th
    // element of the underlying vector: infinity signals a 2 segments motion.
    TrapezoidTrajAsVector() : systems::BasicVector<T>(8) {}

    // Returns the position at the end of the trajectory.
    const T end_target() const {
      return std::isfinite(this->GetAtIndex(3)) ? this->GetAtIndex(7)
                                                : this->GetAtIndex(6);
    }

    // Sets the times and knots. @p times and @p knots must have equal length,
    // and can only be size 3 or 4. @p times must be in strict increasing order.
    void set_times_and_knots(const VectorX<T>& times, const VectorX<T>& knots) {
      DRAKE_THROW_UNLESS(times.size() == 3 || times.size() == 4);
      DRAKE_THROW_UNLESS(times.size() == knots.size());
      this->values().segment(0, times.size()) = times;
      this->values().segment(4, knots.size()) = knots;
      if (times.size() == 3) {
        this->values()[3] = std::numeric_limits<T>::infinity();
        this->values()[7] = std::numeric_limits<T>::infinity();
      }
    }

    // Returns a piecewise polynomial for the position.
    trajectories::PiecewisePolynomial<T> to_trajectory() const {
      std::vector<double> times;
      std::vector<MatrixX<T>> knots;
      const int size = std::isfinite(this->GetAtIndex(3)) ? 4 : 3;
      for (int i = 0; i < size; i++) {
        times.push_back(this->GetAtIndex(i));
        knots.push_back(Vector1<T>(this->GetAtIndex(i + 4)));
      }
      return trajectories::PiecewisePolynomial<T>::FirstOrderHold(times, knots);
    }

   private:
    TrapezoidTrajAsVector<T>* DoClone() const override {
      return new TrapezoidTrajAsVector<T>();
    }
  };

  void CalcTrapzoidTrajParams(double time, double cur_position,
                              double target_position,
                              TrapezoidTrajAsVector<double>* traj) const;

  void CalcInterpolatedStateCommandOutput(
      const systems::Context<double>& context,
      systems::BasicVector<double>* output_vector) const;

  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>& events,
      systems::DiscreteValues<double>* discrete_state) const override;
};

/// This class implements a controller for a Schunk WSG gripper in position
/// control mode. It is a thin wrapper around SchunkWsgPdController, where the
/// position command is smoothed first by SchunkWsgPositionCommandInterpolator
/// before sent to SchunkWsgPdController.
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
  void set_initial_position(systems::Context<double>* context,
                                    double desired_position) const;

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
  SchunkWsgPositionCommandInterpolator* command_interpolator_;

  systems::InputPortIndex desired_position_input_port_{};
  systems::InputPortIndex force_limit_input_port_{};
  systems::InputPortIndex state_input_port_{};
  systems::OutputPortIndex generalized_force_output_port_{};
  systems::OutputPortIndex grip_force_output_port_{};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
