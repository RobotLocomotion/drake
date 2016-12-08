#pragma once

#include <memory>

#include "drake/examples/bouncing_ball/ball.h"

namespace drake {
namespace bouncing_ball {

/// Dynamical representation of the idealized hybrid dynamics
/// of a ball dropping from a height and bouncing on a surface.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
///
/// To use other specific scalar types see bouncing_ball-inl.h.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// They are already available to link against in drakeBouncingBall.
///
/// Inputs: no inputs.
/// States: vertical position (state index 0) and velocity (state index 1) in
/// units of m and m/s, respectively.
/// Outputs: vertical position (state index 0) and velocity (state index 1) in
/// units of m and m/s, respectively.
template <typename T>
class BouncingBall : public Ball<T> {
 public:
  /// Constructor for the BouncingBall system.
  BouncingBall();

  void DoCalcNextUpdateTime(const systems::Context<T>& context,
                            systems::UpdateActions<T>* actions) const override;

  void DoPerformUnrestrictedUpdate(systems::Context<T>* context) const override;

  /// Calculates the closed form solution of the height of the bouncing ball
  /// at the given time, assuming that the initial velocity is zero. Only works
  /// for restricted restitution cases { 0, 1 }.
  /// @p x0 the height of the ball at the initial time.
  /// @p tf the desired time for which the height should be returned.
  /// @returns the height of the ball at time tf.
  /// @throws std::logic_error if restitution is a value other than zero or one.
  T CalcClosedFormHeight(const T x0, const T tf);

  /// Calculates the closed form solution of the velocity of the bouncing ball
  /// at the given time, assuming that the initial velocity is zero. Only works
  /// for restricted restitution cases { 0, 1 }.
  /// @p x0 the height of the ball at the initial time.
  /// @p tf the desired time for which the velocity should be returned.
  /// @returns the velocity of the ball at time tf. *If tf corresponds to an
  ///          impact time, the velocity of the ball after the collision will
  ///          be returned.
  /// @throws std::logic_error if restitution is a value other than zero or one.
  T CalcClosedFormVelocity(const T x0, const T tf);

  /// TODO(jadecastro): This is a prototype implementation to be overridden from
  /// the system API, pending further discussions.
  ///
  /// Evaluate the guard function associated with the system in a particular
  /// mode. If the EvalGuard returns a non-positive value, then the hybrid
  /// system is allowed to make a transition from the `pre` mode to `post` mode.
  T EvalGuard(const systems::Context<T>& context) const;

  /// TODO(jadecastro): This is a prototype implementation to be overridden from
  /// the system API, pending further discussions.
  ///
  /// Performs a reset mapping that occurs if and only if a mode transition
  /// (discrete jump) has been made. It does so by mutating the context so that,
  /// by default, the reset mapping is the identity mapping.
  void PerformReset(systems::Context<T>* context) const;

  /// Getter for the coefficient of restitution for this model.
  double get_restitution_coef() const { return restitution_coef_; }

 private:
  const double restitution_coef_ = 1.0;  // Coefficient of restitution.

  // Numerically intolerant signum function.
  static T sgn(T x) {
    if (x > 0.0)
      return 1.0;
    else if (x < 0.0)
      return -1.0;
    else
      return 0.0;
  }
};

}  // namespace bouncing_ball
}  // namespace drake
