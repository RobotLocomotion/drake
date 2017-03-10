#include "drake/automotive/smooth_acceleration_function.h"

#include <algorithm>
#include <string>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/cond.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
using symbolic::Expression;

namespace automotive {

template <typename T>
T calc_smooth_acceleration(T desired_acceleration, T current_velocity,
    T max_velocity, T velocity_limit_kp) {
  using std::max;
  using std::min;
  using std::tanh;

  // If our current velocity is out of bounds, insist on damping that brings us
  // back toward the limit, but allow for the desired_acceleration to win if it
  // is stronger than the damping and has the desired sign.
  const T underspeed = 0 - current_velocity;
  const T overspeed = current_velocity - max_velocity;
  const T damped_acceleration = cond(
      // If velocity is too low, use positive damping or desired_acceleration.
      underspeed > 0,
      max(desired_acceleration, T(velocity_limit_kp * underspeed)),
      // If velocity is too high, use negative damping or desired_acceleration.
      overspeed > 0,
      min(desired_acceleration, T(-velocity_limit_kp * overspeed)),
      // Velocity is within limits.
      desired_acceleration);

  // TODO(jwnimmer-tri) Declare witness functions for the above conditions,
  // once the framework support is in place.  Until then, smooth out the
  // acceleration using tanh centered around the limit we are headed towards
  // (max speed when accelerating; zero when decelerating).  The smoothing
  // constant within the tanh is arbitrary and un-tuned.
  const T relevant_limit = cond(
      damped_acceleration >= 0, max_velocity, T(0));
  const T smoothing_factor =
      pow(tanh(20.0 * (current_velocity - relevant_limit)), 2);
  return damped_acceleration * smoothing_factor;
}

// These instantiations must match the API documentation in
// smooth_acceleration_function.h.
template double calc_smooth_acceleration<double>(
    double desired_acceleration,
    double current_velocity,
    double max_velocity,
    double velocity_limit_kp);
#if EIGEN_VERSION_AT_LEAST(3, 2, 93)
template drake::AutoDiffXd calc_smooth_acceleration<drake::AutoDiffXd>(
    drake::AutoDiffXd desired_acceleration,
    drake::AutoDiffXd current_velocity,
    drake::AutoDiffXd max_velocity,
    drake::AutoDiffXd velocity_limit_kp);
#endif
template Expression calc_smooth_acceleration<Expression>(
    Expression desired_acceleration,
    Expression current_velocity,
    Expression max_velocity,
    Expression velocity_limit_kp);

}  // namespace automotive
}  // namespace drake
