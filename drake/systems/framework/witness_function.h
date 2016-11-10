#pragma once

namespace drake {
namespace systems {

/**
 * Abstract class for representing witness functions used to identify key
 * times or states in a simulator.
 *
 * A simulator can conceivably integrate over a large interval of time in
 * a single leap. Witness functions can be used to cause the simulator to
 * identify, and even pause at, times or states of interest. For example,
 * a witness function could be used to determine when a flight simulator should
 * swap terrain maps as an aircraft crosses between the U.S. and Canada.
 *
 * Alternatively, a common use of witness functions is to determine when
 * a differential algebraic equation (DAE) solver should change its set of
 * active constraints. Formally, a differential algebraic equation (DAE) takes
 * the form:
 * <pre>
 * dx/dt = f(x, t)
 *  g(x) = 0
 * </pre>
 * Accurately solving multi-rigid body systems with unilateral constraints
 * (e.g., joint limits, contact constraints) requires determining when these
 * constraints change. For example, a rigid, light wing resting on planar
 * ground will be subject to contact forces up to the point that a breeze
 * lifts the wing into the air. In the "resting on ground" case, `g(x) = 0`
 * could represent that the acceleration normal to the ground should be zero. A
 * witness function could help the DAE solver determine the time at which
 * `g(x) = 0` should no longer apply, which in this example would allow the wing
 * to undergo ballistic movement.
 *
 * A classical example of a DAE that can make use of a witness function is
 * the bouncing ball system, which switches between two sets of continuous
 * equations of motion: one when the ball is in ballistic flight and one when
 * the ball is in sustained contact with the ground. Witness functions help
 * a DAE determine when to switch between these sets of equations. 
 *
 * If we modify
 * this example slightly by allowing a strong wind to push the ball upward
 * intermittently, a witness function could be used to determine when contact
 * forces should not be applied (i.e., when the force between the ball and the
 * ground, applied normal to the ground, transitions from a compressive force
 * to a tensile force).
 *
 */
template <class T>
class WitnessFunction {
 public:
  /// Constructs this witness function with a reference to the specified system.
  WitnessFunction(const System<T>& system) : system_(system) {}

  /// Two possible types of witness function output types.
  enum WitnessFunctionType { kContinuous, kDiscontinuous };

  /// Directions of interest for zero crossings.
  enum DirectionType {
    /**
     * Zero crossings of interest are for both increasing (rising) and
     * decreasing (falling) witness functions.
     */
    kRisingAndFalling,

    /**
     * Zero crossings of interest are for increasing (rising) witness functions
     * only.
     */
    kRising,

    /**
     * Zero crossings of interest are for decreasing (falling) witness functions
     * only.
     */
    kFalling
  };

  /**
   * Calculates a conservative estimate of the next event time from the given
   * context. The default implementation returns zero (no estimate). If
   * a positive estimate is provided, the estimate of the event time (`te`) is
   * to be treated as conservative: the event cannot occur before future time
   * `te` has arrived.
   */
  virtual double CalcConservativeNextEventTime(const Context<T>& t) {
    return 0.0; }

  /**
   * Gets the direction of interest for this witness function. The direction
   * of interest can indicate that the event should trigger only on positive to
   * negative transitions (kFalling), only on negative to positive transitions
   * (kRising), or any time zero is crossed (kRisingAndFalling).
   */
  DirectionType get_event_function_direction() const = 0;

  /**
   *  Returns whether this witness function returns a continuous output or
   *  a discontinuous output. A greater collection of methods for determining
   *  zero crossings is applicable for witness functions with continuous outputs.
   */
  virtual WitnessFunctionType get_event_function_output_type() const = 0;

  /**
   * Evaluates the witness function under the given context.
   * @param context the context at which to evaluate the witness function
   * @returns a scalar that indicates the status of the witness function.
   */
  // TODO(edrumwri): Ping edrumwri if it is desired to return a vector
  virtual T EvalWitnessFunction(const Context<T>& context) = 0;

  /**
   * Returns `true` if the derivative for the witness function with respect to
   * time is available and rapidly computable (returns `false` otherwise).
   * @sa EvalWitnessDeriv()
   */
  virtual bool is_deriv_available() const = 0;

  /**
   * Evaluates the derivative for the witness function, at the given context,
   * with respect to time. This function only need be implemented if
   * is_deriv_available() returns `true`. This default implementation returns
   * NaN.
   * @param context the context at which to evaluate the witness function
   *    time derivative.
   * @returns a scalar corresponding to the time derivative of the event
   *    function at the given context
   * @sa is_deriv_available()
   */
  virtual T EvalWitnessDeriv(const Context<T>& context) { return nan(); }

  /**
   * Determines whether this witness function should be treated as active for
   * the given context.
   * (Example of how this could be used for broad phase collision detection)
   * The default implementation returns `true`: the witness function will always
   * be treated as active.
   */
  virtual bool CheckWitnessFunctionActive(const Context<T>& context) {
    return true;
  }

 private:

  const System<T>& system_;
};
} // namespace systems
} // namespace drake