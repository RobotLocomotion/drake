#pragma once

namespace drake {
namespace systems {

/**
 * Abstract class for representing event functions used to compute solutions
 * to piecewise DAEs.
 *
 * A simulator could conceivably integrate over a large interval of time in
 * a single leap. Event functions can be used to cause the simulator to
 * identify, and even pause at, designated times or states. For example,
 * an event function can be used to determine when a flight simulator should
 * swap terrain maps as an aircraft crosses between the U.S. and Canada.
 *
 * Alternatively, a common use of event functions is to determine when
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
 * ground will be subject to contact forces up to the point that a wind
 * lifts the wing into the air. In the "resting on ground" case, `g(x) = 0`
 * could represent the acceleration normal to the ground. An event function
 * can help the DAE solver determine the time at which `g(x) = 0` should no
 * longer be active and the wing can undergo ballistic movement.
 *
 * A classical example of a DAE is the bouncing ball system, which uses
 * one set of equations when the ball is in a ballistic phase (g() is empty)
 * and another set when the ball is in sustained contact with the "ground".
 *
 * a_N = 0, LCP for determination of a_N, f_N
 *
 * Event functions in this case could consist of:<pre>
 * \phi(x) >= 0
 * </pre>
 *
 * An initial value problem for this Index 3 DAE could be solved- for the case
 * of the ball at, e.g., `x = 10, v = 0`- by (1) integrating the ballistic ODE
 * until x = 0, v < 0; (2) updating the state to x = 0, v >= 0; (3) [for v > 0]
 * repeating from Step (1); (4) [for v = 0] Integrating the sustained contact
 * ODE until desired time is reached.
 *
 * For (4), DAE constraint is: d^2phi/dt^2(x, v, a) = 0
 * (we assume from this point no forces are going to lift the ball back up).
 *
 * phi(x) = 0, dphi/dt(x, v) >= 0
 * phi(x) = 0, d^2phi/dt^2(x, v, a) >= 0
 *
 * Have to "reset" the state every time that the ball impacts the ground;
 * constraint is taken into account when ball undegoes sustained contact with
 * the ground.
 *
 * Examples of event functions for piecewise DAEs:
 *
 * g(x) >= 0 ==> g(x) = 0
 *
 * A System can define multiple guard functions through multiple classes.
 * Alternatively, 
 */
template <class T>
class EventFunction {
 public:
  /// Constructs this event function with a reference to the specified system.
  EventFunction(const System<T>& system) : system_(system) {}

  /// Two possible types of event function output types.
  enum EventFunctionType { kContinuous, kDiscontinuous };

  /// Directions of interest for zero crossings.
  enum DirectionType {
    /**
     * Zero crossings of interest are for both increasing (rising) and
     * decreasing (falling) event functions.
     */
    kRisingAndFalling,

    /**
     * Zero crossings of interest are for increasing (rising) event functions
     * only.
     */
    kRising,

    /**
     * Zero crossings of interest are for decreasing (falling) event functions
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
   * Gets the direction of interest for this event function. The direction
   * of interest can indicate that the event should trigger only on positive to
   * negative transitions (kFalling), only on negative to positive transitions
   * (kRising), or any time zero is crossed (kRisingAndFalling).
   */
  DirectionType get_event_function_direction() const = 0;

  /**
   *  Returns whether this event function returns a continuous output or
   *  a discontinuous output. A greater collection of methods for determining
   *  zero crossings is applicable for event functions with continuous outputs.
   */
  virtual EventFunctionType get_event_function_output_type() const = 0;

  /**
   * Evaluates the event function under the given context.
   * @param context the context at which to evaluate the event function
   * @returns a scalar that indicates the status of the witness function.
   */
  // TODO(edrumwri): Ping edrumwri if it is desired to return a vector
  virtual T EvalEventFunction(const Context<T>& context) = 0;

  /**
   * Returns `true` if the derivative for the event function with respect to
   * time is available and rapidly computable (returns `false` otherwise).
   * @sa EvalEventDeriv()
   */
  virtual bool is_deriv_available() const = 0;

  /**
   * Evaluates the derivative for the event function, at the given context,
   * with respect to time. This function only need be implemented if
   * is_deriv_available() returns `true`. This default implementation returns
   * NaN.
   * @param context the context at which to evaluate the event function
   *    time derivative.
   * @returns a scalar corresponding to the time derivative of the event
   *    function at the given context
   * @sa is_deriv_available()
   */
  virtual T EvalEventDeriv(const Context<T>& context) { return nan(); }

  /**
   * Determines whether this event function should be treated as active for
   * the given context.
   * (Example of how this could be used for broad phase collision detection)
   * The default implementation returns `true`: the event function will always
   * be treated as active.
   */
  virtual bool CheckEventFunctionActive(const Context<T>& context) {
    return true;
  }

 private:

  const System<T>& system_;
};
} // namespace systems
} // namespace drake