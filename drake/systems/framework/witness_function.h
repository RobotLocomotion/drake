#pragma once

#include <limits>

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
 * could represent the constraint that the acceleration normal to the ground
 * should be zero. A witness function could help the DAE solver determine the
 * time at which `g(x) = 0` should no longer apply, which in this example
 * would allow the wing to rise from the ground and undergo ballistic movement.
 *
 * A classical example of a DAE that can make use of a witness function is
 * the bouncing ball system, which switches between two sets of continuous
 * equations of motion: one when the ball is in ballistic flight and one when
 * the ball is in sustained contact with the ground. Witness functions help
 * a DAE solver determine when to switch between these sets of equations.
 *
 * If we modify this example slightly by allowing a strong wind to push the
 * ball upward intermittently, a witness function could be used to determine
 * when contact forces should not be applied (i.e., when the force between the
 * ball and the ground, applied normal to the ground, transitions from a
 * compressive force to a tensile force).
 */
template <class T>
class WitnessFunction {
 public:
  virtual ~WitnessFunction() {}
  WitnessFunction(const WitnessFunction&) = delete;
  WitnessFunction& operator=(const WitnessFunction&) = delete;

  /**
   * Constructs this witness function with a reference to the specified system.
   * Tolerances for zero are set exactly to zero; i.e., the witness function
   * will only be treated as zero when it evaluates exactly to zero.
   */
  WitnessFunction() {
  }

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
   * Calculates a conservative estimate of the next zero crossing time from the
   * given context. The default implementation returns -1.0 (no estimate). If
   * a positive estimate is provided, the estimate of the zero crossing time
   * (`te`) is to be treated as conservative: the zero crossing cannot occur
   * before future time `te` has arrived.
   * @returns an estimate of the zero crossing time; negative zero crossing
   *          times indicate that no estimate is available.
   */
  virtual double CalcConservativeNextZeroTime(const Context<T>& t) {
    return -1.0; }

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
   *  zero crossings is applicable for witness functions with continuous
   *  outputs.
   */
  virtual WitnessFunctionType get_event_function_output_type() const = 0;

  /**
   * Evaluates the witness function under the given context.
   * @param context the context at which to evaluate the witness function
   * @returns a scalar that indicates the status of the witness function.
   */
  // TODO(edrumwri): Ping edrumwri if it is desired to return a vector:
  // A system will generally utilize a set of witness functions. A zero crossing
  // will occur when _any_ of those witness functions cross zero. It may be
  // the case that multiple witness functions use the results from a single
  // computation. In that case, returning a vector from this function would
  // allow performing the computation just once.
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
  virtual T EvalWitnessDeriv(const Context<T>& context) { return
        std::numeric_limits<typename Eigen::NumTraits<T>::Real>::quiet_NaN(); }

  /**
   * Determines whether this witness function should be treated as active for
   * the given context over the given interval. The DAE solver will be able
   * to exploit when a witness function is inactive over an interval of time.
   * The default implementation returns `true`: the witness function will always
   * be treated as active.
   * @param context the context given the state and time of the system at the
   *          beginning of the time interval.
   * @param dt the length of the interval.
   * @returns `true` if the witness function is active over the interval
   *           `[context.get_time(), context.get_time()+dt]` and `false`
   *           otherwise (i.e., the witness function does not cross zero in that
   *           time interval).
   */
  virtual bool CheckWitnessFunctionActive(const Context<T>& context) {
    return true;
  }

  /**
   * @name Tolerance settings for zero crossings.
   * @{
   * This group of functions handles tolerance settings for the witness function
   * zero crossings. _Some_ non-zero tolerance is desired, as otherwise locating
   * the zero crossing for the witness function will likely prove to be
   * computationally infeasible: it is possible that the witness function
   * computation never yields exactly zero for any floating point argument.
   * [Press 1994] recommends a tolerance value of ε(|x₀| + |x₁|)/2, where ε
   * is the machine precision and x₀ < 0 < x₁ are input values (times in our
   * application) that "bracket" the zero crossing when the witness function is
   * active.
   * @TODO(edrumwri) Better clarify that this has to do with integration step
   *   size.
   *
   * Although we generally expect the user to use symmetric tolerances for
   * zero, we expect asymmetric tolerance settings to lead to computational
   * efficiency or preferred behavior in isolated cases. For a witness function
   * checking whether two rigid bodies will contact (using signed distance
   * between the bodies), an asymmetric zero tolerance [0, tol] can ensure that
   * bodies are not interpenetrating at the computed zero crossing. If we
   * assume that a root (i.e., zero crossing) finding approach spends roughly
   * half of its time on either side of zero, an asymmetric tolerance should
   * be able to increase computational efficiency in certain cases. In the
   * context of the signed distance between rigid body (the example above),
   * it is far faster to compute the Euclidean distance for disjoint
   * configurations than for intersecting ones, at least when bodies' geometries
   * are polyhedral and convex. A "left" tolerance of 0.0 and a "right"
   * positive tolerance should speed computation.
   */

  /**
   * Sets a "symmetric" tolerance for zero for the witness function. The witness
   * function `w()` will be considered to evaluate to zero when:
   * `-tol <= w() <= tol`. This function is equivalent to:
   * <pre>
   *   set_right_tolerance(tol);
   *   set_left_tolerance(-tol);
   * </pre>
   * @throws std::logic_error if @p tol is negative.
   */
  void set_tolerance(double tol) {
    if (tol < 0.)
      throw std::logic_error("Negative tolerance specified.");
    tolerance_right_ = tol;
    tolerance_left_ = -tol;
  }

  /**
   * Sets a tolerance for zero from the right of zero. The witness
   * function `w()` will be considered to evaluate to zero when:
   * `get_left_tolerance() <= w() <= tol`.
   * @throws std::logic_error if @p tol is negative.
   */
  void set_right_tolerance(double tol) {
    if (tol < 0.)
      throw std::logic_error("Negative tolerance specified.");
    tolerance_right_ = tol;
  }

  /**
 * Sets a tolerance for zero from the left of zero. The witness
 * function `w()` will be considered to evaluate to zero when:
 * tol <= w() <= `get_right_tolerance()`.
 * @throws std::logic_error if @p tol is positive.
 */
  void set_left_tolerance(double tol) {
    if (tol > 0.)
      throw std::logic_error("Positive tolerance specified.");
    tolerance_left_ = tol;
  }

  /**
   * @}
   */

 private:
  double tolerance_left_{0.0};   // Tolerance for zero from right side of zero.
  double tolerance_right_{0.0};  // Tolerance for zero from left side of zero.
  const System<T>& system_;
};
}  // namespace systems
}  // namespace drake
