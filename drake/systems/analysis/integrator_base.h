#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/*
TODO(edrumwri): comments below in preparation of upcoming functionality
    Note: we ensure algorithmically that no report time, scheduled time, or
final time t can occur *within* an event window, that is, we will never have
    t_low < t < t_high for any interesting t. Further, t_report, t_scheduled and
    t_final can coincide with t_high but only t_report can be at t_low. The
    interior of t_low:t_high is a "no man's land" where we don't understand the
solution, so must be avoided.
*/

/**
 * An abstract class for an integrator for ODEs and DAEs as represented by a
 * Drake System. Integrators solve initial value problems of the form:<pre>
 * ẋ(t) = f(t, x(t)) with f : ℝ × ℝⁿ → ℝⁿ
 * </pre>
 * (i.e., `f()` is an ordinary differential equation) given initial conditions
 * (t₀, x₀). Thus, integrators advance the continuous state of a dynamical
 * system forward in time.
 *
 * Apart from solving initial value problems, for which the integrator is a
 * key component of a simulator, integrators can also be used to solve
 * boundary value problems (via numerical methods like the Multiple Shooting
 * Method) and trajectory optimization problems (via numerical methods like
 * direct transcription). This class and its derivatives were developed
 * primarily toward the former application (through IntegrateAtMost() and
 * the Simulator class). However, the IntegratorBase architecture was developed
 * to support these ancillary applications as well using the
 * IntegrateWithMultipleSteps() and IntegrateWithSingleFixedStep() methods;
 * the latter permits the caller to advance time using fixed steps in
 * applications where variable stepping would be deleterious (e.g., direct
 * transcription).
 *
 * A natural question for a user to ask of an integrator is: Which scheme
 * (method) should be applied to a particular problem? The answer is whichever
 * one most quickly computes the solution to the desired accuracy! Selecting
 * an integration scheme for a particular problem is presently an artform. As
 * examples of some selection criteria: multistep methods generally work poorly
 * when events (that require state reinitializations) are common, symplectic
 * methods generally work well at maintaining stability for large integration
 * steps, and stiff integrators are often best for computationally stiff
 * systems. If ignorant as to the characteristics of a particular problem, it
 * is often best to start with an explicit, Runge-Kutta type method. Statistics
 * collected by the integrator can help diagnose performance issues and possibly
 * point to the use of a different integration scheme.
 *
 * Some systems are known to exhibit "computational stiffness", by which it is
 * meant that (excessively) small integration steps are necessary for purposes
 * of stability: in other words, steps must be taken smaller than that
 * required to achieve a desired accuracy *over a particular interval*.
 * Thus, the nature of computationally stiff problems is that the solution to
 * the ODE is *smooth* in the interval of stiffness (in contrast, some problems
 * possess such high frequency dynamics that very small steps are simply
 * necessary to capture the solution accurately). Implicit integrators are the
 * go-to approach for solving computationally stiff problems, but careful
 * consideration is warranted. Implicit integrators typically require much more
 * computation than non-implicit (explicit) integrators, stiffness might be an
 * issue on only a very small time interval, and some problems might be only
 * "moderately stiff". Put another way, applying an implicit integrator to a
 * potentially stiff problem might not yield faster computation. The first
 * chapter of [Hairer, 1996] illustrates the issues broached in this paragraph
 * using various examples.
 *
 * Established methods for integrating ordinary differential equations
 * invariably make provisions for estimating the "local error" (i.e., the
 * error over a small time interval) of a solution. Although the relationship
 * between local error and global error (i.e., the accumulated error over
 * multiple time steps) can be tenuous, such error estimates can allow
 * integrators to work adaptively, subdividing time intervals as necessary
 * (if, e.g., the system is particularly dynamic or stationary in an interval).
 * Even for applications that do not recommend such adaptive integration- like
 * direct transcription methods for trajectory optimization- error estimation
 * allows the user to assess the accuracy of the solution.
 *
 * IntegratorBase provides numerous settings and flags that can leverage
 * problem-specific information to speed integration and/or improve integration
 * accuracy. As an example, set_maximum_step_size() allows the user to prevent
 * overly large integration steps (that integration error control alone might
 * be insufficient to detect). As noted previously, IntegratorBase also collects
 * a plethora of statistics that can be used to diagnose poor integration
 * performance. For example, a large number of shrinkages due to error control
 * could indicate that a system is computationally stiff.
 *
 *  - [Hairer, 1996]   E. Hairer and G. Wanner. Solving Ordinary Differential
 *                     Equations II (Stiff and Differential-Algebraic Problems).
 *                     Springer, 1996.

 * @tparam T The vector element type, which must be a valid Eigen scalar.
 */
template <class T>
class IntegratorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IntegratorBase)

  /**
   * Status returned by StepOnceAtMost().
   * When a step is successful, it will return an indication of what caused it
   * to stop where it did. When unsuccessful it will throw an exception so you
   * won't see any return value. When return of control is due ONLY to reaching
   * a publish time, (status is kReachedPublishTime) the context may return an
   * interpolated value at an earlier time.
   *
   * Note: the simulation step must always end at an update time but can end
   * after a publish time.
   */
  // TODO(edrumwri): incorporate kReachedZeroCrossing into the simulator.
  enum StepResult {
    /**
     * Indicates a publish time has been reached but not an update time.
     */
    kReachedPublishTime = 1,
    /**
     * Localized an event; this is the *before* state (interpolated).
     */
    kReachedZeroCrossing = 2,
    /**
     * Indicates that integration terminated at an update time.
     */
    kReachedUpdateTime = 3,
    /**
     * User requested control whenever an internal step is successful.
     */
    kTimeHasAdvanced = 4,
    /**
     * Reached the desired integration time without reaching an update time.
     */
    kReachedBoundaryTime = 5,
    /** Took maximum number of steps without finishing integrating over the
     * interval. */
    kReachedStepLimit = 6,
  };

  /**
   * Maintains references to the system being integrated and the context used
   * to specify the initial conditions for that system (if any).
   * @param system A reference to the system to be integrated; the integrator
   *               will maintain a reference to the system in perpetuity, so
   *               the integrator must not outlive the system.
   * @param context A pointer to a writeable context (nullptr is ok, but a
   *                non-null pointer must be set before Initialize() is
   *                called). The integrator will advance the system state using
   *                the pointer to this context. The pointer to the context will
   *                be maintained internally. The integrator must not outlive
   *                the context.
   */
  explicit IntegratorBase(const System<T>& system,
                          Context<T>* context = nullptr)
      : system_(system), context_(context) {
    initialization_done_ = false;
  }

  /** Destructor. */
  virtual ~IntegratorBase() = default;

  /**
   * Indicates whether an integrator supports error estimation.
   * Without error estimation, target accuracy will be unused.
   */
  virtual bool supports_error_estimation() const = 0;

  /**
   * Sets an integrator with error control to fixed step mode. If the integrator
   * runs in fixed step mode, it will always take the maximum step size
   * directed (which may be that determined by get_maximum_step_size(), or may
   * be smaller, as directed by, e.g., Simulator for event handling purposes).
   * @throws std::logic_error if integrator does not support error
   *         estimation and @p flag is set to `false`.
   */
  void set_fixed_step_mode(bool flag) {
    if (!flag && !supports_error_estimation())
      throw std::logic_error("Integrator does not support accuracy estimation");
    fixed_step_mode_ = flag;
  }

  /**
   * Gets whether an integrator is running in fixed step mode. If the integrator
   * does not support error estimation, this function will always return `true`.
   * If the integrator runs in fixed step mode, it will always take the maximum
   * step size directed (which may be that determined by get_maximum_step_size()
   * or may be smaller, as directed by, e.g., Simulator for event handling
   * purposes).
   * @sa set_fixed_step_mode()
   */
  bool get_fixed_step_mode() const {
    return (!supports_error_estimation() || fixed_step_mode_);
  }

  /** Request that the integrator attempt to achieve a particular accuracy for
   * the continuous portions of the simulation. Otherwise a default accuracy is
   * chosen for you. This may be ignored for fixed-step integration since
   * accuracy control requires variable step sizes. You should call
   * supports_error_estimation() to ensure that the
   * integrator supports this capability before calling this function; if
   * the integrator does not support it, this method will throw an exception.
   *
   * Integrators vary in the range of accuracy (loosest to tightest) that they
   * can support. If you request accuracy outside the supported range for the
   * chosen integrator it will be quietly adjusted to be in range. You can find
   * out the accuracy setting actually being used using `get_accuracy_in_use()`.
   *
   * The precise meaning of *accuracy* is a complicated discussion, but
   * translates roughly to the number of significant digits you want in the
   * results. By convention it is supplied as `10^-digits`, meaning that an
   * accuracy of 1e-3 provides about three significant digits. For more
   * information, see [Sherman 2011].
   * - M. Sherman, et al. Procedia IUTAM 2:241-261 (2011), Section 3.3.
   *   http://dx.doi.org/10.1016/j.piutam.2011.04.023
   * @throws std::logic_error if integrator does not support error
   *         estimation.
   */
  // TODO(edrumwri): complain if integrator with error estimation wants to drop
  //                 below the minimum step size
  void set_target_accuracy(double accuracy) {
    if (!supports_error_estimation())
      throw std::logic_error(
          "Integrator does not support accuracy estimation "
          "and user has requested error control");
    target_accuracy_ = accuracy;
    accuracy_in_use_ = accuracy;
  }

  /**
   *   Gets the target accuracy.
   *   @sa get_accuracy_in_use()
   */
  double get_target_accuracy() const { return target_accuracy_; }

  /**
   * Gets the accuracy in use by the integrator. This number may differ from
   * the target accuracy if, for example, the user has requested an accuracy
   * not attainable or not recommended for the particular integrator.
   */
  double get_accuracy_in_use() const { return accuracy_in_use_; }

  /**
   * Sets the maximum step size that may be taken by this integrator. The
   * integrator may stretch the maximum step size by as much as 1% to reach a
   * discrete event. For fixed step integrators, all steps will be taken at the
   * maximum step size *unless* an event would be missed.
   */
  // TODO(edrumwri): Update this comment when stretch size is configurable.
  void set_maximum_step_size(const T& max_step_size) {
    DRAKE_ASSERT(max_step_size >= 0.0);
    max_step_size_ = max_step_size;
  }

  /**
   * Gets the maximum step size that may be taken by this integrator. This is
   * a soft maximum: the integrator may stretch it by as much as 1% to hit a
   * discrete event.
   * @sa set_minimum_step_size()
   */
  // TODO(edrumwri): Update this comment when stretch size is configurable.
  const T& get_maximum_step_size() const { return max_step_size_; }

  /**
   * @anchor Minstep
   * @name Methods for minimum integration step size selection and behavior
   *
   * Variable step integrators reduce their step sizes as needed to achieve
   * requirements such as specified accuracy or step convergence. However, it is
   * not possible to take an arbitrarily small step. Normally integrators choose
   * an appropriate minimum step and throw an exception if the requirements
   * can't be achieved without going below that. Methods in this section allow
   * you to influence two aspects of this procedure:
   * - you can increase the minimum step size, and
   * - you can control whether an exception is thrown if a smaller step would
   *   have been needed to achieve the aforementioned integrator requirements.
   *
   * By default, integrators allow a very small minimum step which can
   * result in long run times. Setting a larger minimum can be helpful as a
   * diagnostic to figure out what aspect of your simulation is requiring small
   * steps. You can set the minimum to what should be a "reasonable" minimum
   * based on what you know about the physical system. You will then get an
   * std::runtime_error exception thrown at any point in time where your model 
   * behaves unexpectedly (due to, e.g., a discontinuity in the derivative
   * evaluation function).
   *
   * If you disable the exception (via
   * `set_throw_on_minimum_step_size_violation(false)`), the integrator will
   * simply proceed with a step of the minimum size: accuracy is guaranteed
   * only when the minimum step size is not violated. Beware that there can be 
   * no guarantee about the magnitude of any errors introduced by violating the
   * accuracy "requirements" in this manner, so disabling the exception should
   * be done warily.
   *
   * #### Details
   * Because time is maintained to finite precision, there is an absolute
   * minimum step size `h_floor` required to avoid roundoff error. The
   * integrator will never take a step smaller than `h_floor`. We calculate
   * `h_floor=max(ε,ε⋅t)`, where t is the current time and ε is a small multiple
   * of machine precision, typically a number like 1e-14. Note that `h_floor`
   * necessarily grows with time; if that is a concern you should limit how
   * long your simulations are allowed to run without resetting time.
   *
   * You may request a larger minimum step size `h_min`. Then at every time t,
   * the integrator determines a "working" minimum `h_work=max(h_min,h_floor)`.
   * If the step size selection algorithm determines that a step smaller than
   * `h_work` is needed to meet accuracy or other needs, then a 
   * std::runtime_error exception will be thrown and the simulation halted. On 
   * the other hand, if you have suppressed the exception (again, via
   * `set_throw_on_minimum_step_size_violation(false)`), the integration
   * will continue, taking a step of size `h_work`.
   *
   * Under some circumstances the integrator may legitimately take a step of
   * size `h` smaller than your specified `h_min`, although never smaller than
   * `h_floor`. For example, occasionally the integrator may reach an event or
   * time limit that occurs a very short time after the end of a previous step,
   * necessitating that a tiny "sliver" of a step be taken to complete the
   * interval. That does not indicate an error, and required accuracy and
   * convergence goals are achieved. Larger steps can resume immediately
   * afterwards. Another circumstance is when one of the integrator's stepping
   * methods is called directly requesting a very small step, for example
   * `IntegrateWithMultipleSteps(h)`. No exception will be thrown in either of
   * these cases.
   */

  //@{
  /**
   * Sets the requested minimum step size `h_min` that may be taken by this
   * integrator. No step smaller than this will be taken except under
   * circumstances as described @link Minstep above. @endlink This setting will
   * be ignored if it is smaller than the absolute minimum `h_floor` also
   * described above. Default value is zero.
   * @param min_step_size a non-negative value. Setting this value to zero
   *                      will cause the integrator to use a reasonable value
   *                      instead (see get_working_minimum_step_size()).
   * @sa get_requested_minimum_step_size()
   * @sa get_working_minimum_step_size()
   */
  void set_requested_minimum_step_size(const T& min_step_size) {
    DRAKE_ASSERT(min_step_size >= 0.0);
    req_min_step_size_ = min_step_size;
  }

  /**
   * Gets the requested minimum step size `h_min` for this integrator.
   * @sa set_requested_minimum_step_size()
   * @sa get_working_minimum_step_size(T)
   */
  const T& get_requested_minimum_step_size() const {
    return req_min_step_size_; }

  /**
   * Sets whether the integrator should throw a std::runtime_error exception
   * when the integrator's step size selection algorithm determines that it
   * must take a step smaller than the minimum step size (for, e.g., purposes
   * of error control). Default is `true`. If `false`, the integrator will
   * advance time and state using the minimum specified step size in such
   * situations. See @link Minstep this section @endlink for more detail.
   */
  void set_throw_on_minimum_step_size_violation(bool throws) {
    min_step_exceeded_throws_ = throws;
  }

  /**
   * Reports the current setting of the throw_on_minimum_step_size_violation
   * flag.
   * @sa set_throw_on_minimum_step_size_violation().
   */
  bool get_throw_on_minimum_step_size_violation() const {
    return min_step_exceeded_throws_;
  }

  /**
   * Gets the current value of the working minimum step size `h_work(t)` for
   * this integrator, which may vary with the current time t as stored in the
   * integrator's context.
   * See @link Minstep this section @endlink for more detail.
   */
  T get_working_minimum_step_size() const {
    using std::max;
    // Tolerance is just a number close to machine epsilon.
    const double tol = 1e-14;
    const T smart_minimum = max(tol, get_context().get_time()*tol);
    return max(smart_minimum, req_min_step_size_);
  }
  //@}

  /**
   * Resets the integrator to initial values, i.e., default construction
   * values.
   */
  void Reset() {
    // Kill the error estimate and weighting matrices.
    err_est_.reset();
    qbar_weight_.setZero(0);
    z_weight_.setZero(0);
    pinvN_dq_change_.reset();
    unweighted_substate_change_.setZero(0);
    weighted_q_change_.reset();

    // Integrator no longer operates in fixed step mode.
    fixed_step_mode_ = false;

    // Statistics no longer valid.
    ResetStatistics();

    // Wipe out settings.
    req_min_step_size_ = 0;
    max_step_size_ = nan();
    accuracy_in_use_ = nan();

    // Indicate values used for error controlled integration no longer valid.
    prev_step_size_ = nan();
    ideal_next_step_size_ = nan();

    // Call the derived integrator reset routine.
    DoReset();

    // Indicate that initialization is necessary.
    initialization_done_ = false;
  }

  /**
   * An integrator must be initialized before being used. The pointer to the
   * context must be set before Initialize() is called (or an std::logic_error
   * will be thrown). If Initialize() is not called, an exception will be
   * thrown when attempting to call StepOnceAtMost(). To reinitialize the
   * integrator, Reset() should be called followed by Initialize().
   * @throws std::logic_error If the context has not been set or a user-set
   *         parameter has been set illogically (i.e., one of the
   *         weighting matrix coefficients is set to a negative value- this
   *         check is only performed for integrators that support error
   *         estimation; the maximum step size is smaller than the minimum
   *         step size; the requested initial step size is outside of the
   *         interval [minimum step size, maximum step size]).
   * @sa Reset()
   */
  void Initialize() {
    if (!context_) throw std::logic_error("Context has not been set.");

    // Verify that user settings are reasonable.
    if (max_step_size_ < req_min_step_size_) {
      throw std::logic_error("Integrator maximum step size is less than the "
                             "minimum step size");
    }
    if (req_initial_step_size_ > max_step_size_) {
      throw std::logic_error("Requested integrator initial step size is larger "
                             "than the maximum step size.");
    }
    if (req_initial_step_size_ < req_min_step_size_) {
      throw std::logic_error("Requested integrator initial step size is smaller"
                             " than the minimum step size.");
    }

    // TODO(edrumwri): Compute qbar_weight_, z_weight_ automatically.
    // Set error weighting vectors if not already done.
    if (supports_error_estimation()) {
      // Allocate space for the error estimate.
      err_est_ = system_.AllocateTimeDerivatives();

      const auto& xc = context_->get_state().get_continuous_state();
      const int gv_size = xc->get_generalized_velocity().size();
      const int misc_size = xc->get_misc_continuous_state().size();
      if (qbar_weight_.size() != gv_size) qbar_weight_.setOnes(gv_size);
      if (z_weight_.size() != misc_size) z_weight_.setOnes(misc_size);

      // Verify that minimum values of the weighting matrices are non-negative.
      if ((qbar_weight_.size() && qbar_weight_.minCoeff() < 0) ||
          (z_weight_.size() && z_weight_.minCoeff() < 0))
        throw std::logic_error("Scaling coefficient is less than zero.");
    }

    // Statistics no longer valid.
    ResetStatistics();

    // Call the derived integrator initialization routine (if any)
    DoInitialize();

    initialization_done_ = true;
  }

  /**
   * Request that the first attempted integration step have a particular size.
   * If no request is made, the integrator will estimate a suitable size
   * for the initial step attempt. *If the integrator does not support error
   * control*, this method will throw a std::logic_error (call
   * supports_error_estimation() to verify before calling this method). For
   * variable-step integration, the initial target will be treated as a maximum
   * step size subject to accuracy requirements and event occurrences. You can
   * find out what size *actually* worked with
   * `get_actual_initial_step_size_taken()`.
   * @throws std::logic_error If the integrator does not support error
   *     estimation.
   */
  void request_initial_step_size_target(const T& step_size) {
    using std::isnan;
    if (!supports_error_estimation())
      throw std::logic_error(
          "Integrator does not support error estimation and "
          "user has initial step size target");
    req_initial_step_size_ = step_size;
  }

  /**
   * Gets the target size of the first integration step. You can find out what
   * step size was *actually* used for the first integration step with
   * `get_actual_initial_step_size_taken()`.
   * @see request_initial_step_size_target()
   */
  const T& get_initial_step_size_target() const {
    return req_initial_step_size_;
  }

  /**
   * Integrates the system forward in time by a single step with step size
   * subject to integration error tolerances (assuming that the integrator
   * supports error estimation). The integrator must already have
   * been initialized or an exception will be thrown. The context will be
   * integrated forward by an amount that will never exceed the minimum of
   * `publish_dt`, `update_dt`, and `1.01 * get_maximum_step_size()`.
   *
   * @param publish_dt The step size, >= 0.0 (exception will be thrown
   *        if this is not the case) at which the next publish will occur.
   * @param update_dt The step size, > 0.0 (exception will be thrown
   *        if this is not the case) at which the next update will occur.
   * @param boundary_dt The step size, >= 0.0 (exception will be thrown
   *        if this is not the case) marking the end of the user-designated
   *        simulated interval.
   * @throws std::logic_error If the integrator has not been initialized or one
   *                          of publish_dt, update_dt, or boundary_dt is
   *                          negative.
   * @return The reason for the integration step ending.
   * @warning Users should generally not call this function directly; within
   *          simulation circumstances, users will typically call
   *          `Simulator::StepTo()`. In other circumstances, users will
   *          typically call `IntegratorBase::IntegrateWithMultipleSteps()`.
   *
   * This method at a glance:
   * - For integrating ODEs/DAEs via Simulator
   * - Supports fixed step and variable step integration schemes
   * - Takes only a single step forward.
   */
  // TODO(edrumwri): Make the stretch size configurable.
  StepResult IntegrateAtMost(const T& publish_dt, const T& update_dt,
                             const T& boundary_dt);

  /// Gets the stretch factor (> 1), which is multiplied by the maximum
  /// (typically user-designated) integration step size to obtain the amount
  /// that the integrator is able to stretch the maximum time step toward
  /// hitting an upcoming publish or update event in IntegrateAtMost().
  /// @sa IntegrateAtMost()
  double get_stretch_factor() const { return 1.01; }

  /// Stepping function for integrators operating outside of Simulator that
  /// advances the continuous state exactly by @p dt. This method is designed
  /// for integrator users that do not wish to consider publishing or
  /// discontinuous, mid-interval updates. This method will step the integrator
  /// multiple times, as necessary, to attain requested error tolerances and
  /// to ensure the integrator converges.
  /// @warning Users should simulate systems using `Simulator::StepTo()` in
  ///          place of this function (which was created for off-simulation
  ///          purposes), generally.
  /// @param dt The non-negative integration step to take.
  /// @throws std::logic_error If the integrator has not been initialized or
  ///                          dt is negative.
  /// @sa IntegrateAtMost(), which is designed to be operated by Simulator and
  ///     accounts for publishing and state reinitialization.
  /// @sa IntegrateWithSingleStep(), which is also designed to be operated
  ///     *outside of* Simulator, but throws an exception if the integrator
  ///     cannot advance time by @p dt in a single step.
  ///
  /// This method at a glance:
  /// - For integrating ODEs/DAEs not using Simulator
  /// - Supports fixed step and variable step integration schemes
  /// - Takes as many steps as necessary until time has advanced by @p dt
  void IntegrateWithMultipleSteps(const T& dt) {
    using std::max;
    using std::min;

    const Context<T>& context = get_context();
    const T inf = std::numeric_limits<double>::infinity();
    T t_remaining = dt;

    // Note: A concern below is that the while loop while run forever because
    // t_remaining could be small, but not quite zero, if dt is relatively
    // small compared to the context time. In such a case, t_final will be
    // equal to context.get_time() in the expression immediately below,
    // context.get_time() will not change during the call to IntegrateAtMost(),
    // and t_remaining will be equal to zero (meaning that the loop will
    // indeed terminate, as desired).
    const T t_final = context.get_time() + t_remaining;
    do {
      IntegrateAtMost(inf, inf, min(t_remaining, get_maximum_step_size()));
      t_remaining = t_final - context.get_time();
    } while (t_remaining > 0);
  }

  /// Stepping function for integrators operating outside of Simulator that
  /// advances the continuous state exactly by @p dt *and using a single fixed
  /// step*. This method is designed for integrator users that do not wish to
  /// consider publishing or discontinuous, mid-interval updates. One such
  /// example application is that of direct transcription for trajectory
  /// optimization, for which the integration process should be _consistent_: it
  /// should execute the same sequence of arithmetic operations for all values
  /// of the nonlinear programming variables. In keeping with the naming
  /// semantics of this function, error controlled integration is not supported
  /// (though error estimates will be computed for integrators that support that
  /// feature), which is a minimal requirement for "consistency".
  /// @warning Users should simulate systems using `Simulator::StepTo()` in
  ///          place of this function (which was created for off-simulation
  ///          purposes), generally.
  /// @param dt The non-negative integration step to take.
  /// @throws std::logic_error If the integrator has not been initialized or
  ///                          dt is negative **or** if the integrator
  ///                          is not operating in fixed step mode.
  /// @throws std::runtime_error If the integrator was unable to take a step
  ///         of the requested size.
  /// @sa IntegrateAtMost(), which is designed to be operated by Simulator and
  ///     accounts for publishing and state reinitialization.
  /// @sa IntegrateWithMultipleSteps(), which is also designed to be operated
  ///     *outside of* Simulator, but will take as many integration steps as
  ///     necessary until time has been stepped forward by @p dt.
  ///
  /// This method at a glance:
  /// - For integrating ODEs/DAEs not using Simulator
  /// - Fixed step integration (no step size reductions for error control or
  ///   integrator convergence)
  /// - Takes only a single step forward.
  void IntegrateWithSingleFixedStep(const T& dt) {
    if (dt < 0) {
      throw std::logic_error("IntegrateWithSingleFixedStep() called with a "
                             "negative step size.");
    }
    if (!this->get_fixed_step_mode())
      throw std::logic_error("IntegrateWithSingleFixedStep() requires fixed "
                             "stepping.");
    if (!Step(dt)) {
      throw std::runtime_error("Integrator was unable to take a single fixed "
                                   "step of the requested size.");
    }

    UpdateStepStatistics(dt);
  }

  /**
   * @name Integrator statistics methods.
   *
   * @{
   * These methods allow the caller to manipulate and query integrator
   * statistics. Generally speaking, the larger the integration step taken, the
   * faster a simulation will run. These methods allow querying (and resetting)
   * the integrator statistics as one means of determining how to make
   * a simulation run faster.
   */
  /**
   * Forget accumulated statistics. These are reset to the values they have
   * post construction or immediately after `Initialize()`.
   */
  void ResetStatistics() {
    actual_initial_step_size_taken_ = nan();
    smallest_adapted_step_size_taken_ = nan();
    largest_step_size_taken_ = nan();
    num_steps_taken_ = 0;
    num_ode_evals_ = 0;
    num_shrinkages_from_error_control_ = 0;
    num_shrinkages_from_substep_failures_ = 0;
    num_substep_failures_ = 0;
    DoResetStatistics();
  }

  /// Gets the number of failed sub-steps (implying one or more step size
  /// reductions was required to permit solving the necessary nonlinear system
  /// of equations).
  int64_t get_num_substep_failures() const {
    return num_substep_failures_;
  }

  /// Gets the number of step size shrinkages due to sub-step failures (e.g.,
  /// integrator convergence failures) since the last call to ResetStatistics()
  /// or Initialize().
  int64_t get_num_step_shrinkages_from_substep_failures() const {
    return num_shrinkages_from_substep_failures_;
  }

  /// Gets the number of step size shrinkages due to failure to meet targeted
  /// error tolerances, since the last call to ResetStatistics or Initialize().
  int64_t get_num_step_shrinkages_from_error_control() const {
    return num_shrinkages_from_error_control_;
  }

/**
 * Returns the number of ODE function evaluations (calls to
 * CalcTimeDerivatives()) since the last call to ResetStatistics() or
 * Initialize(). This count includes *all* such calls including (1)
 * those necessary to compute Jacobian matrices; (2) those used in rejected
 * integrated steps (for, e.g., purposes of error control); (3) those used
 * strictly for integrator error estimation; and (4) calls that exhibit little
 * cost (due to results being cached).
 */
  int64_t get_num_derivative_evaluations() const { return num_ode_evals_; }

  /**
   * The actual size of the successful first step.
   */
  const T& get_actual_initial_step_size_taken() const {
    return actual_initial_step_size_taken_;
  }

  /**
   * The size of the smallest step taken *as the result of a controlled
   * integration step adjustment* since the last Initialize() or
   * ResetStatistics() call. This value will be NaN for integrators without
   * error estimation.
   */
  const T& get_smallest_adapted_step_size_taken() const {
    return smallest_adapted_step_size_taken_;
  }

  /**
   * The size of the largest step taken since the last Initialize() or
   * ResetStatistics() call.
   */
  const T& get_largest_step_size_taken() const {
    return largest_step_size_taken_;
  }

  /**
   * The number of integration steps taken since the last Initialize()
   * or ResetStatistics() call.
   */
  int64_t get_num_steps_taken() const { return num_steps_taken_; }

  /**
   * @}
   */

  /**
   * Return the step size the integrator would like to take next, based
   * primarily on the integrator's accuracy prediction. This value will not
   * be computed for integrators that do not support error estimation and
   * NaN will be returned.
   */
  const T& get_ideal_next_step_size() const { return ideal_next_step_size_; }

  /**
   * Returns a const reference to the internally-maintained Context holding
   * the most recent state in the trajectory. This is suitable for publishing or
   * extracting information about this trajectory step.
   */
  const Context<T>& get_context() const { return *context_; }

  /**
   * Returns a mutable pointer to the internally-maintained Context holding
   * the most recent state in the trajectory.
   */
  Context<T>* get_mutable_context() { return context_; }

  /**
   * Replace the pointer to the internally-maintained Context with a different
   * one. This is useful for supplying a new set of initial conditions or
   * wiping out the current context (by passing in a null pointer). You
   * should invoke Initialize() after replacing the Context unless the
   * context is null.
   * @param context The pointer to the new context or nullptr to wipe out
   *                the current context without replacing it with another.
   */
  void reset_context(Context<T>* context) {
    context_ = context;
    initialization_done_ = false;
  }

  /**
   * Gets a constant reference to the system that is being integrated (and
   * was provided to the constructor of the integrator).
   */
  const System<T>& get_system() const { return system_; }

  /// Indicates whether the integrator has been initialized.
  bool is_initialized() const { return initialization_done_; }

  /**
   * Derived classes must override this function to return the order of
   * the integrator's error estimate. If the integrator does not provide an
   * error estimate, the derived class implementation should return 0.
   */
  virtual int get_error_estimate_order() const = 0;

  /**
   * Gets the size of the last (previous) integration step. If no integration
   * steps have been taken, value will be NaN.
   */
  const T& get_previous_integration_step_size() const {
    return prev_step_size_;
  }

  /**
   * Gets the error estimate (used only for integrators that support error
   * estimation). If the integrator does not support error estimation, nullptr
   * is returned.
   */
  const ContinuousState<T>* get_error_estimate() const {
    return err_est_.get();
  }

  /**
   * @name         Methods for weighting state variable errors
   * @{
   * This group of methods describes how errors for state variables with
   * heterogeneous units are weighted in the context of error-controlled
   * integration. This is an advanced topic and most users can simply specify
   * desired accuracy and accept the default state variable weights.
   *
   * A collection of state variables is generally defined in heterogenous units
   * (e.g. length, angles, velocities, energy). Some of the state
   * variables cannot even be expressed in meaningful units, like
   * quaternions. Certain integrators provide an estimate of the absolute error
   * made in each state variable during an integration step. These errors must
   * be properly weighted to obtain an "accuracy" _with respect to each
   * particular variable_. These per-variable accuracy determinations can be
   * compared against the user's requirements and used to select an appropriate
   * size for the next step [Sherman 2011]. The weights are
   * normally determined automatically using the system's characteristic
   * dimensions, so *most users can stop reading now!* Custom weighting is
   * primarily useful for performance improvement; an optimal weighting would
   * allow an error-controlled integrator to provide the desired level of
   * accuracy across all state variables without wasting computation
   * achieving superfluous accuracy for some of those variables.
   *
   * Users interested in more precise control over state variable weighting may
   * use the methods in this group to access and modify weighting factors for
   * individual state variables. Changes to these weights can only be made prior
   * to integrator initialization or as a result of an event being triggered
   * and then followed by re-initialization.
   *
   * <h4>Relative versus absolute accuracy</h4>
   *
   * %State variable integration error, as estimated by an integrator, is an
   * absolute quantity with the same
   * units as the variable. At each time step we therefore need to determine
   * an absolute error that would be deemed "good enough", i.e. satisfies
   * the user's accuracy requirement. If a variable is maintained to a
   * *relative* accuracy then that "good enough" value is defined to be the
   * required accuracy `a` (a fraction like 0.001) times the current value of
   * the variable, as long as that value
   * is far from zero. For variables maintained to an *absolute* accuracy, or
   * relative variables that are at or near zero (where relative accuracy would
   * be undefined or too strict, respectively), we need a different way to
   * determine the "good enough" absolute error. The methods in this section
   * control how that absolute error value is calculated.
   *
   * <h4>How to choose weights</h4>
   *
   * The weight `wᵢ` for a state variable `xᵢ` should be
   * chosen so that the product `wᵢ * dxᵢ` is unitless, and in particular is 1
   * when `dxᵢ` represents a "unit effect" of state variable `xᵢ`; that is, the
   * change in `xᵢ` that produces a unit change in some quantity of interest in
   * the system being simulated. Why unity (1)? Aside from normalizing the
   * values, unity "grounds" the weighted error to the user-specified accuracy.
   * A weighting can be applied individually to each state variable, but
   * typically it is done approximately by combining the known type of the
   * variable (e.g. length, angle) with a "characteristic scale" for that
   * quantity. For example, if a "characteristic length" for the system being
   * simulated is 0.1 meters, and `x₀` is a length variable measured in meters,
   * then `w₀` should be 10 so that `w₀*dx₀=1` when `dx₀=0.1`. For angles
   * representing pointing accuracy (say a camera direction) we typically assume
   * a "characteristic angle" is one radian (about 60 degrees), so if x₁ is a
   * pointing direction then w₁=1 is an appropriate weight. We can now scale an
   * error vector `e=[dx₀ dx₁]` to a unitless fractional error vector
   * `f=[w₀*dx₀ w₁*dx₁]`. Now to achieve a given accuracy `a`, say `a=.0001`,
   * we need only check that `|fᵢ|<=a` for each element `i` of `f`. Further,
   * this gives us a quantitative measure of "worst accuracy" that we can use
   * to increase or reduce size of the next attempted step, so that we will just
   * achieve the required accuracy but not much more. We'll be more precise
   * about this below.
   *
   * <h4>Some subtleties for second-order dynamic systems</h4>
   *
   * Systems governed by 2nd-order differential equations are typically split
   * into second order (configuration) variables q, and rate (velocity)
   * variables v, where the time derivatives qdot of q are linearly related to
   * v by the kinematic differential equation `qdot = dq/dt = N(q)*v`.
   * Velocity variables are
   * chosen to be physically significant, but configuration variables
   * may be chosen for convenience and do not necessarily have direct physical
   * interpretation. For examples, quaternions are chosen as a numerically
   * stable orientation representation. This is problematic for choosing weights
   * which must be done by physical reasoning
   * as sketched above. We resolve this by introducing
   * the notion of "quasi-coordinates" ꝗ (pronounced "qbar") which are defined
   * by the equation `ꝗdot = dꝗ/dt = v`. Other than time scaling,
   * quasi-coordinates have the same units as their corresponding velocity
   * variables. That is, for weighting we need to think
   * of the configuration coordinates in the same physical space as the velocity
   * variables; weight those by their physical significance; and then map back
   * to an instantaneous weighting
   * on the actual configuration variables q. This mapping is performed
   * automatically; you need only to be concerned about physical weightings.
   *
   * Note that generalized quasi-coordinates `ꝗ` can only be defined locally for
   * a particular configuration `q`. There is in general no meaningful set of
   * `n` generalized
   * coordinates which can be differentiated with respect to time to yield `v`.
   * For example, the Hairy Ball Theorem implies that it is not possible for
   * three orientation variables to represent all 3D rotations without
   * singularities, yet three velocity variables can represent angular velocity
   * in 3D without singularities.
   *
   * To summarize, separate weights can be provided for each of
   * - `n` generalized quasi-coordinates `ꝗ`  (configuration variables in the
   *   velocity variable space), and
   * - `nz` miscellaneous continuous state variables `z`.
   *
   * Weights on the generalized velocity variables `v (= dꝗ/dt)` are derived
   * directly from the weights on `ꝗ`, weighted by a characteristic time. Weights
   * on the actual `nq` generalized coordinates can
   * be calculated efficiently from weights on the quasi-coordinates (details
   * below).
   *
   * <h4>How the weights are used</h4>
   *
   * The errors in the `ꝗ` and `z` variables are weighted by the diagonal elements
   * of diagonal weighting matrices Wꝗ and Wz, respectively. (The block-diagonal
   * weighting matrix `Wq` on the original generalized coordinates `q` is
   * calculated from `N` and `Wꝗ`; see below.) In the absence of
   * other information, the default for all weighting values is one, so `Wꝗ` and
   * `Wz` are `n × n` and `nz × nz` identity matrices. The weighting matrix `Wv`
   * for the velocity variables is just `Wv = τ*Wꝗ` where `τ` is a
   * "characteristic time" for the system, that is, a quantity in time units
   * that represents a significant evolution of the trajectory. This serves to
   * control the accuracy with which velocity is determined relative to
   * configuration. Note that larger values of `τ` are more conservative since
   * they increase the velocity weights. Typically we use `τ=1.0` or `0.1`
   * seconds for human-scale mechanical systems.
   * <!-- TODO(sherm1): provide more guidance for velocity weighting. -->
   *
   * The weighting matrices `Wq`, `Wv`, and `Wz` are used to compute a weighted
   * infinity norm as follows. Although `Wv` and `Wz` are constant, the actual
   * weightings may be state dependent for relative-error calculations.
   * Define block diagonal error weighting matrix `E=diag(Eq,Ev,Ez)` as follows:
   * <pre>
   *   Eq = Wq
   *   Ev: Ev(i,i) = { min(Wv(i,i), 1/|vᵢ|)     if vᵢ is relative
   *                 { Wv(i,i)                  if vᵢ is absolute
   *   Ez: Ez(i,i) = { min(Wz(i,i), 1/|zᵢ|)     if zᵢ is relative
   *                 { Wz(i,i)                  if zᵢ is absolute
   * </pre>
   * (`Ev` and `Ez` are diagonal.) A `v` or `z` will be maintained to relative
   * accuracy unless (a) it is "close" to zero (less than 1), or (b) the
   * variable has been defined as requiring absolute accuracy. Position
   * variables `q` are always maintained to absolute accuracy (see
   * [Sherman 2011] for rationale).
   *
   * Now given an error estimate vector `e=[eq ev ez]`, the vector `f=E*e`
   * can be considered to provide a unitless fractional error for each of the
   * state variables. To achieve a given user-specified accuracy `a`, we require
   * that norm_inf(`f`) <= `a`. That is, no element of `f` can have absolute
   * value larger than `a`. We also use `f` to determine an ideal next step
   * size using an appropriate integrator-specific computation.
   *
   * <h4>Determining weights for q</h4>
   *
   * The kinematic differential equations `qdot=N(q)*v` employ an `nq × n`
   * matrix `N`. By construction, this relationship is invertible using `N`'s
   * left pseudo-inverse `N⁺` so that `v=N⁺ qdot` and `N⁺ N = I` (the identity
   * matrix); however, `N N⁺ != I`, as `N` has more rows than columns generally.
   * [Nikravesh 1988] shows how such a matrix `N` can be determined and provides
   * more information. Given this relationship between `N` and `N⁺`, we can
   * relate weighted errors in configuration coordinates `q` to weighted errors in
   * generalized quasi-coordinates `ꝗ`, as the following derivation shows: <pre>
   *            v = N⁺ qdot         Inverse kinematic differential equation
   *        dꝗ/dt = N⁺ dq/dt        Use synonyms for v and qdot
   *           dꝗ = N⁺ dq           Change time derivatives to differentials
   *        Wꝗ dꝗ = Wꝗ N⁺ dq        Pre-multiply both sides by Wꝗ
   *      N Wꝗ dꝗ = N Wꝗ N⁺ dq      Pre-multiply both sides by N
   *      N Wꝗ dꝗ = Wq dq           Define Wq := N Wꝗ N⁺
   *       N Wꝗ v = Wq qdot         Back to time derivatives.
   * </pre>
   * The last two equations show that `Wq` as defined above provides the
   * expected relationship between the weighted `ꝗ` or `v` variables in velocity
   * space and the weighted `q` or `qdot` (resp.) variables in configuration
   * space.
   *
   * Finally, note that a diagonal entry of one of the weighting matrices can
   * be set to zero to disable error estimation for that state variable
   * (i.e., auxiliary variable or configuration/velocity variable pair), but
   * that setting an entry to a negative value will cause an exception to be
   * thrown when the integrator is initialized.
   *
   * - [Nikravesh 1988] P. Nikravesh. Computer-Aided  Analysis of Mechanical
   *     Systems. Prentice Hall, 1988. Sec. 6.3.
   * - [Sherman 2011]   M. Sherman, et al. Procedia IUTAM 2:241-261 (2011),
   *     Section 3.3. http://dx.doi.org/10.1016/j.piutam.2011.04.023
   *
   *  @sa CalcStateChangeNorm()
   */
  /**
   * Gets the weighting vector (equivalent to a diagonal matrix) applied to
   * weighting both generalized coordinate and velocity state variable errors,
   * as described in the group documentation. Only used for integrators that
   * support error estimation.
   */
  const Eigen::VectorXd& get_generalized_state_weight_vector() const {
    return qbar_weight_;
  }

  /**
   * Gets a mutable weighting vector (equivalent to a diagonal matrix) applied
   * to weighting both generalized coordinate and velocity state variable
   * errors, as described in the group documentation. Only used for
   * integrators that support error estimation. Returns a VectorBlock
   * to make the values mutable without permitting changing the size of
   * the vector. Requires re-initializing the integrator after calling
   * this method; if Initialize() is not called afterward, an exception will be
   * thrown when attempting to call StepOnceAtMost(). If the caller sets
   * one of the entries to a negative value, an exception will be thrown
   * when the integrator is initialized.
   */
  Eigen::VectorBlock<Eigen::VectorXd>
  get_mutable_generalized_state_weight_vector() {
    initialization_done_ = false;
    return qbar_weight_.head(qbar_weight_.rows());
  }

  /**
   * Gets the weighting vector (equivalent to a diagonal matrix) for
   * weighting errors in miscellaneous continuous state variables `z`. Only used
   * for integrators that support error estimation.
   */
  const Eigen::VectorXd& get_misc_state_weight_vector() const {
    return z_weight_;
  }

  /**
   * Gets a mutable weighting vector (equivalent to a diagonal matrix) for
   * weighting errors in miscellaneous continuous state variables `z`. Only used
   * for integrators that support error estimation. Returns a VectorBlock
   * to make the values mutable without permitting changing the size of
   * the vector. Requires re-initializing the integrator after calling this
   * method. If Initialize() is not called afterward, an exception will be
   * thrown when attempting to call StepOnceAtMost(). If the caller sets
   * one of the entries to a negative value, an exception will be thrown
   * when the integrator is initialized.
   */
  Eigen::VectorBlock<Eigen::VectorXd> get_mutable_misc_state_weight_vector() {
    initialization_done_ = false;
    return z_weight_.head(z_weight_.rows());
  }

  /**
   * @}
   */

 protected:
  /// Resets any statistics particular to a specific integrator. The default
  /// implementation of this function does nothing. If your integrator
  /// collects its own statistics, you should re-implement this method and
  /// reset them there.
  virtual void DoResetStatistics() {}

  /// Evaluates the derivative function (and updates call statistics).
  /// Subclasses should call this function rather than calling
  /// system.CalcTimeDerivatives() directly.
  void CalcTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* dxdt) {
    get_system().CalcTimeDerivatives(context, dxdt);
    ++num_ode_evals_;
  }

  /// Evaluates the derivative function (and updates call statistics).
  /// Subclasses should call this function rather than calling
  /// system.CalcTimeDerivatives() directly. This version of this function
  /// exists to allow integrators to count AutoDiff'd systems in derivative
  /// function evaluations.
  template <typename U>
  void CalcTimeDerivatives(const System<U>& system,
                           const Context<U>& context,
                           ContinuousState<U>* dxdt) {
    system.CalcTimeDerivatives(context, dxdt);
    ++num_ode_evals_;
  }

  /**
   * Sets the working ("in use") accuracy for this integrator. The working
   * accuracy may not be equivalent to the target accuracy when the latter is
   * too loose or tight for an integrator's capabilities.
   * @sa get_accuracy_in_use()
   * @sa get_target_accuracy()
   */
  void set_accuracy_in_use(double accuracy) { accuracy_in_use_ = accuracy; }

  /**
   * Default code for advancing the continuous state of the system by a single
   * step of @p dt_max (or smaller, depending on error control). This particular
   * function is designed to be called directly by an error estimating
   * integrator's DoStep() method to effect error-controlled integration.
   * The integrator can effect error controlled integration without calling this
   * method, if the implementer so chooses, but this default method is expected
   * to function well in most circumstances.
   * @param[in] dt_max The maximum step size to be taken. The integrator may
   *               take a smaller step than specified to satisfy accuracy
   *               requirements or to respect the integrator's maximum step
   *               size.
   * @throws std::logic_error if integrator does not support error
   *                          estimation.
   * @note This function will shrink the integration step as necessary whenever
   *       the integrator's DoStep() fails to take the requested step
   *       e.g., due to integrator convergence failure.
   * @returns `true` if the full step of size @p dt_max is taken and `false`
   *          otherwise (i.e., a smaller step than @p dt_max was taken).
   */
  bool StepOnceErrorControlledAtMost(const T& dt_max);

  /**
   * Computes the infinity norm of a change in continuous state. We use the
   * infinity norm to capture the idea that, by providing accuracy requirements,
   * the user can indirectly specify error tolerances that act to limit the
   * largest error in any state vector component.
   * @returns the norm (a non-negative value)
   */
  T CalcStateChangeNorm(const ContinuousState<T>& dx_state) const;

  /**
   * Calculates adjusted integrator step sizes toward keeping state variables
   * within error bounds on the next integration step. Note that it is not
   * guaranteed that the (possibly) reduced step size will keep state variables
   * within error bounds; however, the process of (1) taking a trial
   * integration step, (2) calculating the error, and (3) adjusting the step
   * size can be repeated until convergence.
   * @param err
   *      The norm of the integrator error that was computed using
   *       @p current_step_size.
   * @param dt_was_artificially_limited
   *      `true` if step_size was artificially limited (by, e.g., stepping to
   *      a publishing time).
   * @param current_step_size
   *      The current step size on entry.
   * @returns a pair of types bool and T; the bool will be set to `true` if the
   *      new step size is at least as large as the current, `false` otherwise.
   *      The value of the T type will be set to the recommended next step size.
   */
  std::pair<bool, T> CalcAdjustedStepSize(const T& err,
                                          bool dt_was_artificially_limited,
                                          const T& current_step_size) const;

  /**
   * Derived classes can override this method to perform special
   * initialization. This method is called during the Initialize() method. This
   * default method does nothing.
   */
  virtual void DoInitialize() {}

  /**
   * Derived classes can override this method to perform routines when
   * Reset() is called. This default method does nothing.
   */
  virtual void DoReset() {}

  /**
   * Derived classes must implement this method to (1) integrate the continuous
   * portion of this system forward by a single step of size @p dt and
   * (2) set the error estimate (via get_mutable_error_estimate()). This
   * method is called during the default Step() method.
   * @param dt The integration step to take.
   * @returns `true` if successful, `false` if the integrator was unable to take
   *           a single step of size @p dt (due to, e.g., an integrator
   *           convergence failure).
   * @post If the time on entry is denoted `t`, the time and state will be
   *       advanced to `t+dt` if the method returns `true`; otherwise, the
   *       time and state should be reset to those at `t`.
   */
  virtual bool DoStep(const T& dt) = 0;

  /**
   * Gets an error estimate of the state variables recorded by the last call
   * to StepOnceFixedSize(). If the integrator does not support error
   * estimation, this function will return nullptr.
   */
  ContinuousState<T>* get_mutable_error_estimate() { return err_est_.get(); }

  // Sets the actual initial step size taken.
  void set_actual_initial_step_size_taken(const T& dt) {
    actual_initial_step_size_taken_ = dt;
  }

  /**
   *  Sets the size of the smallest-step-taken statistic as the result of a
   *  controlled integration step adjustment.
   */
  void set_smallest_adapted_step_size_taken(const T& dt) {
    smallest_adapted_step_size_taken_ = dt;
  }

  // Sets the largest-step-size-taken statistic.
  void set_largest_step_size_taken(const T& dt) {
    largest_step_size_taken_ = dt;
  }

  // Sets the "ideal" next step size (typically done via error control).
  void set_ideal_next_step_size(const T& dt) { ideal_next_step_size_ = dt; }

 private:
  // Validates that a smaller step size does not fall below the working minimum
  // and throws an exception if desired.
  void ValidateSmallerStepSize(const T& current_step_size,
                               const T& new_step_size) const {
    if (new_step_size < get_working_minimum_step_size() &&
        new_step_size < current_step_size &&  // Verify step adjusted downward.
        min_step_exceeded_throws_) {
      SPDLOG_DEBUG(drake::log(), "Integrator wants to select too small step "
          "size of {}; working minimum is ", new_step_size,
                   get_working_minimum_step_size());
      std::ostringstream str;
      str << "Error control wants to select step smaller than minimum" <<
           " allowed (" << get_working_minimum_step_size() << ")";
      throw std::runtime_error(str.str());
    }
  }

  // Updates the integrator statistics, accounting for a step just taken of
  // size dt.
  void UpdateStepStatistics(const T& dt) {
    // Handle first step specially.
    if (++num_steps_taken_ == 1) {
      set_actual_initial_step_size_taken(dt);
      set_largest_step_size_taken(dt);
    } else {
      if (dt > get_largest_step_size_taken()) set_largest_step_size_taken(dt);
    }

    // Update the previous step size.
    prev_step_size_ = dt;
  }

  // Steps the system forward exactly by @p dt, if possible, by calling DoStep.
  // Does necessary pre-initialization and post-cleanup. This method does not
  // update general integrator statistics (which are updated in the calling
  // methods), because error control might decide that it does not like the
  // result of the step and might "rewind" and take a smaller one.
  // @returns `true` if successful, `false` otherwise (due to, e.g., integrator
  //          convergence failure).
  // @sa DoStep()
  bool Step(const T& dt) {
    if (!DoStep(dt))
      return false;
    return true;
  }

  // Reference to the system being simulated.
  const System<T>& system_;

  // Pointer to the context.
  Context<T>* context_{nullptr};  // The trajectory Context.

  // Runtime variables.
  // For variable step integrators, this is set at the end of each step to guide
  // the next one.
  T ideal_next_step_size_{nan()};  // Indicates that the value is uninitialized.

  // The scaling factor to apply to an integration step size when an integrator
  // convergence failure occurs (to make convergence more likely on the next
  // attempt).
  // TODO(edrumwri): Allow subdivision factor to be user-tweakable.
  const double subdivision_factor_{0.5};

  // The accuracy being used.
  double accuracy_in_use_{nan()};

  // The maximum step size.
  T max_step_size_{nan()};

  // The minimum step size.
  T req_min_step_size_{0};

  // The last step taken by the integrator.
  T prev_step_size_{nan()};

  // Whether error-controlled integrator is running in fixed step mode. Value
  // is irrelevant for integrators without error estimation capabilities.
  bool fixed_step_mode_{false};

  // When the minimum step is exceeded, does the integrator throw an exception?
  bool min_step_exceeded_throws_{true};

  // Statistics.
  T actual_initial_step_size_taken_{nan()};
  T smallest_adapted_step_size_taken_{nan()};
  T largest_step_size_taken_{nan()};
  int64_t num_steps_taken_{0};
  int64_t num_ode_evals_{0};
  int64_t num_shrinkages_from_error_control_{0};
  int64_t num_shrinkages_from_substep_failures_{0};
  int64_t num_substep_failures_{0};

  // Applied as diagonal matrices to weight state change variables.
  Eigen::VectorXd qbar_weight_, z_weight_;

  // State copy for reversion during error-controlled integration.
  VectorX<T> xc0_save_;

  // The error estimate computed during integration with error control.
  std::unique_ptr<ContinuousState<T>> err_est_;

  // The pseudo-inverse of the matrix that converts time derivatives of
  // generalized coordinates to generalized velocities, multiplied by the
  // change in the generalized coordinates (used in state change norm
  // calculations).
  mutable std::unique_ptr<VectorBase<T>> pinvN_dq_change_;

  // Vectors used in state change norm calculations.
  mutable VectorX<T> unweighted_substate_change_;
  mutable std::unique_ptr<VectorBase<T>> weighted_q_change_;

  // Variable for indicating when an integrator has been initialized.
  bool initialization_done_{false};

  // This a workaround for an apparent bug in clang 3.8 in which
  // defining this as a static constexpr member kNaN failed to instantiate
  // properly for the AutoDiffXd instantiation (worked in gcc and MSVC).
  // Restore to sanity when some later clang is current.
  static constexpr double nan() {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double target_accuracy_{nan()};   // means "unspecified, use default"
  T req_initial_step_size_{nan()};  // means "unspecified, use default"
};

template <class T>
bool IntegratorBase<T>::StepOnceErrorControlledAtMost(const T& dt_max) {
  using std::isnan;
  using std::min;

  // Verify that the integrator supports error estimates.
  if (!supports_error_estimation()) {
    throw std::logic_error("StepOnceErrorControlledAtMost() requires error "
                               "estimation.");
  }

  // Save time, continuous variables, and time derivative because we'll possibly
  // revert time and state.
  const Context<T>& context = get_context();
  const T current_time = context.get_time();
  VectorBase<T>* xc =
      get_mutable_context()->get_mutable_continuous_state_vector();
  xc0_save_ = xc->CopyToVector();

  // Set the "current" step size.
  T current_step_size = get_ideal_next_step_size();
  if (isnan(current_step_size)) {
    // Integrator has not taken a step. Set the current step size to the
    // initial step size.
    current_step_size = get_initial_step_size_target();
    DRAKE_DEMAND(!isnan(current_step_size));
  }

  bool step_succeeded = false;
  do {
    // Constants used to determine whether modifications to the step size are
    // close enough to the desired step size to use the unadjusted originals.
    const double near_enough_smaller = 0.95;
    const double near_enough_larger = 1.001;

    // If we lose more than a small fraction of the step size we wanted
    // to take due to a need to stop at dt_max, make a note of that so the
    // step size adjuster won't try to grow from the current step.
    bool dt_was_artificially_limited = false;
    if (dt_max < near_enough_smaller * current_step_size) {
      // dt_max much smaller than current step size.
      dt_was_artificially_limited = true;
      current_step_size = dt_max;
    } else {
      if (dt_max < near_enough_larger * current_step_size)
        current_step_size = dt_max;  // dt_max is roughly current step.
    }

    // Limit the current step size.
    current_step_size = min(current_step_size, get_maximum_step_size());

    // Keep adjusting the integration step size until any integrator
    // convergence failures disappear.
    T adjusted_step_size = current_step_size;
    while (!Step(adjusted_step_size)) {
      SPDLOG_DEBUG(drake::log(), "Sub-step failed at {}", adjusted_step_size);
      adjusted_step_size *= subdivision_factor_;
      ValidateSmallerStepSize(current_step_size, adjusted_step_size);
      dt_was_artificially_limited = true;
      ++num_shrinkages_from_substep_failures_;
      ++num_substep_failures_;
    }
    current_step_size = adjusted_step_size;

    //--------------------------------------------------------------------
    T err_norm = CalcStateChangeNorm(*get_error_estimate());
    std::tie(step_succeeded, current_step_size) = CalcAdjustedStepSize(
        err_norm, dt_was_artificially_limited, current_step_size);
    SPDLOG_DEBUG(drake::log(), "Adjusted step size: {}", current_step_size);

    if (step_succeeded) {
      ideal_next_step_size_ = current_step_size;
      if (isnan(get_actual_initial_step_size_taken()))
        set_actual_initial_step_size_taken(current_step_size);

      // Record the adapted step size taken.
      if (isnan(get_smallest_adapted_step_size_taken()) ||
          (current_step_size < get_smallest_adapted_step_size_taken() &&
              current_step_size < dt_max))
        set_smallest_adapted_step_size_taken(current_step_size);
    } else {
      ++num_shrinkages_from_error_control_;

      // Reset the time, state, and time derivative at t0.
      get_mutable_context()->set_time(current_time);
      xc->SetFromVector(xc0_save_);
    }
  } while (!step_succeeded);

  return (current_step_size == dt_max);
}

template <class T>
T IntegratorBase<T>::CalcStateChangeNorm(
    const ContinuousState<T>& dx_state) const {
  using std::max;
  const Context<T>& context = get_context();
  const auto& system = get_system();

  // Get weighting matrices.
  const auto& qbar_v_weight = this->get_generalized_state_weight_vector();
  const auto& z_weight = this->get_misc_state_weight_vector();

  // Get the differences in the generalized position, velocity, and
  // miscellaneous continuous state vectors.
  const VectorBase<T>& dgq = dx_state.get_generalized_position();
  const VectorBase<T>& dgv = dx_state.get_generalized_velocity();
  const VectorBase<T>& dgz = dx_state.get_misc_continuous_state();

  // (re-)Initialize pinvN_dq_change_ and weighted_q_change_, if necessary.
  // Reinitialization might be required if the system state variables can
  // change during the course of the simulation.
  if (pinvN_dq_change_ == nullptr) {
    pinvN_dq_change_ = std::make_unique<BasicVector<T>>(dgv.size());
    weighted_q_change_ = std::make_unique<BasicVector<T>>(dgq.size());
  }
  DRAKE_DEMAND(pinvN_dq_change_->size() == dgv.size());
  DRAKE_DEMAND(weighted_q_change_->size() == dgq.size());

  // TODO(edrumwri): Acquire characteristic time properly from the system
  //                 (i.e., modify the System to provide this value).
  const double characteristic_time = 1.0;

  // Computes the infinity norm of the weighted velocity variables.
  unweighted_substate_change_ = dgv.CopyToVector();
  T v_nrm = qbar_v_weight.cwiseProduct(unweighted_substate_change_).
      template lpNorm<Eigen::Infinity>() * characteristic_time;

  // Compute the infinity norm of the weighted auxiliary variables.
  unweighted_substate_change_ = dgz.CopyToVector();
  T z_nrm = (z_weight.cwiseProduct(unweighted_substate_change_))
                .template lpNorm<Eigen::Infinity>();

  // Compute N * Wq * dq = N * Wꝗ * N+ * dq.
  unweighted_substate_change_ = dgq.CopyToVector();
  system.MapQDotToVelocity(context, unweighted_substate_change_,
                           pinvN_dq_change_.get());
  system.MapVelocityToQDot(
      context, qbar_v_weight.cwiseProduct(pinvN_dq_change_->CopyToVector()),
      weighted_q_change_.get());
  T q_nrm = weighted_q_change_->CopyToVector().
      template lpNorm<Eigen::Infinity>();
  SPDLOG_DEBUG(drake::log(), "dq norm: {}, dv norm: {}, dz norm: {}",
               q_nrm, v_nrm, z_nrm);

  // TODO(edrumwri): Record the worst offender (which of the norms resulted
  // in the largest value).
  // Infinity norm of the concatenation of multiple vectors is equal to the
  // maximum of the infinity norms of the individual vectors.
  return max(z_nrm, max(q_nrm, v_nrm));
}

template <class T>
std::pair<bool, T> IntegratorBase<T>::CalcAdjustedStepSize(
    const T& err, bool dt_was_artificially_limited,
    const T& current_step_size) const {
  using std::pow;
  using std::min;
  using std::max;
  using std::isnan;
  using std::isinf;

  // Magic numbers come from Simbody.
  const double kSafety = 0.9;
  const double kMinShrink = 0.1;
  const double kMaxGrow = 5.0;
  const double kHysteresisLow = 0.9;
  const double kHysteresisHigh = 1.2;

  /// Get the order for the integrator's error estimate.
  const int err_order = get_error_estimate_order();

  /// Set value for new step size to invalid value initially.
  T new_step_size(-1);

  // First, make a first guess at the next step size to use based on
  // the supplied error norm. Watch out for NaN!
  if (isnan(err) || isinf(err))  // e.g., integrand returned NaN.
    new_step_size = kMinShrink * current_step_size;
  else if (err == 0)  // A "perfect" step; can happen if no dofs for example.
    new_step_size = kMaxGrow * current_step_size;
  else  // Choose best step for skating just below the desired accuracy.
    new_step_size = kSafety * current_step_size *
                    pow(get_accuracy_in_use() / err, 1.0 / err_order);

  // If the new step is bigger than the old, don't make the change if the
  // old one was small for some unimportant reason (like reached a publishing
  // interval). Also, don't grow the step size if the change would be very
  // small; better to keep the step size stable in that case (maybe just
  // for aesthetic reasons).
  if (new_step_size > current_step_size) {
    if (dt_was_artificially_limited ||
        new_step_size < kHysteresisHigh * current_step_size)
      new_step_size = current_step_size;
  }

  // If we're supposed to shrink the step size but the one we have actually
  // achieved the desired accuracy last time, we won't change the step now.
  // Otherwise, if we are going to shrink the step, let's not be shy -- we'll
  // shrink it by at least a factor of kHysteresisLow.
  if (new_step_size < current_step_size) {
    if (err <= get_accuracy_in_use()) {
      new_step_size = current_step_size;  // not this time
    } else {
      T test_value = kHysteresisLow * current_step_size;
      new_step_size = min(new_step_size, test_value);
    }
  }

  // Keep the size change within the allowable bounds.
  T max_grow_step = kMaxGrow * current_step_size;
  T min_shrink_step = kMinShrink * current_step_size;
  new_step_size = min(new_step_size, max_grow_step);
  new_step_size = max(new_step_size, min_shrink_step);

  // Apply user-requested limits on min and max step size.
  // TODO(edrumwri): Introduce some feedback to the user when integrator wants
  // to take a smaller step than user has selected as the minimum. Options for
  // this feedback could include throwing a special exception, logging, setting
  // a flag in the integrator that allows throwing an exception, or returning
  // a special status from IntegrateAtMost().
  if (!isnan(get_maximum_step_size()))
    new_step_size = min(new_step_size, get_maximum_step_size());
  ValidateSmallerStepSize(current_step_size, new_step_size);
  new_step_size = max(new_step_size, get_working_minimum_step_size());

  return std::make_pair(new_step_size >= current_step_size, new_step_size);
}

template <class T>
typename IntegratorBase<T>::StepResult IntegratorBase<T>::IntegrateAtMost(
    const T& publish_dt, const T& update_dt, const T& boundary_dt) {
  if (!IntegratorBase<T>::is_initialized())
    throw std::logic_error("Integrator not initialized.");

  // Verify that update dt is positive.
  if (update_dt <= 0.0)
    throw std::logic_error("Update dt must be strictly positive.");

  // Verify that other dt's are non-negative.
  if (publish_dt < 0.0)
    throw std::logic_error("Publish dt is negative.");
  if (boundary_dt < 0.0)
    throw std::logic_error("Boundary dt is negative.");

  // The size of the integration step is the minimum of the time until the next
  // update event, the time until the next publish event, the boundary time
  // (i.e., the maximum time that the user wished to step to), and the maximum
  // step size (which may stretch slightly to hit a discrete event).

  // We report to the caller which event ultimately constrained the step size.
  // If multiple events constrained it equally, we prefer to report update
  // events over publish events, publish events over boundary step limits,
  // and boundary limits over maximum step size limits. The caller must
  // determine event simultaneity by inspecting the time.

  // The maintainer of this code is advised to consider that, while updates
  // and boundary times, may both conceptually be deemed events, the distinction
  // is made for a reason. If both an update and a boundary time occur
  // simultaneously, the following behavior should result:
  // (1) kReachedUpdateTime is returned, (2) Simulator::StepTo() performs the
  // necessary update, (3) IntegrateAtMost() is called with boundary_dt=0 and
  // returns kReachedBoundaryTime, and (4) the simulation terminates. This
  // sequence of operations will ensure that the simulation state is valid if
  // Simulator::StepTo() is called again to advance time further.

  // We now analyze the following simultaneous cases with respect to Simulator:
  //
  // { publish, update }
  // kReachedUpdateTime will be returned, an update will be followed by a
  // publish.
  //
  // { publish, update, max step }
  // kReachedUpdateTime will be returned, an update will be followed by a
  // publish.
  //
  // { publish, boundary time, max step }
  // kReachedPublishTime will be returned, a publish will be performed followed
  // by another call to this function, which should return kReachedBoundaryTime
  // (followed in rapid succession by StepTo(.) return).
  //
  // { publish, boundary time, max step }
  // kReachedPublishTime will be returned, a publish will be performed followed
  // by another call to this function, which should return kReachedBoundaryTime
  // (followed in rapid succession by StepTo(.) return).
  //
  // { publish, update, boundary time, maximum step size }
  // kUpdateTimeReached will be returned, an update followed by a publish
  // will then be performed followed by another call to this function, which
  // should return kReachedBoundaryTime (followed in rapid succession by
  // StepTo(.) return).

  // By default, the candidate dt is the next discrete update event.
  StepResult candidate_result = IntegratorBase<T>::kReachedUpdateTime;
  T dt = update_dt;

  // If the next discrete publish event is sooner than the next discrete update
  // event, the publish event becomes the candidate dt
  if (publish_dt < update_dt) {
    candidate_result = IntegratorBase<T>::kReachedPublishTime;
    dt = publish_dt;
  }

  // If the stop time (boundary time) is sooner than the candidate, use it
  // instead.
  if (boundary_dt < dt) {
    candidate_result = IntegratorBase<T>::kReachedBoundaryTime;
    dt = boundary_dt;
  }

  // If there is no continuous state, there will be no need to limit the
  // integration step size.
  if (get_context().get_continuous_state()->size() == 0) {
    Context<T>* context = get_mutable_context();
    context->set_time(context->get_time() + dt);
    return candidate_result;
  }

  // If all events are farther into the future than the maximum step
  // size times a stretch factor of 1.01, the maximum step size becomes the
  // candidate dt. Put another way, if the maximum step occurs right before
  // an update or a publish, the update or publish is done instead. In contrast,
  // we never step past boundary_dt, even if doing so would allow hitting a
  // publish or an update.
  const bool reached_boundary =
      (candidate_result == IntegratorBase<T>::kReachedBoundaryTime);
  const T& max_dt = IntegratorBase<T>::get_maximum_step_size();
  if ((reached_boundary && max_dt < dt) ||
      (!reached_boundary && max_dt * get_stretch_factor() < dt)) {
    candidate_result = IntegratorBase<T>::kTimeHasAdvanced;
    dt = max_dt;
  }

  if (dt < 0.0) throw std::logic_error("Negative dt.");

  // If error control is disabled, call the generic stepper. Otherwise, use
  // the error controlled method.
  bool full_step = true;
  if (this->get_fixed_step_mode()) {
    T adjusted_dt = dt;
    while (!Step(adjusted_dt)) {
      ++num_shrinkages_from_substep_failures_;
      ++num_substep_failures_;
      adjusted_dt *= subdivision_factor_;
      ValidateSmallerStepSize(dt, adjusted_dt);
      full_step = false;
    }
  } else {
    full_step = StepOnceErrorControlledAtMost(dt);
  }

  // Update generic statistics.
  UpdateStepStatistics(dt);

  if (full_step) {
    // If the integrator took the entire maximum step size we allowed above,
    // we report to the caller that a step constraint was hit, which may
    // indicate a discrete event has arrived.
    return candidate_result;
  } else {
    // Otherwise, we report to the caller that time has advanced, but no
    // discrete event has arrived.
    return IntegratorBase<T>::kTimeHasAdvanced;
  }
}

}  // namespace systems
}  // namespace drake
