#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
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
 * An abstract class for an integrator for ODEs and DAEs.
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
   */
  // TODO(edrumwri): Update this comment when stretch size is configurable.
  const T& get_maximum_step_size() const { return max_step_size_; }

  /**
   * Sets the minimum step size that may be taken by this integrator.
   * All integration steps will be at least this large.
   */
  void set_minimum_step_size(const T& min_step_size) {
    DRAKE_ASSERT(min_step_size >= 0.0);
    min_step_size_ = min_step_size;
  }

  /**
   * Gets the minimum step size that may be taken by this integrator.
   */
  const T& get_minimum_step_size() const { return min_step_size_; }

  /**
   * Resets the integrator to initial values, i.e., default construction
   * values.
   */
  void Reset() {
    // Kill the error estimate and weighting matrices.
    err_est_.reset();
    qbar_weight_.setZero(0);
    z_weight_.setZero(0);
    pinvN_dq_err_.reset();
    unweighted_err_.setZero(0);
    weighted_q_err_.reset();

    // Integrator no longer operates in fixed step mode.
    fixed_step_mode_ = false;

    // Statistics no longer valid.
    ResetStatistics();

    // Wipe out settings.
    min_step_size_ = nan();
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
    if (max_step_size_ < min_step_size_) {
      throw std::logic_error("Integrator maximum step size is less than the "
                             "minimum step size");
    }
    if (req_initial_step_size_ > max_step_size_) {
      throw std::logic_error("Requested integrator initial step size is larger "
                             "than the maximum step size.");
    }
    if (req_initial_step_size_ < min_step_size_) {
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
   * Integrates the system forward in time. Integrator must already have
   * been initialized or an exception will be thrown. The context will be
   * integrated forward by an amount that will never exceed the minimum of
   * `publish_dt`, `update_dt`, and `1.01 * get_maximum_step_size()`.
   * Error controlled integrators may take smaller steps.
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
   *          typically call `IntegratorBase::StepExactlyFixed()`.
   */
  // TODO(edrumwri): Make the stretch size configurable.
  StepResult StepOnceAtMost(const T& publish_dt, const T& update_dt,
                            const T& boundary_dt);

  /// Stepping function for integrators operating outside of simulation
  /// circumstances. This method is designed for integrator
  /// users that do not wish to consider publishing or discontinuous,
  /// mid-interval updates _and_ are using integrators with error control.
  /// @warning Users should simulate systems using `Simulator::StepTo()` in
  ///          place of this function (which was created for off-simulation
  ///          purposes), generally.
  /// @note Users desiring this functionality for integrators operating in
  ///       fixed step mode should use StepExactlyFixed().
  /// @param dt The non-negative integration step to take.
  /// @throws std::logic_error If the integrator has not been initialized or
  ///                          dt is negative **or** if the integrator
  ///                          is operating in fixed step mode.
  /// @sa StepExactlyFixed()
  void StepExactlyVariable(const T& dt) {
    using std::max;

    if (this->get_fixed_step_mode()) {
      throw std::logic_error("StepExactlyVariable() requires variable "
                             "stepping.");
    }
    const Context<T>& context = get_context();
    const T inf = std::numeric_limits<double>::infinity();
    T t_remaining = dt;

    // Note: A concern below is that the while loop while run forever because
    // t_remaining could be small, but not quite zero, if dt is relatively
    // small compared to the context time. In such a case, t_final will be
    // equal to context.get_time() in the expression immediately below,
    // context.get_time() will not change during the call to StepOnceAtMost(),
    // and t_remaining will be equal to zero.
    const T t_final = context.get_time() + t_remaining;
    do {
      StepOnceAtMost(inf, inf, t_remaining);
      t_remaining = t_final - context.get_time();
    } while (t_remaining > 0);
  }

  /// Stepping function for integrators operating outside of simulation
  /// circumstances. This method is designed for integrator
  /// users that do not wish to consider publishing or discontinuous,
  /// mid-interval updates. One such example application is that of direct
  /// transcription for trajectory optimization. In keeping with the naming
  /// semantics of this function, error controlled integration is not supported
  /// (though error estimates will be computed for integrators that support that
  /// feature).
  /// @warning Users should simulate systems using `Simulator::StepTo()` in
  ///          place of this function (which was created for off-simulation
  ///          purposes), generally.
  /// @note Users desiring this functionality for integrators not operating in
  ///       fixed step mode should use StepExactlyVariable()- these functions
  ///       are kept distinct to make clear to the caller which of these
  ///       functions uses fixed integration steps.
  /// @param dt The non-negative integration step to take.
  /// @throws std::logic_error If the integrator has not been initialized or
  ///                          dt is negative **or** if the integrator
  ///                          is not operating in fixed step mode.
  /// @sa StepExactlyVariable()
  void StepExactlyFixed(const T& dt) {
    if (!this->get_fixed_step_mode())
      throw std::logic_error("StepExactlyFixed() requires fixed stepping.");
    const T inf = std::numeric_limits<double>::infinity();
    StepOnceAtMost(inf, inf, dt);
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
    error_check_failures_ = 0;
  }

  /**
   * Returns the number of failures to accept an integration step due to
   * not meeting error tolerances since the last call to ResetStatistics()
   * or Initialize().
  */
  int64_t get_error_check_failures() const { return error_check_failures_; }

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
   *  @sa CalcErrorNorm()
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
  /**
   * Sets the working ("in use") accuracy for this integrator. The working
   * accuracy may not be equivalent to the target accuracy when the latter is
   * too loose or tight for an integrator's capabilities.
   * @sa get_accuracy_in_use()
   * @sa get_target_accuracy()
   */
  void set_accuracy_in_use(double accuracy) { accuracy_in_use_ = accuracy; }

  /**
   * Increments the count of integration step failures due to error tolerance
   * failure.
   */
  void report_error_check_failure() { ++error_check_failures_; }

  /**
   * Default code for taking a single error controlled step of @p dt_max
   * or smaller. This particular function can be called directly by
   * an error estimating integrator's DoStepAtMost() method to effect
   * error-controlled integration. The integrator can effect error controlled
   * integration without calling this method, if the implementer so chooses, but
   * this default method is expected to function well in most circumstances.
   * @param[in] dt_max The maximum step size to be taken. The integrator may
   *               take a smaller step than specified to satisfy accuracy
   *               requirements.
   * @param[in,out] derivs0 A pointer to the state derivatives at the system's
   *                current time (t0), *which must have already been evaluated
   *                at t0*. These values may be modified on return.
   * @throws std::logic_error if integrator does not support error
   *                          estimation.
   */
  void StepErrorControlled(const T& dt_max, ContinuousState<T>* derivs0);

  /**
   * Computes the infinity norm of the error estimate. We use the infinity norm
   * to capture the idea that, by providing accuracy requirements, the user
   * indirectly specifies error tolerances that act to  limit the largest error
   * in any state vector component.
   * @throws std::logic_error If the integrator does not support error
   *                          estimation.
   * @returns the norm (a non-negative value)
   */
  T CalcErrorNorm();

  /**
   * Calculates the adjusted integrator step size toward keeping state variables
   * within error bounds on the next integration step. Note that it is not
   * guaranteed that the (possibly) reduced step size will keep state variables
   * within error bounds; however, the process of (1) taking a trial
   * integration step, (2) calculating the error, and (3) adjusting the step
   * size- can be repeated until convergence.
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
   * Derived classes may re-implement this method to integrate the continuous
   * portion of this system forward by a *single step* of no greater size than
   * @p max_dt. This method is called during the StepOnceAtMost() method. This
   * default implementation simply calls DoStepOnceFixedSize(max_dt) and
   * returns `{ true, max_dt }`.
   * @param max_dt The maximum integration step to take.
   * @returns a std::pair<bool, T>, with the first element corresponding to
   *          `false` if the integrator does not take the full step of @p max_dt
   *           (and `true` otherwise) and the second element corresponding to
   *           the step size actually taken by the integrator.
   */
  virtual std::pair<bool, T> DoStepOnceAtMost(const T& max_dt);

  /**
   * Derived classes must implement this method to integrate the continuous
   * portion of this system forward exactly by a single step of size dt. This
   * method is called during the default DoStepOnceAtMost() method.
   * @param dt The integration step to take.
   */
  virtual void DoStepOnceFixedSize(const T& dt) = 0;

  /**
   * Updates the integrator statistics, accounting for a step just taken of
   * size dt.
   */
  void UpdateStatistics(const T& dt) {
    // Handle first step specially.
    if (++num_steps_taken_ == 1) {
      set_actual_initial_step_size_taken(dt);
      set_largest_step_size_taken(dt);
    } else {
      if (dt > get_largest_step_size_taken()) set_largest_step_size_taken(dt);
    }
  }

  /**
   * Gets an error estimate of the state variables recorded by the last call
   * to StepOnceFixedSize(). If the integrator does not support error
   * estimation, this function will return nullptr.
   */
  ContinuousState<T>* get_mutable_error_estimate() { return err_est_.get(); }

  /// Get the system state at the start of the last integration interval.
  const VectorX<T>& get_interval_start_state() const { return xc0_save_; }

  /**
   * Get the system state derivative at the start of the last integration
   * interval.
   */
  const VectorX<T>& get_interval_start_state_deriv() const {
    return xcdot0_save_;
  }

  /**
   * Get a mutable reference to the system state at the start of the last
   * integration interval.
   */
  VectorX<T>& get_mutable_interval_start_state() { return xc0_save_; }

  /**
   * Get a mutable reference to the system state derivative at the start of
   * the last integration interval.
   */
  VectorX<T>& get_mutable_interval_start_state_deriv() { return xcdot0_save_; }

 private:
  void set_ideal_next_step_size(const T& dt) { ideal_next_step_size_ = dt; }

  // Sets the actual initial step size taken.
  void set_actual_initial_step_size_taken(const T& dt) {
    actual_initial_step_size_taken_ = dt;
  }

  /**
   *  Sets the size of the smallest step taken as the result of a controlled
   *  integration step adjustment.
   */
  void set_smallest_adapted_step_size_taken(const T& dt) {
    smallest_adapted_step_size_taken_ = dt;
  }

  // Sets the largest step size taken
  void set_largest_step_size_taken(const T& dt) {
    largest_step_size_taken_ = dt;
  }

  // Sets the number of steps taken.
  void set_num_steps_taken(int64_t steps) { num_steps_taken_ = steps; }

  // Calls DoStepOnceFixedSize and does necessary pre-initialization and
  // post-cleanup. This method does not update general integrator statistics.
  void StepOnceAtFixedSize(const T& dt) {
    DoStepOnceFixedSize(dt);
    prev_step_size_ = dt;
  }

 private:
  // Reference to the system being simulated.
  const System<T>& system_;

  // Pointer to the context.
  Context<T>* context_{nullptr};  // The trajectory Context.

  // Runtime variables.
  // For variable step integrators, this is set at the end of each step to guide
  // the next one.
  T ideal_next_step_size_{nan()};  // Indicates that the value is uninitialized.

  // The accuracy being used.
  double accuracy_in_use_{nan()};

  // The maximum step size.
  T max_step_size_{nan()};

  // The minimum step size.
  T min_step_size_{nan()};

  // The last step taken by the integrator.
  T prev_step_size_{nan()};

  // Whether error-controlled integrator is running in fixed step mode. Value
  // is irrelevant for integrators without error estimation capabilities.
  bool fixed_step_mode_{false};

  // Statistics.
  T actual_initial_step_size_taken_{nan()};
  T smallest_adapted_step_size_taken_{nan()};
  T largest_step_size_taken_{nan()};
  int64_t num_steps_taken_{0};
  int64_t error_check_failures_{0};

  // Applied as diagonal matrices to weight error estimates.
  Eigen::VectorXd qbar_weight_, z_weight_;

  // State and time derivative copies for reversion during error-controlled
  // integration.
  VectorX<T> xc0_save_, xcdot0_save_;

  // The error estimate computed during integration with error control.
  std::unique_ptr<ContinuousState<T>> err_est_;

  // The pseudo-inverse of the matrix that converts time derivatives of
  // generalized coordinates to generalized velocities, multiplied by the
  // error in the generalized coordinates (used in error norm calculations).
  std::unique_ptr<VectorBase<T>> pinvN_dq_err_;

  // Vectors used in error norm calculations.
  VectorX<T> unweighted_err_;
  std::unique_ptr<VectorBase<T>> weighted_q_err_;

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
void IntegratorBase<T>::StepErrorControlled(const T& dt_max,
                                            ContinuousState<T>* derivs0) {
  using std::isnan;

  // Constants for step size growth and shrinkage.
  const double kDTShrink = 0.95;
  const double kDTGrow = 1.001;

  // Verify that the integrator supports error estimates.
  if (!supports_error_estimation())
    throw std::logic_error("StepErrorControlled() requires error estimation.");

  // Save time, continuous variables, and time derivative because we'll possibly
  // revert time and state.
  const Context<T>& context = get_context();
  const T current_time = context.get_time();
  VectorBase<T>* xc =
      get_mutable_context()->get_mutable_continuous_state_vector();
  xc0_save_ = xc->CopyToVector();
  xcdot0_save_ = derivs0->CopyToVector();

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
    // If we lose more than a small fraction of the step size we wanted
    // to take due to a need to stop at dt_max, make a note of that so the
    // step size adjuster won't try to grow from the current step.
    bool dt_was_artificially_limited = false;
    if (dt_max < kDTShrink * current_step_size) {
      // dt_max much smaller than current step size.
      dt_was_artificially_limited = true;
      current_step_size = dt_max;
    } else {
      if (dt_max < kDTGrow * current_step_size)
        current_step_size = dt_max;  // dt_max is roughly current step.
    }

    // Attempt to take the step.
    StepOnceAtFixedSize(current_step_size);

    //--------------------------------------------------------------------
    T err_norm = CalcErrorNorm();
    std::tie(step_succeeded, current_step_size) = CalcAdjustedStepSize(
        err_norm, dt_was_artificially_limited, current_step_size);

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
      report_error_check_failure();

      // Reset the time, state, and time derivative at t0.
      get_mutable_context()->set_time(current_time);
      xc->SetFromVector(get_interval_start_state());
      derivs0->SetFromVector(get_interval_start_state_deriv());
    }
  } while (!step_succeeded);
}

template <class T>
T IntegratorBase<T>::CalcErrorNorm() {
  using std::max;
  const Context<T>& context = get_context();
  const auto& system = get_system();

  // Verify that the integrator supports error estimation.
  if (!supports_error_estimation())
    throw std::logic_error("Integrator does not support error estimation.");

  // Get the error estimate and necessary vectors.
  const auto& err_est = get_error_estimate();

  // Get weighting matrices.
  const auto& qbar_v_weight = this->get_generalized_state_weight_vector();
  const auto& z_weight = this->get_misc_state_weight_vector();

  // Get the generalized position, velocity, and miscellaneous continuous
  // state vectors.
  const VectorBase<T>& gq_err = err_est->get_generalized_position();
  const VectorBase<T>& gv_err = err_est->get_generalized_velocity();
  const VectorBase<T>& gz_err = err_est->get_misc_continuous_state();

  // (re-)Initialize pinvN_dq_err_ and weighted_q_err_, if necessary.
  // Reinitialization might be required if the system state variables can
  // change during the course of the simulation.
  if (pinvN_dq_err_ == nullptr) {
    pinvN_dq_err_ = std::make_unique<BasicVector<T>>(gv_err.size());
    weighted_q_err_ = std::make_unique<BasicVector<T>>(gq_err.size());
  }
  DRAKE_DEMAND(pinvN_dq_err_->size() == gv_err.size());
  DRAKE_DEMAND(weighted_q_err_->size() == gq_err.size());

  // TODO(edrumwri): Acquire characteristic time properly from the system
  //                 (i.e., modify the System to provide this value).
  const double characteristic_time = 1.0;

  // Computes the infinity norm of the weighted velocity variables.
  unweighted_err_ = gv_err.CopyToVector();
  T v_nrm = qbar_v_weight.cwiseProduct(unweighted_err_).
      template lpNorm<Eigen::Infinity>() * characteristic_time;

  // Compute the infinity norm of the weighted auxiliary variables.
  unweighted_err_ = gz_err.CopyToVector();
  T z_nrm = (z_weight.cwiseProduct(unweighted_err_))
                .template lpNorm<Eigen::Infinity>();

  // Compute N * Wq * dq = N * Wꝗ * N+ * dq.
  unweighted_err_ = gq_err.CopyToVector();
  system.MapQDotToVelocity(context, unweighted_err_, pinvN_dq_err_.get());
  system.MapVelocityToQDot(
      context, qbar_v_weight.cwiseProduct(pinvN_dq_err_->CopyToVector()),
      weighted_q_err_.get());
  T q_nrm = weighted_q_err_->CopyToVector().template lpNorm<Eigen::Infinity>();

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
    if (err <= get_accuracy_in_use())
      new_step_size = current_step_size;  // not this time
    else
      new_step_size = min(new_step_size, kHysteresisLow * current_step_size);
  }

  // Keep the size change within the allowable bounds.
  new_step_size = min(new_step_size, kMaxGrow * current_step_size);
  new_step_size = max(new_step_size, kMinShrink * current_step_size);

  // Apply user-requested limits on min and max step size.
  // TODO(edrumwri): Introduce some feedback to the user when integrator wants
  // to take a smaller step than user has selected as the minimum. Options for
  // this feedback could include throwing a special exception, logging, setting
  // a flag in the integrator that allows throwing an exception, or returning
  // a special status from StepOnceAtMost().
  if (!isnan(get_maximum_step_size()))
    new_step_size = min(new_step_size, get_maximum_step_size());
  if (get_minimum_step_size() > 0) {
    if (new_step_size < get_minimum_step_size())
      throw std::runtime_error(
          "Error control wants to select step smaller "
          "than minimum allowed for this integrator.");
    new_step_size = max(new_step_size, get_minimum_step_size());
  }

  return std::make_pair(new_step_size >= current_step_size, new_step_size);
}

template <class T>
std::pair<bool, T> IntegratorBase<T>::DoStepOnceAtMost(const T& max_dt) {
  StepOnceAtFixedSize(max_dt);
  return std::make_pair(true, max_dt);
}

template <class T>
typename IntegratorBase<T>::StepResult IntegratorBase<T>::StepOnceAtMost(
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
  // necessary update, (3) StepOnceAtMost() is called with boundary_dt=0 and
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

  // If all events are farther into the future than the maximum step
  // size times a stretch factor of 1.01, the maximum step size becomes the
  // candidate dt. Put another way, if the maximum step occurs right before
  // an update or a publish, the update or publish is done instead. In contrast,
  // we never step past boundary_dt, even if doing so would allow hitting a
  // publish or an update.
  const bool reached_boundary =
      (candidate_result == IntegratorBase<T>::kReachedBoundaryTime);
  static constexpr double kMaxStretch = 1.01;  // Allow 1% step size stretch.
  const T& max_dt = IntegratorBase<T>::get_maximum_step_size();
  if ((reached_boundary && max_dt < dt) ||
      (!reached_boundary && max_dt * kMaxStretch < dt)) {
    candidate_result = IntegratorBase<T>::kTimeHasAdvanced;
    dt = max_dt;
  }

  if (dt < 0.0) throw std::logic_error("Negative dt.");
  bool step_size_was_dt;
  T actual_dt;
  std::tie(step_size_was_dt, actual_dt) = DoStepOnceAtMost(dt);

  // Update generic statistics.
  UpdateStatistics(actual_dt);

  if (step_size_was_dt) {
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
