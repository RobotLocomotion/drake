#pragma once

#include <algorithm>
#include <limits>
#include <utility>

#include "drake/common/drake_assert.h"
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
    /** Took maximum number of steps without finishing integrating over the
     * interval. */
    kReachedStepLimit = 5,
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

  /**
   * Indicates whether an integrator supports error estimation.
   * Without error estimation, target accuracy will be unused.
   */
  virtual bool supports_error_estimation() const = 0;

  /** Request that the integrator attempt to achieve a particular accuracy for
   * the continuous portions of the simulation. Otherwise a default accuracy is
   * chosen for you. This may be ignored for fixed-step integration since
   * accuracy control requires variable step sizes. Additionally, the integrator
   * may support estimating accuracy but not provide the ability to control
   * it. You should call supports_accuracy_estimation() to ensure that the
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
   * @throws std::logic_error if integrator does not support accuracy
   *         estimation.
   */
  // TODO(edrumwri): complain if integrator with error estimation wants to drop
  //                 below the minimum step size
  void set_target_accuracy(double accuracy) {
    if (!supports_accuracy_estimation())
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
   * Sets the maximum step size that may be taken by this integrator. For fixed
   * step integrators all steps will be taken at the maximum step size *unless*
   * an event would be missed.
   */
  void set_maximum_step_size(const T& max_step_size) {
    DRAKE_ASSERT(max_step_size >= 0.0);
    max_step_size_ = max_step_size;
  }

  /**
   * Gets the maximum step size that may be taken by this integrator.
   */
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
   * An integrator must be initialized before being used. The pointer to the
   * context must be set before Initialize() is called (or an std::logic_error
   * will be thrown). If Initialize() is not called, an exception will be
   * thrown when attempting to call StepOnceAtMost().
   * @throws std::logic_error If the context has not been set or one of the
   *         scaling matrix coefficients is set to a negative value (this
   *         check is only performed for integrators that support error
   *         estimation).
   */
  void Initialize() {
    if (!context_) throw std::logic_error("Context has not been set.");

    // Allocate space for the error estimate.
    if (supports_error_estimation())
      err_est_ = system_.AllocateTimeDerivatives();

    // TODO(edrumwri): Compute v_scal_, z_scal_ automatically.
    // Set error scaling vectors if not already done.
    if (supports_error_estimation()) {
      const auto& contstate = context_->get_state().get_continuous_state();
      const int gv_size = contstate->get_generalized_velocity().size();
      const int misc_size = contstate->get_misc_continuous_state().size();
      if (v_scal_.size() != gv_size) v_scal_.setOnes(gv_size);
      if (z_scal_.size() != misc_size) z_scal_.setOnes(misc_size);

      // Verify that minimum values of the scaling matrices are non-negative.
      if (v_scal_.minCoeff() < 0 || z_scal_.minCoeff() < 0)
        throw std::logic_error("Scaling coefficient is less than zero.");
    }

    // Call the derived integrator initialization routine (if any)
    DoInitialize();

    initialization_done_ = true;
  }

  /** Request that the first attempted integration step have a particular size.
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
    if (!supports_error_estimation())
      throw std::logic_error(
          "Integrator does not support error estimation and "
          "user has initial step size target");
    req_initial_step_size_ = step_size;
  }

  /** Gets the target size of the first integration step. You can find out what
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
   * integrated forward *at most*
   * by the minimum of { `publish_dt`, `update_dt`, `get_maximum_step_size()` }.
   * Error controlled integrators may take smaller steps.
   * @param publish_dt The step size, >= 0.0 (exception will be thrown
   *        if this is not the case) at which the next publish will occur.
   * @param update_dt The step size, >= 0.0 (exception will be thrown
   *        if this is not the case) at which the next update will occur.
   * @throws std::logic_error If the integrator has not been initialized or one
   *                          of publish_dt or update_dt is negative.
   * @return The reason for the integration step ending.
   */
  StepResult StepOnceAtMost(const T& publish_dt, const T& update_dt);

  /** Forget accumulated statistics. These are reset to the values they have
   * post construction or immediately after `Initialize()`.
   */
  void ResetStatistics() {
    actual_initial_step_size_taken_ = nan();
    smallest_adapted_step_size_taken_ = nan();
    largest_step_size_taken_ = nan();
    prev_step_size_ = nan();
    ideal_next_step_size_ = nan();
    num_steps_taken_ = 0;
    error_test_failures_ = 0;
  }

  /** The actual size of the successful first step. */
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

  /** Return the step size the integrator would like to take next, based
   * primarily on the integrator's accuracy prediction. This value will not
   * be computed for integrators that do not support accuracy estimation and
   * NaN will be returned.
   */
  const T& get_ideal_next_step_size() const { return ideal_next_step_size_; }

  /** Returns a const reference to the internally-maintained Context holding
   * the most recent state in the trajectory. This is suitable for publishing or
   * extracting information about this trajectory step.
   */
  const Context<T>& get_context() const { return *context_; }

  /** Returns a mutable pointer to the internally-maintained Context holding
   * the most recent state in the trajectory. */
  Context<T>* get_mutable_context() { return context_; }

  /** Replace the pointer to the internally-maintained Context with a different
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
   *  Gets a constant reference to the system that is being integrated (and
   *  was provided to the constructor of the integrator).
   */
  const System<T>& get_system() const { return system_; }

  /// Indicates whether the integrator has been initialized.
  bool is_initialized() const { return initialization_done_; }

  /**
   *  Derived classes must override this function to return the integrator's
   *  error estimate. If the integrator does not provide an error estimate,
   *  the derived class implementation should return 0.
   */
  virtual int get_error_estimate_order() const = 0;

  /**
   * Gets the size of the last (previous) integration step. If no integration
   * steps have been taken, value will be NaN.
   */
  const T& get_previous_integration_step_size() const {
    return prev_step_size_;
  }

  /** Gets the error estimate (used only for integrators that support accuracy
   * estimation). If the error estimate has not been computed, nullptr
   * is returned.
   */
  const ContinuousState<T>* get_error_estimate() const {
    return err_est_.get();
  }

  /**
   * @name Scaling vector functions.
   *
   * This group of functions describes how errors for state variables with
   * heterogeneous units are scaled in the context of error-controlled
   * integration.
   *
   * A collection of state variables is generally defined in heterogenous units
   * (e.g. centimeters, angles, velocities, energy). Some of the state
   * variables cannot even be expressed in meaningful units, like
   * quaternions. We must be able to scale errors in these
   * heterogeneous variables to determine whether the error across all state
   * variables is acceptable given the user-specified accuracy
   * requirement for the simulation being performed (see
   * [Sherman 2011] for elaboration on that topic). The scaling factors are
   * normally determined automatically using the system's unit
   * dimensions, so *most users can stop reading now!*
   *
   * Users interested in more precise control over scaling may
   * use the methods in this group to access and modify weighting factors for
   * individual state variables. Changes to these weights can only be made prior
   * to integrator initialization, or as a result of an event which is followed
   * by re-initialization.
   *
   * <h4>How to choose weights</h4>
   *
   * The weight `wᵢ` for a state variable `xᵢ` should be
   * chosen so that the product `wᵢ * dxᵢ` is unitless, and in particular is 1
   * when `dxᵢ` represents a "unit effect" of state variable `xᵢ`; that is, the
   * change in `xᵢ` that produces a unit change in some quantity of interest in
   * the system being simulated. The user-specified integration accuracy will
   * then operate upon this weighted error. For example, if a "characteristic
   * length" for the system being simulated is 0.1 meters, and `x₀` is a
   * length variable measured in meters, then `w₀` should be 10 so that
   * `w₀*dx₀=1` when `dx₀=0.1`. For angles representing pointing accuracy we
   * typically assume a "unit angle" is one radian (about 60 degrees), so if x₁
   * is a pointing angle then w₁=1 is an appropriate weight. We can scale
   * an error vector `e=[dx₀ dx₁]` to a unitless fractional error
   * `f=[w₀*dx₀ w₁*dx₁]`. Now to achieve a given accuracy `a`, say `a=.0001`, we
   * need only check that `|fᵢ|<=a` for each element `i` of `f`. We'll be more
   * precise about this below.
   *
   * <h4>How the weights are used</h4>
   *
   * Separate weights are determined for each of
   * - `n` generalized quasi-coordinates `ꝗ`  (position variables in the
   *   velocity variable space), and
   * - `nz` miscellaneous state variables `z`.
   *
   * Weights on the generalized velocity variables `v (= dꝗ/dt)` are derived
   * directly from the weights on `ꝗ`, scaled by a unit time. Weights on the
   * actual `nq` generalized coordinates (which are generally not equal to the
   * `n` quasi-coordinates and can include unit quaternions) can
   * be calculated efficiently from weights on the quasi-coordinates, using the
   * relationship <pre>
   *        dq/dt = N(q) v
   *   =>      dq = N(q) dꝗ
   * </pre>
   *
   * The errors in the `ꝗ` and `z` variables are scaled by the diagonal elements
   * of diagonal weighting matrices Wꝗ and Wz, respectively. In the absence of
   * other information, the default for all scaling values is one, so `Wꝗ` and
   * `Wz` are `n × n` and `nz × nz` identity matrices. The weighting matrix Wv
   * for the velocity variables is just `Wv=τ*Wꝗ` where τ is a "characteristic
   * time" for the system, that is, the quantity in time units that represents a
   * significant evolution of the trajectory. This does *not* mean the fastest
   * oscillation in the system but rather the time over which velocity errors
   * may propagate before their position error effects are noticed. Typically
   * `τ=1.0` or `0.1` seconds for human-scale mechanical systems. Note that
   * larger values of `τ` are more conservative since they increase the velocity
   * weights.
   *
   * The block-diagonal weighting matrix `Wq` on generalized coordinates
   * `q` is calculated from `N` and `Wꝗ` (see below).
   *
   * In addition to these weights, accuracy for some of the state variables is
   * maintained to a *relative* error (e.g. 0.01%) while others are maintained
   * to an absolute tolerance. Even relative-error variables must be evaluated
   * against absolute tolerances when they are near zero where relative errors
   * can be undefined or unreasonably strict.
   *
   * The weighting matrices `Wq`, `Wv`, and `Wz` are used to compute a weighted
   * infinity norm as follows. Although `Wv` and `Wz` are constant, the actual
   * weightings may be state dependent for relative-error calculations. First
   * define adjusted block diagonal matrix `E=diag(Eq,Ev,Ez)` as follows:
   * <pre>
   *   Eq = Wq
   *   Ev: Ev(i,i) = { min(Wv(i,i), 1/|vᵢ|)     if vᵢ is relative
   *                 { Wv(i,i)                  if vᵢ is absolute
   *   Ez: Ez(i,i) = { min(Wz(i,i), 1/|zᵢ|)     if zᵢ is relative
   *                 { Wz(i,i)                  if zᵢ is absolute
   * </pre>
   * (`Ev` and `Ez` are diagonal.) A `v` or `z` will be maintained to relative
   * accuracy unless (a) it is "close" to zero, or (b) the variable has been
   * defined as requiring absolute accuracy. Position variables `q` are always
   * maintained to absolute accuracy (see [Sherman 2011] for rationale).
   *
   * Now given an error estimate vector `e=[eq ev ez]`, the vector `f=E*e`
   * can be considered to provide a unitless fractional error for each of the
   * state variables. To achieve a given user-specified accuracy `a`, we require
   * that norm_inf(`f`) <= `a`. (That is, no element of `f` can have absolute
   * value larger than `a`.)
   *
   * <h4>Determining weights for q</h4>
   *
   * The kinematic differential equations `qdot=N(q)*v` employ an `nq × n`
   * matrix `N`. By construction, this relationship is invertible using `N`'s
   * left pseudo-inverse `N⁺` so that `v=N⁺ qdot` and `N⁺ N = I` (the identity
   * matrix); however, `N N⁺ != I`, as `N` has more rows than columns generally.
   * [Nikravesh 1988] shows how such a matrix `N` can be determined and provides
   * more information. Given this relationship between `N` and `N⁺`, we can
   * relate scaled errors in configuration coordinates `q` to scaled errors in
   * generalized quasi-coordinates `ꝗ`, as the following derivation shows: <pre>
   *            v = N⁺ qdot         Inverse kinematic differential equation
   *        dꝗ/dt = N⁺ dq/dt        Use synonyms for v and qdot
   *           dꝗ = N⁺ dq           Change time derivatives to differentials
   *        Wꝗ dꝗ = Wꝗ N⁺ dq        Pre-multiply both sides by Wꝗ
   *      N Wꝗ dꝗ = N Wꝗ N⁺ dq      Pre-multiply both sides by N
   *      N Wꝗ dꝗ = Wq dq           Define Wq := N Wꝗ N⁺
   * </pre>
   * The last equation shows that `Wq` as defined above provides the expected
   * relationship between the weighted `ꝗ` variables in velocity space and the
   * weighted `q` variables in configuration space.
   *
   * Note that generalized quasi-coordinates `ꝗ` can only be defined locally for
   * a particular configuration `q`, such that the generalized velocities
   * `v=dꝗ/dt`. There is in general no meaningful set of `n` generalized
   * coordinates which can be differentiated with respect to time to yield `v`.
   * For example, the Hairy Ball Theorem implies that it is not possible for
   * three orientation variables to represent all 3D rotations without
   * singularities, yet three velocity variables can repesent angular velocity
   * in 3D without singularities.
   *
   * Finally, note that a diagonal entry of one of the scaling matrices can
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
   *  Gets the scaling vector (equivalent to a diagonal matrix) for generalized
   *  coordinate and velocity state variables. Only used for integrators that
   *  support accuracy estimation.
   */
  const Eigen::VectorXd& get_generalized_state_scaling_vector() const {
    return v_scal_;
  }

  /**
   *  Gets a mutable scaling vector (equivalent to a diagonal matrix) for
   *  generalized coordinate and velocity state variables. Only used for
   *  integrators that support accuracy estimation. Returns a VectorBlock
   *  to make the values mutable without permitting changing the size of
   *  the vector. Requires re-initializing the integrator after calling
   *  this method; if Initialize() is not called afterward, an exception will be
   *  thrown when attempting to call StepOnceAtMost(). If the caller sets
   *  one of the entries to a negative value, an exception will be thrown
   *  when the integrator is initialized.
   */
  Eigen::VectorBlock<Eigen::VectorXd>
    get_mutable_generalized_state_scaling_vector() {
      initialization_done_ = false;
      return v_scal_.head(v_scal_.rows());
  }

  /**
   *  Gets the scaling vector (equivalent to a diagonal matrix) for
   *  miscellaneous continuous state variables. Only used for integrators that
   *  support accuracy estimation.
   */
  const Eigen::VectorXd& get_misc_state_scaling_vector() const {
    return z_scal_;
  }

  /**
   *  Gets a mutable scaling vector (equivalent to a diagonal matrix) for
   *  miscellaneous continuous state variables. Only used for integrators that
   *  support accuracy estimation. Returns a VectorBlock
   *  to make the values mutable without permitting changing the size of
   *  the vector. Requires re-initializing the integrator after calling this
   *  method. If Initialize() is not called afterward, an exception will be
   *  thrown when attempting to call StepOnceAtMost(). If the caller sets
   *  one of the entries to a negative value, an exception will be thrown
   *  when the integrator is initialized.
   */
  Eigen::VectorBlock<Eigen::VectorXd> get_mutable_misc_state_scaling_vector() {
    initialization_done_ = false;
    return z_scal_.head(z_scal_.rows());
  }

  /**
   * @}
   */

 protected:
  /**
   * Returns the number of failures to accept an integration step due to
   * not meeting error tolerances since the last call to ResetStatistics()
   * or Initialize().
  */
  int64_t get_error_test_failures() const { return error_test_failures_; }

  /**
   * Increments the number of integration step failures due to error tolerance
   * failure.
   */
  void report_error_test_failure() { ++error_test_failures_; }

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
   * @throws std::logic_error if integrator does not support accuracy
   *                          estimation.
   */
  void StepErrorControlled(const T& dt_max,
                                  ContinuousState<T>* derivs0);

  /**
   * Computes the infinity norm of the error estimate. We use the infinity norm
   * to capture the idea that, by providing accuracy requirements, the user
   * indirectly specifies error tolerances that act to  limit the largest error
   * in any state vector component.
   * @throws std::logic_error If the integrator does not support accuracy
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
   * size- can be repeated until convergence. This code adapted from Simbody.
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

  /** Derived classes can override this method to perform special
   * initialization. This method is called during the Initialize() method. This
   * default method does nothing.
   */
  virtual void DoInitialize() {}

  /** Derived classes may re-implement this method to integrate the continuous
   * portion of this system forward by a single step of no greater size than
   * dt_max. This method is called during the StepOnceAtMost() method. This
   * default implementation simply calls DoStepOnceFixedSize(dt_max).
   * @param dt The maximum integration step to take.
   * @returns `false` if the integrator does not take the full step of dt;
   *           otherwise, should return `true`.
   */
  virtual bool DoStepOnceAtMost(const T& dt_max);

  /** Derived classes must implement this method to integrate the continuous
   * portion of this system forward exactly by a single step of size dt. This
   * method is called during the default DoStepOnceAtMost() method.
   * @param dt The integration step to take.
   */
  virtual void DoStepOnceFixedSize(const T& dt) = 0;

  /** Updates the integrator statistics, accounting for a step just taken of
   *  size dt.
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

 private:
  void set_ideal_next_step_size(const T& dt) { ideal_next_step_size_ = dt; }

  // Sets the actual initial step size taken.
  void set_actual_initial_step_size_taken(const T& dt) {
    actual_initial_step_size_taken_ = dt;
  }

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

  // The last step taken by the integrator
  T prev_step_size_{nan()};

  // Statistics.
  T actual_initial_step_size_taken_{nan()};
  T smallest_adapted_step_size_taken_{nan()};
  T largest_step_size_taken_{nan()};
  int64_t num_steps_taken_{0};
  int64_t error_test_failures_{0};

  // Vectors that will be applied as diagonal matrices to scale error estimates.
  Eigen::VectorXd v_scal_, z_scal_;

  // State and time derivative copies for reversion during error-controlled
  // integration.
  VectorX<T> xc0_save_, xcdot0_save_;

  // The error estimate computed during integration with error control.
  std::unique_ptr<ContinuousState<T>> err_est_;

  // Vectors used in error norm calculations.
  std::unique_ptr<VectorBase<T>> pinvN_dq_err_;
  VectorX<T> unscaled_err_;
  std::unique_ptr<VectorBase<T>> scaled_q_err_;

  // Variable for indicating when an integrator has been initialized.
  bool initialization_done_{false};

  // This a workaround for an apparent bug in clang 3.8 in which
  // defining this as a static constexpr member kNaN failed to instantiate
  // properly for the AutoDiffXd instantiation (worked in gcc and MSVC).
  // Restore to sanity when some later clang is current.
  static constexpr double nan() {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double target_accuracy_{nan()};        // means "unspecified, use default"
  T req_initial_step_size_{nan()};       // means "unspecified, use default"
};

template <class T>
void IntegratorBase<T>::StepErrorControlled(const T& dt_max,
                                            ContinuousState<T>* derivs0) {
  using std::isnan;

  // Constants for integration growth and shrinkage.
  const double kDTShrink = 0.95;
  const double kDTGrow = 1.001;

  // Verify that the integrator supports error estimates.
  if (!supports_error_estimation())
    throw std::logic_error(
        "StepErrorControlled() requires accuracy "
        "estimation.");

  // Save time, continuous variables, and time derivative because we'll possibly
  // revert time and state.
  auto& context = get_context();
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
    T err_nrm = CalcErrorNorm();
    std::tie(step_succeeded, current_step_size) = CalcAdjustedStepSize(err_nrm,
                                                    dt_was_artificially_limited,
                                                    current_step_size);
    if (!step_succeeded) {
      report_error_test_failure();

      // Record the adapted step size taken.
      if (isnan(get_smallest_adapted_step_size_taken()) ||
          get_smallest_adapted_step_size_taken() < current_step_size)
        set_smallest_adapted_step_size_taken(current_step_size);

      // Reset the time, state, and time derivative at t0.
      get_mutable_context()->set_time(current_time);
      xc->SetFromVector(get_interval_start_state());
      derivs0->SetFromVector(get_interval_start_state_deriv());
    } else {  // Step succeeded.
      ideal_next_step_size_ = current_step_size;
      if (isnan(get_actual_initial_step_size_taken()))
        set_actual_initial_step_size_taken(current_step_size);
    }
  } while (!step_succeeded);
}

template <class T>
T IntegratorBase<T>::CalcErrorNorm() {
  using std::max;
  const auto& context = get_context();
  const auto& system = get_system();

  // Verify that the integrator supports accuracy estimation.
  if (!supports_error_estimation())
    throw std::logic_error("Integrator does not support accuracy estimation.");

  // Get the error estimate and necessary vectors.
  const auto& err_est = get_error_estimate();
  auto& unscaled_err = unscaled_err_;
  auto& pinvN_dq_err = pinvN_dq_err_;
  auto& scaled_q_err = scaled_q_err_;

  // Get scaling matrices.
  const auto& v_scal = get_generalized_state_scaling_vector();
  const auto& z_scal = get_misc_state_scaling_vector();

  // Get the generalized position, velocity, and miscellaneous continuous
  // state vectors.
  const VectorBase<T>& gq_err = err_est->get_generalized_position();
  const VectorBase<T>& gv_err = err_est->get_generalized_velocity();
  const VectorBase<T>& gz_err = err_est->get_misc_continuous_state();

  // (re-)Initialize pinvN_dq_err_ and scaled_q_err_, if necessary.
  // Reinitialization might be required if the system state variables can
  // change during the course of the simulation.
  if (pinvN_dq_err == nullptr) {
    pinvN_dq_err = std::make_unique<BasicVector<T>>(gv_err.size());
    scaled_q_err = std::make_unique<BasicVector<T>>(gq_err.size());
  }
  DRAKE_DEMAND(pinvN_dq_err->size() == gv_err.size());
  DRAKE_DEMAND(scaled_q_err->size() == gq_err.size());

  // Computes the infinity norm of the un-scaled velocity variables.
  unscaled_err = gv_err.CopyToVector();
  T v_nrm = (v_scal * unscaled_err).template lpNorm<Eigen::Infinity>();

  // Compute the infinity norm of the scaled auxiliary variables.
  unscaled_err = gz_err.CopyToVector();
  T z_nrm = (z_scal * unscaled_err).template lpNorm<Eigen::Infinity>();

  // Compute Wq * dq = N * Wv * N+ * dq.
  unscaled_err = gq_err.CopyToVector();
  system.MapConfigurationDerivativesToVelocity(context, unscaled_err,
                                               pinvN_dq_err.get());
  system.MapVelocityToConfigurationDerivatives(
      context, v_scal * pinvN_dq_err->CopyToVector(), scaled_q_err.get());
  T q_nrm =
      scaled_q_err->CopyToVector().template lpNorm<Eigen::Infinity>();

  // TODO(edrumwri): Record the worst offender.
  // Infinity norm of the concatenation of multiple vectors is equal to the
  // maximum of the infinity norms of the individual vectors.
  return max(z_nrm, max(q_nrm, v_nrm));
}

template <class T>
std::pair<bool, T> IntegratorBase<T>::CalcAdjustedStepSize(const T& err,
                                             bool dt_was_artificially_limited,
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
  int err_order = get_error_estimate_order();

  /// Indicator for new step size not set properly.
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
  if (get_maximum_step_size() < std::numeric_limits<double>::infinity())
    new_step_size = min(new_step_size, get_maximum_step_size());
  if (get_minimum_step_size() > 0) {
    if (new_step_size < get_minimum_step_size())
      throw std::runtime_error("Error control wants to select step smaller "
                                   "than minimum allowed for this integrator.");
    new_step_size = max(new_step_size, get_minimum_step_size());
  }

  // Note: this is an odd definition of success. It only works because we're
  // refusing to shrink the step size above if accuracy was achieved.
  bool success = (new_step_size >= current_step_size);
  return std::make_pair(success, new_step_size);
}

template <class T>
bool IntegratorBase<T>::DoStepOnceAtMost(const T& max_dt) {
  StepOnceAtFixedSize(max_dt);
  return true;
}

template <class T>
typename IntegratorBase<T>::StepResult IntegratorBase<T>::StepOnceAtMost(
    const T& publish_dt, const T& update_dt) {
  if (!IntegratorBase<T>::is_initialized())
    throw std::logic_error("Integrator not initialized.");

  // Sort the times for stopping- sort is stable to preserve preferences for
  // stopping. In decreasing order of preference for equal values, we want
  // the update step, then the publish step, then the maximum step size.
  const int64_t kDTs = 3;  // Number of dt values to evaluate.
  const T& max_step_size = IntegratorBase<T>::get_maximum_step_size();
  const T* stop_dts[kDTs] = {&update_dt, &publish_dt, &max_step_size};
  std::stable_sort(stop_dts, stop_dts + kDTs,
                   [](const T* t1, const T* t2) { return *t1 < *t2; });

  // Set dt and take the step.
  const T& dt = *stop_dts[0];
  if (dt < 0.0) throw std::logic_error("Negative dt.");
  bool result = DoStepOnceAtMost(dt);

  // Update generic statistics.
  UpdateStatistics(dt);

  // Return depending on the step taken.
  if (!result || &dt == &max_step_size)
    return IntegratorBase<T>::kTimeHasAdvanced;
  if (&dt == &publish_dt) return IntegratorBase<T>::kReachedPublishTime;
  if (&dt == &update_dt) return IntegratorBase<T>::kReachedUpdateTime;
  DRAKE_ABORT_MSG("Never should have reached here.");
}

}  // namespace systems
}  // namespace drake
