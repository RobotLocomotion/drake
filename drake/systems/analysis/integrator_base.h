#pragma once

#include <limits>

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
   * Status returned by Step().
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
   * Indicates whether an integrator supports accuracy estimation.
   * Without accuracy estimation, target accuracy will be unused.
   */
  virtual bool supports_accuracy_estimation() const = 0;

  /**
   * Indicates whether an integrator supports stepping with error control.
   * Without stepping with error control, initial step size targets will be
   * unused.
   */
  virtual bool supports_error_control() const = 0;

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
   * information, see <pre>Sherman, et al. Procedia IUTAM 2:241-261 (2011),
   * Section 3.3.
   * http://dx.doi.org/10.1016/j.piutam.2011.04.023
   * @throws std::logic_error if integrator does not support accuracy estimation
   */
  // TODO(edrumwri): complain if integrator with error estimation wants to drop
  //                 below the minimum step size
  void set_target_accuracy(const T& accuracy) {
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
  const T& get_target_accuracy() const { return target_accuracy_; }

  /**
   * Gets the accuracy in use by the integrator. This number may differ from
   * the target accuracy if, for example, the user has requested an accuracy
   * not attainable or not recommended for the particular integrator.
   */
  const T& get_accuracy_in_use() const { return accuracy_in_use_; }

  /**
   * Sets the maximum step size for this integrator. For fixed step integrators
   * all steps will be taken at the maximum step size *unless* an event would
   * be missed.
   */
  void set_maximum_step_size(const T& max_step_size) {
    DRAKE_ASSERT(max_step_size >= 0.0);
    max_step_size_ = max_step_size;
  }

  /**
   * Gets the maximum step size for this integrator.
   */
  const T& get_maximum_step_size() const { return max_step_size_; }

  /**
   * Sets the minimum step size for this integrator. All integration steps
   * will be at least this large.
   */
  void set_minimum_step_size(const T& min_step_size) {
    DRAKE_ASSERT(min_step_size >= 0.0);
    min_step_size_ = min_step_size;
  }

  /**
   * Gets the minimum step size for this integrator
   */
  const T& get_minimum_step_size() const { return min_step_size_; }

  /**
   * An integrator must be initialized before being used. The pointer to the
   * context must be set before Initialize() is called (or an std::logic_error
   * will be thrown). If Initialize() is not called, an exception will be
   * thrown when attempting to call Step().
   * @throws std::logic_error If the context has not been set.
   */
  void Initialize() {
    if (!context_) throw std::logic_error("Context has not been set.");

    // Call the derived integrator initialization routine (if any)
    DoInitialize();

    initialization_done_ = true;
  }

  /** Request that the first attempted integration step have a particular size.
   * If no request is made, the integrator will estimate a suitable size
   * for the initial step attempt. *If the integrator does not support error
   * control*, this method will throw a std::logic_error (call
   * supports_error_control() to verify before calling this method). For
   * variable-step integration, the initial target will be treated as a maximum
   * step size subject to accuracy requirements and event occurrences. You can
   * find out what size *actually* worked with
   * `get_actual_initial_step_size_taken()`.
   * @throws std::logic_error If the integrator does not support error control.
   */
  void request_initial_step_size_target(const T& step_size) {
    if (!supports_error_control())
      throw std::logic_error(
          "Integrator does not support error control and "
          "user has initial step size target");
    req_initial_step_size_ = step_size;
  }

  /** Gets the first integration target step size. You can find out what size
   * *actually* was used with `get_actual_initial_step_size_taken()`.
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
  StepResult Step(const T& publish_dt, const T& update_dt);

  /** Forget accumulated statistics. These are reset to the values they have
   * post construction or immediately after `Initialize()`.
   */
  void ResetStatistics() {
    actual_initial_step_size_taken_ = nan();
    smallest_step_size_taken_ = nan();
    largest_step_size_taken_ = nan();
    num_steps_taken_ = 0;
  }

  /** The actual size of the successful first step. */
  const T& get_actual_initial_step_size_taken() const {
    return actual_initial_step_size_taken_;
  }

  /** The size of the smallest step taken since the last Initialize() call. */
  const T& get_smallest_step_size_taken() const {
    return smallest_step_size_taken_;
  }

  /** The size of the largest step taken since the last Initialize() call. */
  const T& get_largest_step_size_taken() const {
    return largest_step_size_taken_;
  }

  /** The number of integration steps taken since the last Initialize() call.
   */
  int64_t get_num_steps_taken() const { return num_steps_taken_; }

  /** Return the step size the integrator would like to take next, based
   * primarily on the integrator's accuracy prediction (variable step
   * integrators; will change as the simulation progresses) or using the fixed
   * step for fixed step integrators.
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

  // Gets a reference to the system.
  const System<T>& get_system() const { return system_; }

  // Indicates whether the integrator been initialized.
  bool is_initialized() const { return initialization_done_; }

 protected:
  /** Derived classes can override this method to perform special
   * initialization. This method is called during the Initialize() method.
   */
  virtual void DoInitialize() {}

  /** Derived classes must implement this method to integrate the continuous
   * portion of this system forward by a maximum step of dt. This method
   * is called during the Step() method.
   * @param dt The maximum integration step to take.
   * @returns *false* if the integrator does not take the full step of dt;
   *           otherwise, should return *true*
   */
  virtual bool DoStep(const T& dt) = 0;

  // Sets the ideal next step size.
  void set_ideal_next_step_size(const T& dt) { ideal_next_step_size_ = dt; }

  // Sets the actual initial step size taken.
  void set_actual_initial_step_size_taken(const T& dt) {
    actual_initial_step_size_taken_ = dt;
  }

  // Sets the smallest step size taken
  void set_smallest_step_size_taken(const T& dt) {
    smallest_step_size_taken_ = dt;
  }

  // Sets the largest step size taken
  void set_largest_step_size_taken(const T& dt) {
    largest_step_size_taken_ = dt;
  }

  // Sets the number of steps taken.
  void set_num_steps_taken(int64_t steps) { num_steps_taken_ = steps; }

  // Updates the integrator statistics.
  void UpdateStatistics(const T& dt) {
    // Handle first step specially.
    if (++num_steps_taken_ == 1) {
      set_actual_initial_step_size_taken(dt);
      set_smallest_step_size_taken(dt);
      set_largest_step_size_taken(dt);
    } else {
      if (dt < get_smallest_step_size_taken()) set_smallest_step_size_taken(dt);
      if (dt > get_largest_step_size_taken()) set_largest_step_size_taken(dt);
    }
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
  T accuracy_in_use_{nan()};

  // The maximum step size.
  T max_step_size_{nan()};

  // The minimum step size.
  T min_step_size_{nan()};

  // Statistics.
  T actual_initial_step_size_taken_{nan()};
  T smallest_step_size_taken_{nan()};
  T largest_step_size_taken_{nan()};
  int64_t num_steps_taken_{0};

  // Variable for indicating when an integrator has been initialized.
  bool initialization_done_{false};

  // This a workaround for an apparent bug in clang 3.8 in which
  // defining this as a static constexpr member kNaN failed to instantiate
  // properly for the AutoDiffXd instantiation (worked in gcc and MSVC).
  // Restore to sanity when some later clang is current.
  static constexpr double nan() {
    return std::numeric_limits<double>::quiet_NaN();
  }

  T target_accuracy_{nan()};        // means "unspecified, use default"
  T req_initial_step_size_{nan()};  // means "unspecified, use default"
};

template <class T>
typename IntegratorBase<T>::StepResult IntegratorBase<T>::Step(
    const T& publish_dt, const T& update_dt) {
  if (!IntegratorBase<T>::is_initialized())
    throw std::logic_error("Integrator not initialized.");

  // Sort the times for stopping- sort is stable to preserve preferences for
  // stopping. In decreasing order of preference for equal values, we want
  // the update step, then the publish step, then the maximum step size.
  const int64_t kDTs = 3;  // number of dt values to evaluate
  const T& max_step_size = IntegratorBase<T>::get_maximum_step_size();
  const T* stop_dts[kDTs] = {&update_dt, &publish_dt, &max_step_size};
  std::stable_sort(stop_dts, stop_dts + kDTs,
                   [](const T* t1, const T* t2) { return *t1 < *t2; });

  // Set dt and take the step.
  const T& dt = *stop_dts[0];
  if (dt < 0.0) throw std::logic_error("Negative dt.");
  bool result = DoStep(dt);

  // Return depending on the step taken.
  if (!result || &dt == &max_step_size)
    return IntegratorBase<T>::kTimeHasAdvanced;
  if (&dt == &publish_dt) return IntegratorBase<T>::kReachedPublishTime;
  if (&dt == &update_dt) return IntegratorBase<T>::kReachedUpdateTime;
  DRAKE_ABORT_MSG("Never should have reached here.");
}

}  // namespace systems
}  // namespace drake
