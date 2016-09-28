#pragma once

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/vector_base.h"
#include "drake/systems/vector.h"

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

TODO(edrumwri): consider taking out kReachedStepLimit
*/

/**
 * An abstract class for an integrator for ODEs and DAEs.
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 *
 * Fixed step integrators derived from this class should accept a maximum
 * step size argument in the constructor to signal to remind the user to
 * select this fixed step size.
 */
template <class T>
class IntegratorBase {
 public:
  // Status returned by Step()
  /**
   * When a step is successful, it will return an indication of what caused it
   * to stop where it did. When unsuccessful it will throw an exception so you
   * won't see any return value. When return of control is due ONLY to reaching
   * a report time, (status is ReachedReportTime) the context may return an
   * interpolated value at an earlier time.
   *
   * Note: the simulation step must always end at an update time but can end
   * after a publish time.

   **/
  enum StepResult {
    /**
     * Indicates a publish time has been reached but not an update time.
     */
    kReachedPublishTime = 1,
    /** localized an event; this is the *before* state (interpolated) **/
    kReachedZeroCrossing = 2,
    /**
     * Indicates that integration terminated at an update time/event.
     */
    kReachedUpdateTime = 3,
    /**
     * user requested control whenever an internal step is successful;
     */
    kTimeHasAdvanced = 4,
    /** took maximum number of steps without finishing integrating over the
     * interval **/
    kReachedStepLimit = 5,
  };

  /**
   * @param system a reference to the system to be integrated; the integrator
   *               will maintain a reference to the system in perpetuity
   * @param context a pointer to a writeable context. The integrator will
   *                advance the system state using the pointer to this context.
   *                The pointer to the context will be maintained internally.
   *                The integrator must not outlive the context.
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
  virtual bool does_support_accuracy_estimation() const = 0;

  /**
   * Indicates whether an integrator supports stepping with error control.
   * Without stepping with error control, initial step size targets will be
   * unused.
   */
  virtual bool does_support_error_control() const = 0;

  /** Request that the integrator attempt to achieve a particular accuracy for
   * the continuous portions of the simulation. Otherwise a default accuracy is
   * chosen for you. This may be ignored for fixed-step integration since
   * accuracy control requires variable step sizes. Alternatively, the
   * integrator may log a message- assuming the integrator supports this
   * capability- when the designated accuracy has not been achieved.
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
   * TODO(edrumwri): complain if integrator with error estimation wants to drop
   *                 below the minimum step size
   **/
  virtual void set_target_accuracy(double accuracy) {
    target_accuracy_ = accuracy;
    accuracy_in_use_ = accuracy;
  }

  /**
   *   Gets the target accuracy.
   */
  virtual double get_target_accuracy() const { return target_accuracy_; }

  /**
   * Gets the accuracy in use by the integrator
   */
  virtual double get_accuracy_in_use() const { return accuracy_in_use_; }

  /**
   * Sets the maximum step size for this integrator
   */
  virtual void set_maximum_step_size(const T& max_step_size) {
    DRAKE_ASSERT(max_step_size >= (double) 0.0);
    max_step_size_ = max_step_size;
  }

  /**
   * Gets the maximum step size for this integrator
   */
  virtual const T& get_maximum_step_size() const { return max_step_size_; }

  /**
   * Sets the minimum step size for this integrator
   */
  virtual void set_minimum_step_size(const T& min_step_size) {
    DRAKE_ASSERT(min_step_size >= (double) 0.0);
    min_step_size_ = min_step_size;
  }

  /**
   * Gets the minimum step size for this integrator
   */
  virtual const T& get_minimum_step_size() const { return min_step_size_; }

  /**
   *   Integrator must be initialized before being used.
   */
  virtual void Initialize() {
    DRAKE_ASSERT(context_);
    initialization_done_ = true;
  }

  /** Request that the first attempted integration step have a particular size.
   * Otherwise the integrator will estimate a suitable size for the initial step
   * attempt. For fixed-step integration, all steps will be taken at this step
   * size. For variable-step integration this will be treated as a maximum size
   * subject to accuracy requirements and event occurrences. You can find out
   * what size *actually* worked with `get_actual_initial_step_size_taken()`.
   * Note that this function is virtual b/c so that fixed step integrators
   * exhibit the user-expected behavior without having to keep track of two
   * variable settings (step size and requested initial step size).
   */
  virtual void request_initial_step_size_target(const T& step_size) {
    req_initial_step_size_ = step_size;
  }

  /** Get the first integration step target size. Otherwise the integrator will
   * estimate a suitable size for the initial step attempt. For fixed-step
   * integration, all steps will be taken at this step size. For variable-step
   * integration this will be treated as a maximum size subject to accuracy
   * requirements and event occurrences. You can find out what size *actually*
   * worked with `get_actual_initial_step_size_taken()`. **/
  virtual const T& get_initial_step_size_target() const {
    return req_initial_step_size_;
  }

  /** Advance the System's trajectory by at most update_dt. Initialize() must
   * be called before making any calls to Step(), or an exception will
   * be thrown.
   **/
  virtual StepResult Step(const T& publish_dt, const T& update_dt) = 0;

  /** Forget accumulated statistics. These are reset to the values they have
   * post construction or immediately after `Initialize()`.
   **/
  void ResetStatistics() {
    actual_initial_step_size_taken_ = nan();
    smallest_step_size_taken_ = nan();
    largest_step_size_taken_ = nan();
    num_steps_taken_ = 0;
  }

  /** What what the actual size of the successful first step? **/
  const T& get_actual_initial_step_size_taken() const {
    return actual_initial_step_size_taken_;
  }

  /** What was the size of the smallest step taken since the last
   * Initialize() call?
   **/
  const T& get_smallest_step_size_taken() const {
    return smallest_step_size_taken_;
  }

  /** What was the size of the largest step taken since the last
   * @return Initialize() call?
   **/
  const T& get_largest_step_size_taken() const {
    return largest_step_size_taken_;
  }

  /** How many integration steps have been taken since the last
   * Initialize() call?
   **/
  int64_t get_num_steps_taken() const { return num_steps_taken_; }

  /** Return the step size the simulator would like to take next, based
   * primarily on the integrator's accuracy prediction (variable step
   * integrators; will change as the simulation progresses) or using the fixed
   * step for fixed step integrators.
   **/
  virtual const T& get_ideal_next_step_size() const {
    return ideal_next_step_size_;
  }

  /** Returns a const reference to the internally-maintained Context holding
   * the most recent step in the trajectory. This is suitable for publishing or
   * extracting information about this trajectory step.
   **/
  const Context<T>& get_context() const { return *context_; }

  /** Returns a mutable pointer to the internally-maintained Context holding
   * the most recent step in the trajectory. This is suitable for use in
   * updates, sampling operations, event handlers, and constraint projection.
   * You can also modify this prior to calling Initialize() to set initial
   * conditions. **/
  Context<T>* get_mutable_context() { return context_; }

  /** Replace the pointer to the internally-maintained Context with a different
   * one. This is useful for supplying a new set of initial conditions. You
   * should invoke Initialize() after replacing the Context.
   **/
  void reset_context(Context<T>* context) {
    context_ = context;
    initialization_done_ = false;
  }

  // Gets a reference to the system.
  const System<T>& get_system() const { return system_; }

  // Has the integrator been initialized?
  bool is_initialized() const { return initialization_done_; }

 protected:
  // Sets the ideal next step size.
  void set_ideal_next_step_size(const T& dt) { ideal_next_step_size_ = dt; }

  // Sets the actual initial step size taken.
  void set_actual_initial_step_size_taken(const T& dt) {
    actual_initial_step_size_taken_ = dt; }

  // Sets the smallest step size taken
  void set_smallest_step_size_taken(const T& dt) {
    smallest_step_size_taken_ = dt; }

  // Sets the largest step size taken
  void set_largest_step_size_taken(const T& dt) {
    largest_step_size_taken_ = dt;
  }

  // Sets the number of steps taken.
  void set_num_steps_taken(int64_t steps) { num_steps_taken_ = steps; }

  // Updates the integrator statistics
  void UpdateStatistics(const T& dt) {
    // handle first step specially
    if (++num_steps_taken_ == 1) {
      set_actual_initial_step_size_taken(dt);
      set_smallest_step_size_taken(dt);
      set_largest_step_size_taken(dt);
    } else {
      if (dt < get_smallest_step_size_taken())
        set_smallest_step_size_taken(dt);
      if (dt > get_largest_step_size_taken())
        set_largest_step_size_taken(dt);
    }
  }


 private:
  // Reference to the system being simulated.
  const System<T>& system_;

  // Pointer to the context
  Context<T>* context_{nullptr};  // The trajectory Context.

  // Runtime variables.
  // For variable step integrators, this is set at the end of each step to guide
  // the next one.
  T ideal_next_step_size_{nan()};  // indicates that the value is uninitialized

  // TODO(edrumwri): update to T?
  // the accuracy being used
  double accuracy_in_use_;

  // The maximum step size
  T max_step_size_{(T) std::numeric_limits<double>::infinity()};

  // The minimum step size
  T min_step_size_{(T) 0.0};

  // Statistics.
  T actual_initial_step_size_taken_{nan()};
  T smallest_step_size_taken_{nan()};
  T largest_step_size_taken_{nan()};
  int64_t num_steps_taken_{0};

  // Variable for indicating when an integrator has been initialized
  bool initialization_done_{false};

  // This a workaround for an apparent bug in clang 3.8 in which
  // defining this as a static constexpr member kNaN failed to instantiate
  // properly for the AutoDiffXd instantiation (worked in gcc and MSVC).
  // Restore to sanity when some later clang is current.
  static constexpr double nan() {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double target_accuracy_{0.0};      // means "unspecified, use default"
  T req_initial_step_size_{(T)0.0};  // means "unspecified, use default"
};                                   // IntegratorBase
}  // namespace systems
}  // namespace drake
