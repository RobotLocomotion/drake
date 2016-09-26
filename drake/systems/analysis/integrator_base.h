//
// Created by drum on 9/14/16.
//

#ifndef DRAKE_SUPERBUILD_INTEGRATOR_H
#define DRAKE_SUPERBUILD_INTEGRATOR_H

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/vector_base.h"
#include "drake/systems/vector.h"

namespace drake {
namespace systems {

// An abstract class for an integrator for ODEs and DAEs
/**
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 */
template <class T>
class IntegratorBase {
 public:
  // Status returned by Step()
  /**
   When a step is successful, it will return an indication of what caused it
   to stop where it did. When unsuccessful it will throw an exception so you
   won't see any return value. When return of control is due ONLY to reaching a
   report time, (status is ReachedReportTime) the context may return an
   interpolated value at an earlier time.

   TODO(edrumwri): comments below in preparation of upcoming functionality
   Note: we ensure algorithmically that no report time, scheduled time, or
   final time t can occur *within* an event window, that is, we will never have
   t_low < t < t_high for any interesting t. Further, t_report, t_scheduled and
   t_final can coincide with t_high but only t_report can be at t_low. The
   interior of t_low:t_high is a "no man's land" where we don't understand the
   solution, so must be avoided.

   Note: the simulation step must always end at an update time but can end
   after a publish time
   */
  enum StepResult {
    /**
     * The implication is that no discrete update is necessary (which may be
     * because System::CalcNextUpdateTime(.) gives a time far into the future or
     * because witness functions indicate no upcoming event.
     */
    kReachedReportTime = 1,
    /** localized an event; this is the *before* state (interpolated) **/
    kReachedZeroCrossing = 2,
    kReachedScheduledEvent = 3,
    /**
     * user requested control whenever an internal step is successful;
     * TODO(edrumwri): possibly take this out (if integrator should take as big
     * a step as possible until publish or update occurs)
     */
    kTimeHasAdvanced = 4,
    /** TODO(edrumwri): possibly take this out, took maximum number of steps
        without finishing integrating over the interval **/
    kReachedStepLimit = 5,
    /** TODO(edrumwri): possibly remove this after implementing variable step
     * integration **/
    kStartOfContinuousInterval = 7,
  };

  /**
   * @param system a reference to the system to be integrated
   * @param context a pointer to a writeable context. The integrator will
   *                advance the system state using this context. The integrator
   *                must not outlive the context.
   * @return
   */
  explicit IntegratorBase(const System<T>& system,
                          Context<T>* context = nullptr)
      : system_(system), context_(context) {
    initialization_done_ = false;
  }

  /** Request that the integrator attempt to achieve a particular accuracy for
  the continuous portions of the simulation. Otherwise a default accuracy is
  chosen for you. This is ignored for fixed-step integration since accuracy
  control requires variable step sizes.

  Integrators vary in the range of accuracy (loosest to tightest) that they can
  support. If you request accuracy outside the supported range for the chosen
  integrator it will be quietly adjusted to be in range. You can find out the
  accuracy setting actually being used using `get_accuracy_in_use()`.

  The precise meaning of *accuracy* is a complicated discussion, but translates
  roughly to the number of significant digits you want in the results. By
  convention it is supplied as `10^-digits`, meaning that an accuracy of 1e-3
  provides about three significant digits. For more information, see <pre>
  Sherman, et al. Procedia IUTAM 2:241-261 (2011), section 3.3.
  http://dx.doi.org/10.1016/j.piutam.2011.04.023
  </pre>
  **/
  virtual void set_target_accuracy(double accuracy) {
    target_accuracy_ = accuracy;
  }

  /**
   *   Gets the target accuracy.
   */
  virtual double get_target_accuracy() const { return target_accuracy_; }

  /**
   *   Integrator must be initialized before being used.
   */
  virtual void Initialize() { initialization_done_ = true; }

  /** Request that the first attempted integration step have a particular size.
Otherwise the integrator will estimate a suitable size for the initial step
attempt. For fixed-step integration, all steps will be taken at this step
size. For variable-step integration this will be treated as a maximum size
subject to accuracy requirements and event occurrences. You can find out what
size *actually* worked with `get_actual_initial_step_size_taken()`. **/
  // Note that this function is virtual b/c so that
  // fixed step integrators exhibit the user-expected behavior without having to
  // keep track of two variable settings (step size and requested initial step
  // size)
  virtual void request_initial_step_size_target(const T& step_size) {
    req_initial_step_size_ = step_size;
  }

  /** Get the first integration step target size.
Otherwise the integrator will estimate a suitable size for the initial step
attempt. For fixed-step integration, all steps will be taken at this step
size. For variable-step integration this will be treated as a maximum size
subject to accuracy requirements and event occurrences. You can find out what
size *actually* worked with `get_actual_initial_step_size_taken()`. **/
  virtual const T& get_initial_step_size_target() const {
    return req_initial_step_size_;
  }

  /** Advance the System's trajectory by at most dt. Initialize() must
   * be called before making any calls to Step(), or an exception will
   * be thrown.
   **/
  virtual StepResult Step(const T& dt) = 0;

  /** @name                       Statistics
These methods track relevant activity of the %Simulator since the last call
to `Initialize()`. **/
  /**@{**/
  /** Forget accumulated statistics. These are reset to the values they have
  post construction or immediately after `Initialize()`. **/
  void ResetStatistics() {
    actual_initial_step_size_taken_ = nan();
    smallest_step_size_taken_ = nan();
    largest_step_size_taken_ = nan();
    num_steps_taken_ = 0;
    num_samples_taken_ = 0;
  }

  /** What what the actual size of the successful first step? **/
  const T& get_actual_initial_step_size_taken() const {
    return actual_initial_step_size_taken_;
  }

  /** What was the size of the smallest step taken since the last Initialize()
  call? **/
  const T& get_smallest_step_size_taken() const {
    return smallest_step_size_taken_;
  }

  /** What was the size of the largest step taken since the last Initialize()
  call? **/
  const T& get_largest_step_size_taken() const {
    return largest_step_size_taken_;
  }

  /** How many integration steps have been taken since the last Initialize()
  call? **/
  int64_t get_num_steps_taken() const { return num_steps_taken_; }

  /** How many discrete sample events have been processed since the last
  Initialize() call? **/
  int64_t get_num_samples_taken() const { return num_samples_taken_; }
  /**@}**/

  /** Return the step size the simulator would like to take next, based
  primarily on the integrator's accuracy prediction (variable step integrators;
  will change as the simulation
   progresses) or using the fixed step for fixed step integrators. **/
  virtual const T& get_ideal_next_step_size() const {
    return ideal_next_step_size_;
  }

  /** Returns a const reference to the internally-maintained Context holding the
most recent step in the trajectory. This is suitable for publishing or
extracting information about this trajectory step. **/
  const Context<T>& get_context() const { return *context_; }

  /** Returns a mutable pointer to the internally-maintained Context holding the
  most recent step in the trajectory. This is suitable for use in updates,
  sampling operations, event handlers, and constraint projection. You can
  also modify this prior to calling Initialize() to set initial conditions. **/
  Context<T>* get_mutable_context() { return context_; }

  /** Replace the internally-maintained Context with a different one. The
  current Context is deleted. This is useful for supplying a new set of initial
  conditions. You should invoke Initialize() after replacing the Context. **/
  void reset_context(Context<T>* context) {
    context_ = context;
    initialization_done_ = false;
  }

 protected:
  // Reference to the system being simulated
  const System<T>& system_;

  // Pointer to the context
  Context<T>* context_;  // The trajectory Context.

  // Runtime variables.
  // For variable step integrators, this is set at the end of each step to guide
  // the next one.
  T ideal_next_step_size_{nan()};  // indicates that the value is uninitialized

  // Statistics.
  T actual_initial_step_size_taken_{nan()};
  T smallest_step_size_taken_{nan()};
  T largest_step_size_taken_{nan()};
  int64_t num_steps_taken_{0};
  int64_t num_samples_taken_{0};

  // Variable for indicating when an integrator has been initialized
  bool initialization_done_{false};

  // TODO(edrumwri): flesh this out later
  void EvaluateWitnessFunctions() {}

  // TODO(edrumwri): flesh this out later
  bool CheckWitnessFunctions() { throw std::runtime_error("Not implemented"); }

 private:
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
#endif  // DRAKE_SUPERBUILD_INTEGRATOR_H
