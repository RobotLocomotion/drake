#pragma once

#include <tuple>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/drakeSystemAnalysis_export.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/** Used to specify a particular choice of integration method.
Currently the default is 2nd order Runge Kutta (explicit trapezoid rule). **/

/** A forward dynamics solver for hybrid dynamic systems represented by
`System<T>` objects. Starting with an initial Context for a given System,
%Simulator advances time and produces a series of Context values that forms a
trajectory satisfying the system's dynamic equations to a specified accuracy.
Only the Context is modified by a %Simulator; the System is const.

A Drake System is a continuous/discrete/hybrid dynamic system where the
continuous part is a DAE, that is, it is expected to consist of a set of
differential equations and bilateral algebraic constraints. The set of active
constraints may change as a result of particular events, such as contact.

Given a current Context, we expect a System to provide us with
 - derivatives for the continuous differential equations that already satisfy
   the differentiated form of the constraints (typically, acceleration
   constraints),
 - a projection method for least-squares correction of violated higher-level
   constraints (position and velocity level),
 - a time-of-next-sample method that can be used to adjust the integrator
   step size in preparation for a discrete update,
 - a method that can update discrete variables when their sample time is
   reached,
 - witness (guard) functions for event isolation,
 - event handlers (reset functions) for making appropriate changes to state and
   mode variables when an event has been isolated.

The continuous parts of the trajectory are advanced using a numerical
integrator. Different integrators have different properties; if you know about
that you can choose the one that is most appropriate for your application.
Otherwise, a default is provided which is adequate for most systems.

@tparam T The vector element type, which must be a valid Eigen scalar.

Instantiated templates for the following kinds of T's are provided and
available to link against in libdrakeSystemAnalysis:
 - double
 - AutoDiffXd

 Other instantiations are permitted but take longer to compile. **/
// TODO(sherm1) When API stabilizes, should list the methods above in addition
// to describing them.
template <typename T>
class Simulator {
 public:
  /** Create a %Simulator that can advance a given System through time to
  produce a trajectory consisting of a sequence of Context values. The System
  must not have unresolved input ports if the values of those ports are
  necessary for computations performed during simulation (see class
  documentation).

  The Simulator holds an internal, non-owned reference to the System
  object so you must ensure that `system` has a longer lifetime than the
  %Simulator. It also owns a compatible Context internally that takes on each
  of the trajectory values. You may optionally provide a Context that will be
  used as the initial condition for the simulation; otherwise the %Simulator
  will obtain a default Context from `system`. **/
  explicit Simulator(const System<T>& system,
                     std::unique_ptr<Context<T>> context = nullptr);

  /** Prepares the %Simulator for a simulation. This requires determining the
  integrator type, processing the options requested by the caller, and choosing
  an initial step size to attempt. If the initial Context does not satisfy the
  System's constraints, an attempt is made to modify the values of the
  continuous state variables to satisfy the constraints. This method will throw
  `std::logic_error` if the combination of options doesn't make sense, and
  `std::runtime_error` if it is unable to find a constraint-satisfying
  initial condition. **/
  void Initialize();

  /** Advance the System's trajectory until `final_time` is reached or some
  other termination condition occurs. The System's `Publish()` method is called
  at the start of each step. A variety of `std::runtime_error` conditions are
  possible here, as well as error conditions that may be thrown by the System
  when it is asked to perform computations. Be sure to enclose your simulation
  in a `try-catch` block and display the `what()` message.

  We recommend that you call `Initialize()` prior to making the first call to
  `StepTo()`. However, if you don't it will be called for you the first
  time you attempt a step, possibly resulting in unexpected error conditions.
  See documentation for `Initialize()` for the error conditions it might
  produce. **/
  // TODO(sherm1) Publish() should be called at publishing sample times.
  void StepTo(const T& final_time);

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
  </pre> **/
  void set_accuracy(double accuracy) { integrator_->set_accuracy(accuracy); }

  /** Report the accuracy setting actually being used. **/
  double get_target_accuracy() const {
    return integrator_->get_target_accuracy();
  }

  /** Request that the first attempted integration step have a particular size.
  Otherwise the integrator will estimate a suitable size for the initial step
  attempt. For fixed-step integration, all steps will be taken at this step
  size. For variable-step integration this will be treated as a maximum size
  subject to accuracy requirements and event occurrences. You can find out what
  size *actually* worked with `get_actual_initial_step_size_taken()`. **/
  void request_initial_step_size_target(double step_size) {
    integrator_->request_initial_step_size_target(step_size);
  }

  /** Report the step size we will attempt for an initial step. **/
  const T& get_initial_step_size_target() const {
    return integrator_->get_initial_step_size_target();
  }
  /**@}**/

  /** Returns a const reference to the internally-maintained Context holding the
  most recent step in the trajectory. This is suitable for publishing or
  extracting information about this trajectory step. **/
  const Context<T>& get_context() const { return *context_; }

  /** Returns a mutable pointer to the internally-maintained Context holding the
  most recent step in the trajectory. This is suitable for use in updates,
  sampling operations, event handlers, and constraint projection. You can
  also modify this prior to calling Initialize() to set initial conditions. **/
  Context<T>* get_mutable_context() { return context_.get(); }

  /** Replace the internally-maintained Context with a different one. The
  current Context is deleted. This is useful for supplying a new set of initial
  conditions. You should invoke Initialize() after replacing the Context. **/
  void reset_context(std::unique_ptr<Context<T>> context) {
    context_ = std::move(context);
    if (integrator_) integrator_->reset_context(context_.get());
    initialization_done_ = false;
  }

  /** Transfer ownership of this %Simulator's internal Context to the caller.
  The %Simulator will no longer contain a Context. **/
  std::unique_ptr<Context<T>> release_context() {
    if (integrator_) integrator_->reset_context(NULL);
    initialization_done_ = false;
    return std::move(context_);
  }

  /** @name                       Statistics
  These methods track relevant activity of the %Simulator since the last call
  to `Initialize()`. **/
  /**@{**/
  /** Forget accumulated statistics. These are reset to the values they have
  post construction or immediately after `Initialize()`. **/
  void ResetStatistics() {
    integrator_->ResetStatistics();
    num_steps_taken_ = 0;
    num_updates_ = 0;
  }

  /** What what the actual size of the successful first step? **/
  T get_actual_initial_step_size_taken() const {
    return integrator_->get_actual_initial_step_size_taken();
  }

  /** What was the size of the smallest step taken since the last Initialize()
  call? **/
  T get_smallest_step_size_taken() const {
    return integrator_->get_smallest_step_size_taken();
  }

  /** What was the size of the largest step taken since the last Initialize()
  call? **/
  T get_largest_step_size_taken() const {
    return integrator_->get_largest_step_size_taken();
  }

  /** How many integration steps have been taken since the last Initialize()
  call? **/
  int64_t get_num_steps_taken() const { return num_steps_taken_; }

  /** How many discrete update events have been processed since the last
  Initialize() call? **/
  int64_t get_num_updates() const { return num_updates_; }
  /**@}**/

  /** Return the step size the simulator would like to take next, based
  primarily on the integrator's accuracy prediction. For variable
  step integrators this will change as the simulation progresses. **/
  T get_ideal_next_step_size() const {
    return integrator_->get_ideal_next_step_size();
  }

  /// Gets a pointer to the integrator
  IntegratorBase<T>* get_integrator() const { return integrator_.get(); }

  // TODO(edrumwri): undo initialization?
  /// Sets the integrator
  void reset_integrator(std::unique_ptr<IntegratorBase<T>>& integrator) {
    integrator_ = std::move(integrator);
  }

 private:
  Simulator(const Simulator& s) = delete;
  Simulator& operator=(const Simulator& s) = delete;

  // Return a proposed end time for this step, and whether we picked that time
  // because we hit the next sample time.
  static std::pair<T, bool> ProposeStepEndTime(const T& step_start_time,
                                               const T& ideal_step_size,
                                               const T& next_update_time,
                                               const T& final_time);

  // Put "in use" settings back to their default values and reset runtime
  // variables. These will be modified afterwards in accordance with caller's
  // requests.
  void ResetSimulatorSettingsInUse() {
    initialization_done_ = false;

    // TODO(edrumwri): create a new integrator when a variable step integrator
    // is available
    // integrator_ = std::unique_ptr<IntegratorBase<T>>(new
    // ExplicitEulerIntegrator<T>(system_, context_.get()));
    std::cerr << "ResetSimulatorSettingsInUse() should not be called until a "
                 "variable step integrator is "
              << "implemented" << std::endl;

    // TODO(edrumwri): reset integrator settings
  }

  // A pointer to the integrator
  std::unique_ptr<IntegratorBase<T>> integrator_;

  // TODO(sherm1) This a workaround for an apparent bug in clang 3.8 in which
  // defining this as a static constexpr member kNaN failed to instantiate
  // properly for the AutoDiffXd instantiation (worked in gcc and MSVC).
  // Restore to sanity when some later clang is current.
  static constexpr double nan() {
    return std::numeric_limits<double>::quiet_NaN();
  }

  static constexpr double kDefaultAccuracy = 1e-3;  // 1/10 of 1%.
  static constexpr double kDefaultInitialStepSizeAttempt = 1e-3;

  const System<T>& system_;              // Just a reference; not owned.
  std::unique_ptr<Context<T>> context_;  // The trajectory Context.

  int64_t num_updates_{
      0};  // the number of updates since the last call to Initialize()
  int64_t num_steps_taken_{0};  // the number of integration steps since the
                                // last call to Initialize()

  // Set by Initialize() and reset by various traumas.
  bool initialization_done_{false};
};

// No need for user code to instantiate these; they are in the library.

// TODO(sherm1) Clean this up with a more nuanced export macro.
#ifdef _MSC_VER
extern template class Simulator<double>;
extern template class Simulator<AutoDiffXd>;
#else
extern template class DRAKESYSTEMANALYSIS_EXPORT Simulator<double>;
extern template class DRAKESYSTEMANALYSIS_EXPORT Simulator<AutoDiffXd>;
#endif

template <typename T>
Simulator<T>::Simulator(const System<T>& system,
                        std::unique_ptr<Context<T>> context)
    : system_(system), context_(std::move(context)) {
  if (!context_) context_ = system_.CreateDefaultContext();
}

template <typename T>
void Simulator<T>::Initialize() {
  // TODO(sherm1) Modify Context to satisfy constraints.
  // TODO(sherm1) Invoke System's initial conditions computation.

  // create integrator if necessary
  if (!integrator_)
    // TODO(edrumwri): create a new integrator when a variable step integrator is
    // available
    //    integrator_ = std::unique_ptr<IntegratorBase<T>>(new
    //    ExplicitEulerIntegrator<T>(system_, context_.get()));
    throw std::runtime_error("No integrator set");

  // Restore default values.
  ResetStatistics();
  ResetSimulatorSettingsInUse();

  // Initialize runtime variables.
  initialization_done_ = true;
}

/**
 * The simulation loop is as follows:
 * 1. perform necessary discrete updates (incl. computing DAE constraints)
 * 2. publish
 * 3. integrate the smooth system (the ODE, or ODE part of the DAE)
 * 4. post-step stabilization for DAEs (if desired)
 * @param final_time
 */
template <typename T>
void Simulator<T>::StepTo(const T& final_time) {
  if (!initialization_done_) Initialize();

  DRAKE_THROW_UNLESS(final_time >= context_->get_time());

  // TODO(edrumwri):
  SampleActions sample_actions;
  bool update_time_hit = false;
  while (context_->get_time() <= final_time) {
    // Starting a new step on the trajectory.
    const T step_start_time = context_->get_time();

    // First make any necessary discrete updates.
    if (update_time_hit) {
      system_.Update(context_.get(), sample_actions);
      ++num_updates_;
    }

    // Allow System a chance to produce some output.
    // TODO(sherm1) This should be called only at Publish sample times.
    system_.Publish(*context_);

    // That may have been the final trajectory entry.
    if (step_start_time == final_time) break;

    // How far can we go before we have to take a sampling break?
    const T next_update_time =
        system_.CalcNextSampleTime(*context_, &sample_actions);
    DRAKE_ASSERT(next_update_time >= step_start_time);

    // Figure out the largest step we can reasonably take, and whether we had
    // to stop there due to hitting the next sample time.
    T step_end_time;
    std::tie(step_end_time, update_time_hit) = ProposeStepEndTime(
        step_start_time, integrator_->get_ideal_next_step_size(),
        next_update_time, final_time);

    // integrate here
    typename IntegratorBase<T>::StepResult result =
        integrator_->Step(step_end_time - step_start_time);
    if (result != result)
      std::cerr << "error!" << std::endl;
    else
      ++num_steps_taken_;

    // TODO(sherm1) Constraint projection goes here.
  }
}

template <typename T>
std::pair<T, bool> Simulator<T>::ProposeStepEndTime(const T& step_start_time,
                                                    const T& ideal_step_size,
                                                    const T& next_update_time,
                                                    const T& final_time) {
  static constexpr double kMaxStretch = 0.01;  // Allow 1% step size stretch.

  // Start with the ideal step size.
  T step_end_time = step_start_time + ideal_step_size;

  // We can be persuaded to take a slightly bigger step if necessary to
  // avoid a tiny sliver step before we have to do something discrete.
  const T step_stretch_time = step_end_time + kMaxStretch * ideal_step_size;

  // The step may be limited or stretched either by final time or sample
  // time, whichever comes sooner.
  bool update_time_hit = false;
  if (next_update_time <= final_time) {
    if (next_update_time <= step_stretch_time) {
      step_end_time = next_update_time;
      update_time_hit = true;
    }
  } else {  // final_time < next_update_time.
    if (final_time <= step_stretch_time) step_end_time = final_time;
  }

  return std::make_pair(step_end_time, update_time_hit);
}

}  // namespace systems
}  // namespace drake
