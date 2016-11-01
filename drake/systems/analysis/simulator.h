#pragma once

#include <limits>
#include <tuple>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/text_logging.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/** A forward dynamics solver for hybrid dynamic systems represented by
 * `System<T>` objects. Starting with an initial Context for a given System,
 * %Simulator advances time and produces a series of Context values that forms a
 * trajectory satisfying the system's dynamic equations to a specified accuracy.
 * Only the Context is modified by a %Simulator; the System is const.
 *
 * A Drake System is a continuous/discrete/hybrid dynamic system where the
 * continuous part is a DAE, that is, it is expected to consist of a set of
 * differential equations and bilateral algebraic constraints. The set of active
 * constraints may change as a result of particular events, such as contact.
 *
 * Given a current Context, we expect a System to provide us with
 * - derivatives for the continuous differential equations that already satisfy
 * the differentiated form of the constraints (typically, acceleration
 * constraints),
 * - a projection method for least-squares correction of violated higher-level
 * constraints (position and velocity level),
 * - a time-of-next-update method that can be used to adjust the integrator
 * step size in preparation for a discrete update,
 * - a method that can update discrete variables when their update time is
 * reached,
 * - witness (guard) functions for event isolation,
 * - event handlers (reset functions) for making appropriate changes to state
 * and mode variables when an event has been isolated.
 *
 * The continuous parts of the trajectory are advanced using a numerical
 * integrator. Different integrators have different properties; if you know
 * about that you can choose the one that is most appropriate for your
 * application. Otherwise, a default is provided which is adequate for most
 * systems.
 *
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 *
 * Instantiated templates for the following kinds of T's are provided and
 * available to link against in libdrakeSystemAnalysis:
 * - double
 * - AutoDiffXd
 *
 * Other instantiations are permitted but take longer to compile.
 */
// TODO(sherm1) When API stabilizes, should list the methods above in addition
// to describing them.
template <typename T>
class Simulator {
 public:
  /** Create a %Simulator that can advance a given System through time to
   * produce a trajectory consisting of a sequence of Context values. The System
   * must not have unresolved input ports if the values of those ports are
   * necessary for computations performed during simulation (see class
   * documentation).
   *
   * The Simulator holds an internal, non-owned reference to the System
   * object so you must ensure that `system` has a longer lifetime than the
   * %Simulator. It also owns a compatible Context internally that takes on each
   * of the trajectory values. You may optionally provide a Context that will be
   * used as the initial condition for the simulation; otherwise the %Simulator
   * will obtain a default Context from `system`.
   */
  explicit Simulator(const System<T>& system,
                     std::unique_ptr<Context<T>> context = nullptr);

  /** Prepares the %Simulator for a simulation. If the initial Context does not
   * satisfy the System's constraints, an attempt is made to modify the values
   * of the continuous state variables to satisfy the constraints. This method
   * will throw `std::logic_error` if the combination of options doesn't make
   * sense, and `std::runtime_error` if it is unable to find a
   * constraint-satisfying initial condition. */
  void Initialize();

  // TODO(edrumwri): add ability to account for final time
  /** Advance the System's trajectory until `boundary_time` is reached in
   * the context or some
   * other termination condition occurs. A variety of `std::runtime_error`
   * conditions are possible here, as well as error conditions that may be
   * thrown by the System when it is asked to perform computations. Be sure to
   * enclose your simulation in a `try-catch` block and display the
   * `what()` message.
   *
   * We recommend that you call `Initialize()` prior to making the first call to
   * `StepTo()`. However, if you don't it will be called for you the first
   * time you attempt a step, possibly resulting in unexpected error conditions.
   * See documentation for `Initialize()` for the error conditions it might
   * produce.
   */
  void StepTo(const T& boundary_time);

  /** Returns a const reference to the internally-maintained Context holding the
   * most recent step in the trajectory. This is suitable for publishing or
   * extracting information about this trajectory step.
   */
  const Context<T>& get_context() const { return *context_; }

  /** Returns a mutable pointer to the internally-maintained Context holding the
   * most recent step in the trajectory. This is suitable for use in updates,
   * sampling operations, event handlers, and constraint projection. You can
   * also modify this prior to calling Initialize() to set initial conditions.
   */
  Context<T>* get_mutable_context() { return context_.get(); }

  /** Replace the internally-maintained Context with a different one. The
   * current Context is deleted. This is useful for supplying a new set of
   * initial conditions. You should invoke Initialize() after replacing the
   * Context.
   * @param context The new context, which may be null. If the context is
   *                null, a new context must be set before attempting to step
   *                the system forward.
   */
  void reset_context(std::unique_ptr<Context<T>> context) {
    context_ = std::move(context);
    integrator_->reset_context(context_.get());
    initialization_done_ = false;
  }

  /** Transfer ownership of this %Simulator's internal Context to the caller.
   * The %Simulator will no longer contain a Context. The caller must not
   * attempt to advance the simulator in time after that point.
   * @sa reset_context()
   */
  std::unique_ptr<Context<T>> release_context() {
    integrator_->reset_context(nullptr);
    initialization_done_ = false;
    return std::move(context_);
  }

  /** Forget accumulated statistics. Statistics are reset to the values they
   * have post construction or immediately after `Initialize()`.
   */
  void ResetStatistics() {
    integrator_->ResetStatistics();
    num_steps_taken_ = 0;
    num_updates_ = 0;
    num_publishes_ = 0;
  }

  /**
   * Gets the number of publishes made since the last Initialize() or
   * ResetStatistics() call.
   */
  int64_t get_num_publishes() const { return num_publishes_; }

  /** Gets the number of integration steps since the last Initialize() call. */
  int64_t get_num_steps_taken() const { return num_steps_taken_; }

  /** Gets the number of difference equation updates performed since the last
  Initialize() call? */
  int64_t get_num_updates() const { return num_updates_; }

  /**
   *   Gets a pointer to the integrator used to advance the continuous aspects
   *   of the system.
   */
  const IntegratorBase<T>* get_integrator() const { return integrator_.get(); }

  /**
   *   Gets a pointer to the mutable integrator used to advance the continuous
   *   aspects of the system.
   */
  IntegratorBase<T>* get_mutable_integrator() { return integrator_.get(); }

  /**
   *   Resets the integrator with a new one. An example usage is:
   *   simulator.reset_integrator<ExplicitEulerIntegrator<double>>(sys, context,
   *   DT). The integrator must be initialized (via
   *   IntegratorBase::Initialize() function)
   *   before being used. The simulator will call that function automatically
   *   in its own Initialize() function.
   */
  template <class U, typename... Args>
  U* reset_integrator(Args&&... args) {
    initialization_done_ = false;
    integrator_ = std::make_unique<U>(std::forward<Args>(args)...);
    return static_cast<U*>(integrator_.get());
  }

 private:
  Simulator(const Simulator& s) = delete;
  Simulator& operator=(const Simulator& s) = delete;

  // Return a proposed end time for this step, and whether we picked that time
  // because we hit the next update time.
  static std::pair<T, bool> ProposeStepEndTime(const T& step_start_time,
                                               const T& ideal_step_size,
                                               const T& next_update_time,
                                               const T& final_time);

  // A pointer to the integrator.
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

  // The number of updates since the last call to Initialize().
  int64_t num_updates_{0};

  // The number of publishes since the last call to Initialize().
  int64_t num_publishes_{0};

  // The number of integration steps since the last call to Initialize().
  int64_t num_steps_taken_{0};

  // Set by Initialize() and reset by various traumas.
  bool initialization_done_{false};

  // Pre-allocated temporaries for updated difference states.
  std::unique_ptr<DifferenceState<T>> discrete_updates_;
};

template <typename T>
Simulator<T>::Simulator(const System<T>& system,
                        std::unique_ptr<Context<T>> context)
    : system_(system), context_(std::move(context)) {
  // TODO(edrumwri): remove default step size
  const double DT = 1e-3;

  // create a context if necessary
  if (!context_) context_ = system_.CreateDefaultContext();

  // create a default integrator and initialize it.
  integrator_ = std::unique_ptr<IntegratorBase<T>>(
      new RungeKutta2Integrator<T>(system_, DT, context_.get()));
  integrator_->Initialize();

  discrete_updates_ = system_.AllocateDifferenceVariables();
}

template <typename T>
void Simulator<T>::Initialize() {
  // TODO(sherm1) Modify Context to satisfy constraints.
  // TODO(sherm1) Invoke System's initial conditions computation.

  // Initialize the integrator.
  integrator_->Initialize();

  // Do a publish before the simulation starts.
  system_.Publish(*context_);

  // Restore default values.
  ResetStatistics();

  // Initialize runtime variables.
  initialization_done_ = true;
}

/**
 * Steps the simulation to the specified time.
 * The simulation loop is as follows:
 * 1. Perform necessary difference variable updates.
 * 2. Publish.
 * 3. Integrate the smooth system (the ODE or DAE)
 * 4. Perform post-step stabilization for DAEs (if desired).
 * @param boundary_time The time to advance the context to.
 */
template <typename T>
void Simulator<T>::StepTo(const T& boundary_time) {
  if (!initialization_done_) Initialize();

  DRAKE_THROW_UNLESS(boundary_time >= context_->get_time());

  // Updates/publishes can be triggered throughout the integration process,
  // but are not active at the start of the step.
  bool update_hit = false;
  bool publish_hit = false;

  // Integrate until desired interval has completed.
  UpdateActions<T> update_actions;
  while (context_->get_time() <= boundary_time) {
    // Starting a new step on the trajectory.
    const T step_start_time = context_->get_time();
    SPDLOG_TRACE(log(), "Starting a simulation step at {}", step_start_time);

    // First take any necessary discrete actions.
    if (update_hit) {
      for (const DiscreteEvent<T>& event : update_actions.events) {
        switch (event.action) {
          case DiscreteEvent<T>::kPublishAction: {
            system_.Publish(*context_, event);
            break;
          }
          case DiscreteEvent<T>::kUpdateAction: {
            DifferenceState<T>* xd = context_->get_mutable_difference_state();
            // Systems with discrete update events must have difference state.
            DRAKE_DEMAND(xd != nullptr);
            // First, compute the discrete updates into a temporary buffer.
            system_.EvalDifferenceUpdates(*context_, event,
                                          discrete_updates_.get());
            // Then, write them back into the context.
            xd->CopyFrom(*discrete_updates_);
            break;
          }
          default: {
            DRAKE_ABORT_MSG("Unknown DiscreteEvent action.");
            break;
          }
        }
      }
      ++num_updates_;
    }

    // Allow System a chance to produce some output.
    if (publish_hit) system_.Publish(*context_);

    // Remove old events
    update_actions.events.clear();

    // How far can we go before we have to take a sampling break?
    const T next_update_time =
        system_.CalcNextUpdateTime(*context_, &update_actions);
    DRAKE_ASSERT(next_update_time >= step_start_time);
    const T next_update_dt = next_update_time - step_start_time;

    // TODO(edrumwri): Get the next publish time when API available.
    T next_publish_dt = std::numeric_limits<double>::infinity();
    T next_publish_time = step_start_time + next_publish_dt;

    // Attempt to integrate.
    typename IntegratorBase<T>::StepResult result =
        integrator_->Step(next_publish_dt, next_update_dt);
    switch (result) {
      case IntegratorBase<T>::kReachedUpdateTime:
        update_hit = true;

        // Check whether update time effectively identical to publish time.
        publish_hit = (context_->get_time() >= next_publish_time);
        break;

      case IntegratorBase<T>::kReachedPublishTime:
        update_hit = false;
        publish_hit = true;
        break;

      case IntegratorBase<T>::kTimeHasAdvanced:
        update_hit = false;
        // TODO(edrumwri): Check if not publishing after every step, then
        //                 turn this off if that is the case.
        publish_hit = true;
        break;

      default:
        DRAKE_ABORT_MSG("Unexpected integrator result.");
    }
    ++num_steps_taken_;

    // TODO(sherm1) Constraint projection goes here.
  }

  // publish at the end of the step
  system_.Publish(*context_);
}

// TODO(edrumwri): Prepare to remove
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

  // The step may be limited or stretched either by final time or update
  // time, whichever comes sooner.
  bool update_time_hit = false;
  if (next_update_time <= final_time) {
    if (next_update_time <= step_stretch_time) {
      step_end_time = next_update_time;
      update_time_hit = true;
    }
  } else {  // boundary_time < next_update_time.
    if (final_time <= step_stretch_time) step_end_time = final_time;
  }

  return std::make_pair(step_end_time, update_time_hit);
}

}  // namespace systems
}  // namespace drake
