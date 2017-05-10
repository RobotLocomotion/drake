#pragma once

#include <algorithm>
#include <chrono>
#include <limits>
#include <list>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/text_logging.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/witness_function.h"

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
 * available to link against in the containing library:
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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Simulator)

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

  /** Slow the simulation down to *approximately* synchronize with real time
   * when it would otherwise run too fast. Normally the %Simulator takes steps
   * as quickly as it can. You can request that it slow down to synchronize with
   * real time by providing a realtime rate greater than zero here.
   *
   * @warning No guarantees can be made about how accurately the simulation
   * can be made to track real time, even if computation is fast enough. That's
   * because the system utilities used to implement this do not themselves
   * provide such guarantees. So this is likely to work nicely for visualization
   * purposes where human perception is the only concern. For any other uses
   * you should consider whether approximate real time is adequate for your
   * purposes.
   *
   * @note If the full-speed simulation is already slower than real time you
   * can't speed it up with this call! Instead consider requesting less
   * integration accuracy, using a faster integration method or fixed time
   * step, or using a simpler model.
   *
   * @param realtime_rate
   *   Desired rate relative to real time. Set to 1 to track real time, 2 to
   *   run twice as fast as real time, 0.5 for half speed, etc. Zero or
   *   negative restores the rate to its default of 0, meaning the simulation
   *   will proceed as fast as possible.
   */
  // TODO(sherm1): Provide options for issuing a warning or aborting the
  // simulation if the desired rate cannot be achieved.
  void set_target_realtime_rate(double realtime_rate) {
    target_realtime_rate_ = std::max(realtime_rate, 0.);
  }

  /** Return the real time rate target currently in effect. The default is
   * zero, meaning the %Simulator runs as fast as possible. You can change the
   * target with set_target_realtime_rate().
   */
  double get_target_realtime_rate() const {
    return target_realtime_rate_;
  }

  /** Return the rate that simulated time has progressed relative to real time.
   * A return of 1 means the simulation just matched real
   * time, 2 means the simulation was twice as fast as real time, 0.5 means
   * it was running in 2X slow motion, etc.
   *
   * The value returned here is calculated as follows: <pre>
   *
   *          simulated_time_now - initial_simulated_time
   *   rate = -------------------------------------------
   *                realtime_now - initial_realtime
   * </pre>
   * The `initial` times are recorded when Initialize() or ResetStatistics()
   * is called. The returned rate is undefined if Initialize() has not yet
   * been called.
   *
   * @returns The rate achieved since the last Initialize() or ResetStatistics()
   *          call.
   *
   * @see set_target_realtime_rate()
   */
  double get_actual_realtime_rate() const;

  /** Sets whether the simulation should invoke Publish on the System under
   * simulation during every time step. If enabled, Publish will be invoked
   * after discrete updates and before continuous integration. Regardless of
   * whether publishing every time step is enabled, Publish will be invoked at
   * Simulator initialize time, and as System<T>::CalcNextUpdateTime requests.
   */
  void set_publish_every_time_step(bool publish) {
    publish_every_time_step_ = publish;
  }

  /** Sets whether the simulation should invoke Publish in Initialize().
   */
  void set_publish_at_initialization(bool publish) {
    publish_at_initialization_ = publish;
  }

  /** Returns true if the simulation should invoke Publish on the System under
   * simulation every time step.  By default, returns true.
   */
  // TODO(sherm1, edrumwri): Consider making this false by default.
  bool get_publish_every_time_step() const { return publish_every_time_step_; }

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
  void ResetStatistics();

  /**
   * Gets the number of publishes made since the last Initialize() or
   * ResetStatistics() call.
   */
  int64_t get_num_publishes() const { return num_publishes_; }

  /** Gets the number of integration steps since the last Initialize() call. */
  int64_t get_num_steps_taken() const { return num_steps_taken_; }

  /** Gets the number of discrete variable updates performed since the last
  Initialize() call. */
  int64_t get_num_discrete_updates() const { return num_discrete_updates_; }

  /** Gets the number of "unrestricted" updates performed since the last
  Initialize() call. */
  int64_t get_num_unrestricted_updates() const {
    return num_unrestricted_updates_; }

  /** Gets a pointer to the integrator used to advance the continuous aspects
   *  of the system.
   */
  const IntegratorBase<T>* get_integrator() const { return integrator_.get(); }

  /** Gets a pointer to the mutable integrator used to advance the continuous
   * aspects of the system.
   */
  IntegratorBase<T>* get_mutable_integrator() { return integrator_.get(); }

  /**
   * Resets the integrator with a new one. An example usage is:
   * @code
   * simulator.reset_integrator<ExplicitEulerIntegrator<double>>
   *               (sys, context, DT).
   * @endcode
   * The %Simulator must be reinitialized after resetting the integrator to
   * ensure the integrator is properly initialized. You can do that explicitly
   * with the Initialize() method or it will be done implicitly at the first
   * time step.
   */
  template <class U, typename... Args>
  U* reset_integrator(Args&&... args) {
    initialization_done_ = false;
    integrator_ = std::make_unique<U>(std::forward<Args>(args)...);
    return static_cast<U*>(integrator_.get());
  }



  /**
   * Gets a constant reference to the system.
   * @note a mutable reference is not available.
   */
  const System<T>& get_system() const { return system_; }

 private:
  void HandleUnrestrictedUpdate(const std::vector<DiscreteEvent<T>>& events);
  void HandleDiscreteUpdate(const std::vector<DiscreteEvent<T>>& events);
  void HandlePublish(const std::vector<DiscreteEvent<T>>& events);

  bool IntegrateContinuousState(const T& next_publish_dt,
      const T& next_update_dt,
      const T& next_sample_time,
      const T& boundary_dt,
      UpdateActions<T>* update_actions);
  std::list<WitnessFunction<T>*> IsolateWitnessTriggers(
      const std::vector<WitnessFunction<T>*>& witnesses, const VectorX<T>& w0,
      const T& t0, const VectorX<T>& x0, const T& tf);

  // The steady_clock is immune to system clock changes so increases
  // monotonically. We'll work in fractional seconds.
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::duration<double>;
  using TimePoint = std::chrono::time_point<Clock, Duration>;

  // If the simulated time in the context is ahead of real time, pause long
  // enough to let real time catch up (approximately).
  void PauseIfTooFast() const;

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

  // Slow down to this rate if possible (user settable).
  double target_realtime_rate_{0.};

  bool publish_every_time_step_{true};

  bool publish_at_initialization_{true};

  // These are recorded at initialization or statistics reset.
  double initial_simtime_{nan()};  // Simulated time at start of period.
  TimePoint initial_realtime_;     // Real time at start of period.

  // The number of discrete updates since the last call to Initialize().
  int64_t num_discrete_updates_{0};

  // The number of unrestricted updates since the last call to Initialize().
  int64_t num_unrestricted_updates_{0};

  // The number of publishes since the last call to Initialize().
  int64_t num_publishes_{0};

  // The number of integration steps since the last call to Initialize().
  int64_t num_steps_taken_{0};

  // Set by Initialize() and reset by various traumas.
  bool initialization_done_{false};

  // Per step events that need to be handled. This is set by Initialize().
  std::vector<DiscreteEvent<T>> per_step_actions_;

  // Pre-allocated temporaries for updated discrete states.
  std::unique_ptr<DiscreteValues<T>> discrete_updates_;

  // Pre-allocated temporaries for states from unrestricted updates.
  std::unique_ptr<State<T>> unrestricted_updates_;
};

template <typename T>
Simulator<T>::Simulator(const System<T>& system,
                        std::unique_ptr<Context<T>> context)
    : system_(system), context_(std::move(context)) {
  // Setup defaults that should be generally reasonable.
  const double dt = 1e-3;

  // Create a context if necessary.
  if (!context_) context_ = system_.CreateDefaultContext();

  // @TODO(edrumwri): Make variable step integrator default.
  // Create a default integrator and initialize it.
  integrator_ = std::unique_ptr<IntegratorBase<T>>(
      new RungeKutta2Integrator<T>(system_, dt, context_.get()));
  integrator_->Initialize();

  // Allocate the necessary temporaries for storing state in update calls
  // (which will then be transferred back to system state).
  discrete_updates_ = system_.AllocateDiscreteVariables();
  unrestricted_updates_ = context_->CloneState();
}

template <typename T>
void Simulator<T>::Initialize() {
  // TODO(sherm1) Modify Context to satisfy constraints.
  // TODO(sherm1) Invoke System's initial conditions computation.

  // Initialize the integrator.
  integrator_->Initialize();

  // Gets all the events that need handling.
  system_.GetPerStepEvents(*context_, &per_step_actions_);

  // Restore default values.
  ResetStatistics();

  // TODO(siyuan): transfer publish entirely to individual systems.
  // Do a publish before the simulation starts.
  if (publish_at_initialization_) {
    system_.Publish(*context_);
    ++num_publishes_;
  }

  // Initialize runtime variables.
  initialization_done_ = true;
}

template <typename T>
void Simulator<T>::HandleUnrestrictedUpdate(
    const std::vector<DiscreteEvent<T>>& events) {
  for (const DiscreteEvent<T>& event : events) {
    if (event.action == DiscreteEvent<T>::kUnrestrictedUpdateAction) {
      State<T>* x = context_->get_mutable_state();
      DRAKE_DEMAND(x != nullptr);
      // First, compute the unrestricted updates into a temporary buffer.
      system_.CalcUnrestrictedUpdate(*context_, event,
          unrestricted_updates_.get());
      // TODO(edrumwri): simply swap the states for additional speed.
      // Now write the update back into the context.
      x->CopyFrom(*unrestricted_updates_);
      ++num_unrestricted_updates_;
    } else if (event.action == DiscreteEvent<T>::kUnknownAction) {
      throw std::logic_error("kUnknownAction encountered.");
    }
  }
}

template <typename T>
void Simulator<T>::HandleDiscreteUpdate(
    const std::vector<DiscreteEvent<T>>& events) {
  for (const DiscreteEvent<T>& event : events) {
    if (event.action == DiscreteEvent<T>::kDiscreteUpdateAction) {
      DiscreteValues<T>* xd = context_->get_mutable_discrete_state();
      // Systems with discrete update events must have discrete state.
      DRAKE_DEMAND(xd != nullptr);
      // First, compute the discrete updates into a temporary buffer.
      system_.CalcDiscreteVariableUpdates(*context_, event,
          discrete_updates_.get());
      // Then, write them back into the context.
      xd->CopyFrom(*discrete_updates_);
      ++num_discrete_updates_;
    } else if (event.action == DiscreteEvent<T>::kUnknownAction) {
      throw std::logic_error("kUnknownAction encountered.");
    }
  }
}

template <typename T>
void Simulator<T>::HandlePublish(
    const std::vector<DiscreteEvent<T>>& events) {
  for (const DiscreteEvent<T>& event : events) {
    if (event.action == DiscreteEvent<T>::kPublishAction) {
      system_.Publish(*context_, event);
      ++num_publishes_;
    } else if (event.action == DiscreteEvent<T>::kUnknownAction) {
      throw std::logic_error("kUnknownAction encountered.");
    }
  }
}

/**
 * Steps the simulation to the specified time.
 * The simulation loop is as follows:
 * 1. Perform necessary discrete variable updates.
 * 2. Publish.
 * 3. Integrate the smooth system (the ODE or DAE)
 * 4. Perform post-step stabilization for DAEs (if desired).
 * @param boundary_time The time to advance the context to.
 * @pre The simulation state is valid  (i.e., no discrete updates or state
 *      projections are necessary) at the present time.
 */
template <typename T>
void Simulator<T>::StepTo(const T& boundary_time) {
  if (!initialization_done_) Initialize();

  DRAKE_THROW_UNLESS(boundary_time >= context_->get_time());

  // Updates/publishes can be triggered throughout the integration process,
  // but are not active at the start of the step.
  bool sample_time_hit = false;

  // Integrate until desired interval has completed.
  UpdateActions<T> update_actions;

  while (context_->get_time() < boundary_time || sample_time_hit) {
    // Starting a new step on the trajectory.
    const T step_start_time = context_->get_time();
    SPDLOG_TRACE(log(), "Starting a simulation step at {}", step_start_time);

    // Delay to match target realtime rate if requested and possible.
    PauseIfTooFast();

    // The general policy here is to do actions in decreasing order of
    // "violence" to the state, i.e. unrestricted -> discrete -> publish.
    // The "timed" actions happen before the "per step" ones.

    // Do unrestricted updates first.
    if (sample_time_hit)
      HandleUnrestrictedUpdate(update_actions.events);
    HandleUnrestrictedUpdate(per_step_actions_);

    // Do restricted (discrete variable) updates next.
    if (sample_time_hit)
      HandleDiscreteUpdate(update_actions.events);
    HandleDiscreteUpdate(per_step_actions_);

    // Do any publishes last.
    if (sample_time_hit)
      HandlePublish(update_actions.events);
    HandlePublish(per_step_actions_);

    // TODO(siyuan): transfer per step publish entirely to individual systems.
    // Allow System a chance to produce some output.
    if (get_publish_every_time_step()) {
      system_.Publish(*context_);
      ++num_publishes_;
    }

    // How far can we go before we have to take a sampling break?
    const T next_sample_time =
        system_.CalcNextUpdateTime(*context_, &update_actions);
    DRAKE_DEMAND(next_sample_time >= step_start_time);

    // Determine whether the DiscreteEvent requested by the System at
    // next_sample_time includes an Update action, a Publish action, or both.
    T next_update_dt = std::numeric_limits<double>::infinity();
    T next_publish_dt = std::numeric_limits<double>::infinity();
    for (const DiscreteEvent<T>& event : update_actions.events) {
      if (event.action == DiscreteEvent<T>::kDiscreteUpdateAction ||
          event.action == DiscreteEvent<T>::kUnrestrictedUpdateAction) {
        next_update_dt = next_sample_time - step_start_time;
      }
      if (event.action == DiscreteEvent<T>::kPublishAction) {
        next_publish_dt = next_sample_time - step_start_time;
      }
    }

    // Get the dt that gets to the boundary time.
    const T boundary_dt = boundary_time - step_start_time;

    // Integrate the continuous state forward in time.
    sample_time_hit = IntegrateContinuousState(next_publish_dt,
                                               next_update_dt,
                                               next_sample_time,
                                               boundary_dt,
                                               &update_actions);

    // Update the number of simulation steps taken.
    ++num_steps_taken_;

    // TODO(sherm1) Constraint projection goes here.
  }
}

// Isolates the time at which a witness function triggered, to the desired
// tolerance.
// @post The context will be isolated to the first witness function trigger,
//       to within the isolation tolerance specified by each witness function.
template <class T>
std::list<WitnessFunction<T>*> Simulator<T>::IsolateWitnessTriggers(
    const std::vector<WitnessFunction<T>*>& witnesses, const VectorX<T>& w0,
    const T& t0, const VectorX<T>& x0, const T& tf) {
  using std::max;

  // TODO(edrumwri): Speed this process using interpolation between states.
  // TODO(edrumwri): Speed this process using more powerful root finding
  //                 methods.

  // Get the tolerance for floating point equivalence.
  const T zero_tol = max(tf, T(1)) * std::numeric_limits<double>::epsilon();

  // Get the system.
  const System<T>& system = get_system();

  // Will need to alter the context repeatedly.
  Context<T>* context = get_mutable_context();

  // Mini function for integrating the system forward in time.
  std::function<void(const T&)> fwd_int =
      [t0, x0, zero_tol, context, this](const T& t_des) {
    const T inf = std::numeric_limits<double>::infinity();
    context->set_time(t0);
    context->get_mutable_continuous_state()->SetFromVector(x0);
    T t_remaining = t_des - t0;
    while (t_remaining > zero_tol) {
      integrator_->StepOnceAtMost(inf, inf, t_remaining);
      t_remaining = t_des - context->get_time();
    }
  };

  // Set the first witness function trigger and the witness function that
  // makes that trigger.
  std::list<WitnessFunction<T>*> triggered_witnesses;
  T t_first = tf;

  // Loop over all witness functions.
  for (size_t i = 0; i < witnesses.size(); ++i) {
    // Set a and b
    T a = t0;
    T b = t_first;

    // Set the witness function values.
    T fa = w0[i];

    // Integrate to b.
    fwd_int(t_first);

    // Evaluate the witness function.
    T fb = system.EvalWitnessFunction(*context, witnesses[i]);

    // See whether there has been a sign change; after shrinking the time
    // interval one or more times, there may no longer be one, in which case
    // we can skip the bisection for this interval.
    if (!witnesses[i]->should_trigger(fa, fb))
      continue;

    // Get the dead bands.
    const T pos_dead = witnesses[i]->get_positive_dead_band();
    const T neg_dead = witnesses[i]->get_negative_dead_band();

    while (true) {
      // Determine the midpoint.
      T c = (a + b) / 2;

      // Restore the state and time to t0, then integrate to c.
      fwd_int(c);

      // Evaluate the witness function.
      T fc = system.EvalWitnessFunction(*context, witnesses[i]);

      // Bisect.
      if (witnesses[i]->should_trigger(fa, fc)) {
        b = c;
        fb = fc;
      } else {
        a = c;
        fa = fc;
      }

      // If the time is sufficiently isolated and fc is within the dead band,
      // quit. This conjunction on the condition helps find a zero well into
      // the interior of the dead band.
      if (b - c < witnesses[i]->get_time_isolation_tolerance() &&
          fc < pos_dead && fc > neg_dead) {
        T t_trigger = witnesses[i]->get_trigger_time(std::make_pair(a, fa),
                                                     std::make_pair(b, fb));

        // Only clear the list of witnesses if t_trigger strictly less than
        // t_first.
        DRAKE_DEMAND(t_first >= t_trigger);
        if (t_trigger < t_first)
          triggered_witnesses.clear();

        // Set the first trigger time and add the witness index.
        triggered_witnesses.push_back(witnesses[i]);
        t_first = t_trigger;
        break;
      }
    }
  }

  DRAKE_DEMAND(t_first < tf);
  return triggered_witnesses;
}

// Integrates the continuous state forward in time while attempting to locate
// the first zero of any triggered witness functions.
// @returns `true` if integration terminated on a sample time, indicating that
//          an event needs to be handled at the state/time on return.
template <class T>
bool Simulator<T>::IntegrateContinuousState(const T& next_publish_dt,
                                            const T& next_update_dt,
                                            const T& next_sample_time,
                                            const T& boundary_dt,
                                            UpdateActions<T>* update_actions) {
  // Save the time and current state.
  const Context<T>& context = get_context();
  const T t0 = context.get_time();
  const VectorX<T> x0 = context.get_continuous_state()->CopyToVector();

  // Get the set of witness functions active at the current time.
  const System<T>& system = get_system();
  const std::vector<WitnessFunction<T>*> witness_functions = system.
      get_witness_functions(context);

  // Evaluate the witness functions.
  VectorX<T> w0(witness_functions.size());
  for (size_t i = 0; i < witness_functions.size(); ++i)
      w0[i] = system.EvalWitnessFunction(context, witness_functions[i]);

  // Attempt to integrate. Updates and boundary times are consciously
  // distinguished between. See internal documentation for
  // IntegratorBase::StepOnceAtMost() for more information.
  typename IntegratorBase<T>::StepResult result =
      integrator_->StepOnceAtMost(next_publish_dt, next_update_dt,
                                  boundary_dt);
  const T tf = context.get_time();

  // Evaluate the witness functions again.
  VectorX<T> wf(witness_functions.size());
  for (size_t i =0; i < witness_functions.size(); ++i)
    wf[i] = system.EvalWitnessFunction(context, witness_functions[i]);

  // See whether a witness function triggered.
  bool witness_triggered = false;
  for (size_t i =0; i < witness_functions.size() && !witness_triggered; ++i) {
      if (witness_functions[i]->should_trigger(w0[i], wf[i]))
        witness_triggered = true;
  }

  // Triggering requires isolating the witness function time.
  if (witness_triggered) {
    // Isolate the time that the witness function.
    std::list<WitnessFunction<T>*> triggered_witnesses = IsolateWitnessTriggers(
      witness_functions, w0, t0, x0, tf);

    // TODO(edrumwri): Store witness function(s) that triggered.
    for (WitnessFunction<T>* fn : witness_functions) {
      SPDLOG_DEBUG(drake::log(), "Witness function {} crossed zero at time {}",
                   fn->get_name(), context.get_time());
      update_actions->time = context.get_time();
      update_actions->events.push_back(DiscreteEvent<T>());
      DiscreteEvent<T>& event = update_actions->events.back();
      event.action = fn->get_action_type();
      event.triggered_witness_function = fn;
    }

    // Indicate a "sample time was hit". In more understandable terms, this
    // means that an event should be handled on the next simulation loop.
    return true;
  } else {
    // No witness function triggered; handle integration as usual.
    // Updates and boundary times are consciously distinguished between. See
    // internal documentation for IntegratorBase::StepOnceAtMost() for more
    // information.
    switch (result) {
      case IntegratorBase<T>::kReachedUpdateTime:
      case IntegratorBase<T>::kReachedPublishTime:
        // Next line sets the time to the exact sample time rather than
        // introducing rounding error by summing the context time + dt.
        context_->set_time(next_sample_time);
        return true;            // Sample time hit.
        break;

      case IntegratorBase<T>::kTimeHasAdvanced:
      case IntegratorBase<T>::kReachedBoundaryTime:
        return false;           // Did not hit a sample time.
        break;

      default:DRAKE_ABORT_MSG("Unexpected integrator result.");
    }
    ++num_steps_taken_;

    // TODO(sherm1) Constraint projection goes here.
  }
}

}  // namespace systems
}  // namespace drake
