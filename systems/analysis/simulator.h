#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/extract_double.h"
#include "drake/common/text_logging.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace systems {

/**
 A class for advancing the state of hybrid dynamic systems, represented by
 `System<T>` objects, forward in time. Starting with an initial Context for a
 given System, %Simulator advances time and produces a series of Context
 values that forms a trajectory satisfying the system's dynamic equations to
 a specified accuracy. Only the Context is modified by a %Simulator; the
 System is const.

 A Drake System is a continuous/discrete/hybrid dynamic system where the
 continuous part is a DAE, that is, it is expected to consist of a set of
 differential equations and bilateral algebraic constraints. The set of
 active constraints may change as a result of particular events, such as
 contact.

 Given a current Context, we expect a System to provide us with
 - derivatives for the continuous differential equations that already satisfy
   the differentiated form of the constraints (typically, acceleration
   constraints),
 - a projection method for least-squares correction of violated higher-level
   constraints (position and velocity level),
 - a time-of-next-update method that can be used to adjust the integrator
   step size in preparation for a discrete update,
 - methods that can update discrete variables when their update time is
   reached,
 - witness (guard) functions for event isolation,
 - event handlers (reset functions) for making appropriate changes to state
   and mode variables when an event has been isolated.

The continuous parts of the trajectory are advanced using a numerical
integrator. Different integrators have different properties; you can choose
the one that is most appropriate for your application or use the default
which is adequate for most systems.

<h3>How the simulation is stepped: simulation mechanics for authors of discrete
and hybrid systems</h3>

This section is targeted toward users who have created a LeafSystem implementing
a discrete or hybrid system. For authors of such systems, it can be useful to
understand the simulation details in order to attain the desired state behavior
over time. This behavior is dependent on the ordering in which discrete events
and continuous updates are processed. (By "discrete events" we mean to include
any of Drake's event handlers.) The basic issues and terminology are
introduced in the @ref discrete_systems module; please look there first before
proceeding.

As pictured in @ref discrete_systems, when a continuous-time system has
discrete events, the state x can have two significant values at the event
time t. These are
 - x⁻(t), the value of x _before_ the discrete update occurs (○ markers), and
 - x⁺(t), the value of x _after_ the discrete update occurs (● markers).

Thus the value of the Context, which contains both time and state, advances
from {t, x⁻(t)} to {t, x⁺(t)} as a result of the update. While those Context
values are user-visible, the details of stepping here require an intermediate
value which we'll denote {t, x*(t)}.

Recall that Drake's state x is partitioned into continuous, discrete, and
abstract partitions xc, xd, and xa, so `x = { xc, xd, xa }`. Within a single
step, these are updated in three stages:
  -# Unrestricted update (can change x)
  -# Discrete update (can change only xd)
  -# Continuous update (changes t and xc)

Where needed, we extend the above notation to xc⁻, xa⁺, etc. to indicate the
value of an individual partition at a particular stage of the stepping
algorithm.

The following pseudocode uses the above notation to describe the algorithm
"Advance()" that the %Simulator uses to incrementally advance the system
trajectory (time t and state x). We refer to such an advancement as a "substep"
to make it clear that we are not talking about a StepTo() call, which may cause
many substeps to occur. (We'll define StepTo() in terms of Advance() below.) In
general, the length of a substep is not known a priori and is determined by
the Advance() algorithm. Each substep consists of zero or more unrestricted
updates, followed by zero or more discrete updates, followed by (possibly
zero-length) continuous time and state advancement, followed by zero or more
publishes.

The pseudocode will clarify the effects on time and state of each of the update
stages above. This algorithm is given a starting Context value `{tₛ, x⁻(tₛ)}`
and returns an end Context value `{tₑ, x⁻(tₑ)}`, where tₑ is _no later_ than a
given tₘₐₓ.
```
// Advance the trajectory (time and state) from start value {tₛ, x⁻(tₛ)} to an
// end value {tₑ, x⁻(tₑ)}, where tₛ ≤ tₑ ≤ tₘₐₓ.
procedure Advance(tₛ, x⁻(tₛ), tₘₐₓ)

  // Update any variables (no restrictions).
  x*(tₛ) ← DoAnyUnrestrictedUpdates(tₛ, x⁻(tₛ))

  // ----------------------------------
  // Time and state are at {tₛ, x*(tₛ)}
  // ----------------------------------

  // Update discrete variables.
  xd⁺(tₛ) ← DoAnyDiscreteUpdates(tₛ, x*(tₛ))

  xc⁺(tₛ) ← xc*(tₛ)  // These values carry over from x*(tₛ).
  xa⁺(tₛ) ← xa*(tₛ)

  // ----------------------------------
  // Time and state are at {tₛ, x⁺(tₛ)}
  // ----------------------------------

  // See how far it is safe to integrate without missing any events.
  tₑᵥₑₙₜ ← CalcNextEventTime(tₛ, x⁺(tₛ))

  // Integrate continuous variables forward in time. Integration may terminate
  // before reaching tₛₜₒₚ due to witnessed events.
  tₛₜₒₚ ← min(tₑᵥₑₙₜ, tₘₐₓ)
  tₑ, xc⁻(tₑ) ← Integrate(tₛ, x⁺(tₛ), tₛₜₒₚ)

  xd⁻(tₑ) ← xd⁺(tₛ)  // Discrete values are held from x⁺(tₛ).
  xa⁻(tₑ) ← xa⁺(tₛ)

  // ----------------------------------
  // Time and state are at {tₑ, x⁻(tₑ)}
  // ----------------------------------

  DoAnyPublishes(tₑ, x⁻(tₑ))

  return {tₑ, x⁻(tₑ)}
```

We can use the notation and pseudocode to flesh out the StepTo() and
Initialize() functions:
```
// Advance the simulation until time tₘₐₓ.
procedure StepTo(tₘₐₓ)
  t ← current_time
  while t < tₘₐₓ
    {tₑ, x⁻(tₑ)} ← Advance(t, x⁻(t), tₘₐₓ)
    {t, x⁻(t)} ← {tₑ, x⁻(tₑ)}
  endwhile

// Update time and state to {t₀, x⁻(t₀)}, which is the starting value of the
// trajectory, and thus the value the Context should contain at the start of the
// first simulation substep.
procedure Initialize(t₀, x₀)
  x⁺(t₀) ← DoAnyUpdates as in Advance()
  x⁻(t₀) ← x⁺(t₀)  // No continuous update needed.

  // ----------------------------------
  // Time and state are at {t₀, x⁻(t₀)}
  // ----------------------------------

  DoAnyPublishes(t₀, x⁻(t₀))
```
Initialize() can be viewed as a "0ᵗʰ substep" that occurs before the first
Advance() substep as described above. Like Advance(), Initialize() first
performs pending updates (in this case only initialization events can be
"pending"). Time doesn't advance so there is no continuous update phase and
witnesses cannot trigger. Finally, again like Advance(), the initial trajectory
point `{t₀, x⁻(t₀)}` is provided to the handlers for any triggered publish
events. That includes initialization publish events, per-step publish events,
and periodic or timed publish events that trigger at t₀.

@tparam T The vector element type, which must be a valid Eigen scalar.

Instantiated templates for the following kinds of T's are provided and
available to link against in the containing library:
 - double
 - AutoDiffXd

Other instantiations are permitted but take longer to compile.
*/

// TODO(sherm1) When API stabilizes, should list the methods above in addition
// to describing them.
template <typename T>
class Simulator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Simulator)

  /// Create a %Simulator that can advance a given System through time to
  /// produce a trajectory consisting of a sequence of Context values. The
  /// System must not have unresolved input ports if the values of those ports
  /// are necessary for computations performed during simulation (see class
  /// documentation).
  ///
  /// The Simulator holds an internal, non-owned reference to the System
  /// object so you must ensure that `system` has a longer lifetime than the
  /// %Simulator. It also owns a compatible Context internally that takes on
  /// each of the trajectory values. You may optionally provide a Context that
  /// will be used as the initial condition for the simulation; otherwise the
  /// %Simulator will obtain a default Context from `system`.
  explicit Simulator(const System<T>& system,
                     std::unique_ptr<Context<T>> context = nullptr);

  /// Create a %Simulator which additionally maintains ownership of the System.
  ///
  /// @exclude_from_pydrake_mkdoc{The prior overload's docstring is better, and
  /// we only need one of the two -- overloading on ownership doesn't make
  /// sense for pydrake.}
  Simulator(std::unique_ptr<const System<T>> system,
            std::unique_ptr<Context<T>> context = nullptr);

  /// Prepares the %Simulator for a simulation. In order, the sequence of
  /// actions taken here are:
  /// - The active integrator's Initialize() method is invoked.
  /// - Statistics are reset.
  /// - Initialization update events are triggered and handled to produce the
  ///   initial trajectory value `{t₀, x(t₀)}`.
  /// - Then that initial value is provided to the handlers for any publish
  ///   events that have triggered, including initialization and per-step
  ///   publish events, and periodic or other time-triggered publish events
  ///   that are scheduled for the initial time t₀.
  ///
  /// See the class documentation for more information. We recommend calling
  /// Initialize() explicitly prior to beginning a simulation so that error
  /// conditions will be discovered early. However, Initialize() will be called
  /// automatically by the first StepTo() call if it hasn't already been called.
  ///
  /// @note If you make a change to the Context or to Simulator options between
  /// StepTo() calls you should consider whether to call Initialize() before
  /// resuming; StepTo() will not do that automatically for you. Whether to do
  /// so depends on whether you want the above initialization operations
  /// performed. For example, if you changed the time you will likely want the
  /// time-triggered events to be recalculated in case one is due at the new
  /// starting time.
  ///
  /// @warning Initialize() does not automatically attempt to satisfy System
  /// constraints -- it is up to you to make sure that constraints are
  /// satisifed by the initial conditions.
  ///
  /// This method will throw `std::logic_error` if the combination of options
  /// doesn't make sense. Other failures are possible from the System and
  /// integrator in use.
  // TODO(sherm1) Make Initialize() attempt to satisfy constraints.
  // TODO(sherm1) Add a ReInitialize() or Resume() method that is called
  //              automatically by StepTo() if the Context has changed.
  void Initialize();

  /// Advances the System's trajectory until `boundary_time` is reached in
  /// the context or some other termination condition occurs. A variety of
  /// `std::runtime_error` conditions are possible here, as well as error
  /// conditions that may be thrown by the System when it is asked to perform
  /// computations. Be sure to enclose your simulation in a `try-catch` block
  /// and display the `what()` message.
  ///
  /// We recommend that you call Initialize() prior to making the first call
  /// to StepTo(). However, if you don't it will be called for you the first
  /// time that you attempt a step, possibly resulting in unexpected error
  /// conditions. See documentation for `Initialize()` for the error conditions
  /// it might produce.
  ///
  /// @warning You should consider calling Initialize() if you alter the
  /// the Context or Simulator options between successive StepTo() calls. See
  /// Initialize() for more information.
  ///
  /// @param boundary_time The time to advance the context to.
  /// @pre The internal Context satisfies all System constraints or will after
  ///      pending Context updates are performed.
  void StepTo(const T& boundary_time);

  /// Slow the simulation down to *approximately* synchronize with real time
  /// when it would otherwise run too fast. Normally the %Simulator takes steps
  /// as quickly as it can. You can request that it slow down to synchronize
  /// with real time by providing a realtime rate greater than zero here.
  ///
  /// @warning No guarantees can be made about how accurately the simulation
  /// can be made to track real time, even if computation is fast enough. That's
  /// because the system utilities used to implement this do not themselves
  /// provide such guarantees. So this is likely to work nicely for
  /// visualization purposes where human perception is the only concern. For any
  /// other uses you should consider whether approximate real time is adequate
  /// for your purposes.
  ///
  /// @note If the full-speed simulation is already slower than real time you
  /// can't speed it up with this call! Instead consider requesting less
  /// integration accuracy, using a faster integration method or fixed time
  /// step, or using a simpler model.
  ///
  /// @param realtime_rate
  ///   Desired rate relative to real time. Set to 1 to track real time, 2 to
  ///   run twice as fast as real time, 0.5 for half speed, etc. Zero or
  ///   negative restores the rate to its default of 0, meaning the simulation
  ///   will proceed as fast as possible.
  // TODO(sherm1): Provide options for issuing a warning or aborting the
  // simulation if the desired rate cannot be achieved.
  void set_target_realtime_rate(double realtime_rate) {
    target_realtime_rate_ = std::max(realtime_rate, 0.);
  }

  /// Return the real time rate target currently in effect. The default is
  /// zero, meaning the %Simulator runs as fast as possible. You can change the
  /// target with set_target_realtime_rate().
  double get_target_realtime_rate() const {
    return target_realtime_rate_;
  }

  /// Return the rate that simulated time has progressed relative to real time.
  /// A return of 1 means the simulation just matched real
  /// time, 2 means the simulation was twice as fast as real time, 0.5 means
  /// it was running in 2X slow motion, etc.
  ///
  /// The value returned here is calculated as follows: <pre>
  ///
  ///          simulated_time_now - initial_simulated_time
  ///   rate = -------------------------------------------
  ///                realtime_now - initial_realtime
  /// </pre>
  /// The `initial` times are recorded when Initialize() or ResetStatistics()
  /// is called. The returned rate is undefined if Initialize() has not yet
  /// been called.
  ///
  /// @returns The rate achieved since the last Initialize() or
  ///           ResetStatistics() call.
  ///
  /// @see set_target_realtime_rate()
  double get_actual_realtime_rate() const;

  /// Sets whether the simulation should invoke Publish on the System under
  /// simulation during every time step. If enabled, Publish will be invoked
  /// after discrete updates and before continuous integration. Regardless of
  /// whether publishing every time step is enabled, Publish will be invoked at
  /// Simulator initialize time, and as System<T>::CalcNextUpdateTime requests.
  void set_publish_every_time_step(bool publish) {
    publish_every_time_step_ = publish;
  }

  /// Sets whether the simulation should invoke Publish in Initialize().
  void set_publish_at_initialization(bool publish) {
    publish_at_initialization_ = publish;
  }

  /// Returns true if the simulation should invoke Publish on the System under
  /// simulation every time step.  By default, returns true.
  // TODO(sherm1, edrumwri): Consider making this false by default.
  bool get_publish_every_time_step() const { return publish_every_time_step_; }

  /// Returns a const reference to the internally-maintained Context holding the
  /// most recent step in the trajectory. This is suitable for publishing or
  /// extracting information about this trajectory step. Do not call this method
  /// if there is no Context.
  const Context<T>& get_context() const {
    DRAKE_ASSERT(context_ != nullptr);
    return *context_;
  }

  /// Returns a mutable reference to the internally-maintained Context holding
  /// the most recent step in the trajectory. This is suitable for use in
  /// updates, sampling operations, event handlers, and constraint projection.
  /// You can also modify this prior to calling Initialize() to set initial
  /// conditions. Do not call this method if there is no Context.
  Context<T>& get_mutable_context()  {
    DRAKE_ASSERT(context_ != nullptr);
    return *context_;
  }

  /// Returns `true` if this Simulator has an internally-maintained Context.
  /// This is always true unless `reset_context()` has been called.
  bool has_context() const { return context_ != nullptr; }

  /// Replace the internally-maintained Context with a different one. The
  /// current Context is deleted. This is useful for supplying a new set of
  /// initial conditions. You should invoke Initialize() after replacing the
  /// Context.
  /// @param context The new context, which may be null. If the context is
  ///                null, a new context must be set before attempting to step
  ///                the system forward.
  void reset_context(std::unique_ptr<Context<T>> context) {
    context_ = std::move(context);
    integrator_->reset_context(context_.get());
    initialization_done_ = false;
  }

  /// Transfer ownership of this %Simulator's internal Context to the caller.
  /// The %Simulator will no longer contain a Context. The caller must not
  /// attempt to advance the simulator in time after that point.
  /// @sa reset_context()
  std::unique_ptr<Context<T>> release_context() {
    integrator_->reset_context(nullptr);
    initialization_done_ = false;
    return std::move(context_);
  }

  /// Forget accumulated statistics. Statistics are reset to the values they
  /// have post construction or immediately after `Initialize()`.
  void ResetStatistics();

  /// Gets the number of publishes made since the last Initialize() or
  /// ResetStatistics() call.
  int64_t get_num_publishes() const { return num_publishes_; }

  /// Gets the number of substeps since the last Initialize() call. (We're
  /// not counting the Initialize() 0-length "substep".)
  int64_t get_num_steps_taken() const { return num_substeps_taken_; }

  /// Gets the number of discrete variable updates performed since the last
  /// Initialize() call.
  int64_t get_num_discrete_updates() const { return num_discrete_updates_; }

  /// Gets the number of "unrestricted" updates performed since the last
  /// Initialize() call.
  int64_t get_num_unrestricted_updates() const {
    return num_unrestricted_updates_; }

  /// Gets a pointer to the integrator used to advance the continuous aspects
  /// of the system.
  const IntegratorBase<T>* get_integrator() const { return integrator_.get(); }

  /// Gets a pointer to the mutable integrator used to advance the continuous
  /// aspects of the system.
  IntegratorBase<T>* get_mutable_integrator() { return integrator_.get(); }

  /// Resets the integrator with a new one. An example usage is:
  /// @code
  /// simulator.reset_integrator(std::move(integrator));
  /// @endcode
  /// The %Simulator must be reinitialized after resetting the integrator to
  /// ensure the integrator is properly initialized. You can do that explicitly
  /// with the Initialize() method or it will be done implicitly at the first
  /// time step.
  template <class U>
  U* reset_integrator(std::unique_ptr<U> integrator) {
    initialization_done_ = false;
    integrator_ = std::move(integrator);
    return static_cast<U*>(integrator_.get());
  }

  /// Resets the integrator with a new one using factory construction. An
  /// example usage is:
  /// @code
  /// simulator.reset_integrator<ExplicitEulerIntegrator<double>>
  ///               (sys, DT, context).
  /// @endcode
  /// See the base overload for `reset_integrator` for more details.
  template <class U, typename... Args>
  U* reset_integrator(Args&&... args) {
    auto integrator = std::make_unique<U>(std::forward<Args>(args)...);
    return reset_integrator(std::move(integrator));
  }

  /// Gets the length of the interval used for witness function time isolation.
  /// The length of the interval is computed differently, depending on context,
  /// to support multiple applications, as described below:
  ///
  /// * **Simulations using error controlled integrators**: the isolation time
  ///   interval will be scaled by the product of the system's characteristic
  ///   time and the accuracy stored in the Context.
  /// * **Simulations using integrators taking fixed steps**: the isolation time
  ///   interval will be determined differently depending on whether the
  ///   accuracy is set in the Context or not. If the accuracy *is* set in the
  ///   Context, the nominally fixed steps for integrating continuous state will
  ///   be subdivided until events have been isolated to the requisite interval
  ///   length, which is scaled by the step size times the accuracy in the
  ///   Context. If accuracy is not set in the Context, event isolation will
  ///   not be performed.
  ///
  /// The isolation window length will never be smaller than the integrator's
  /// working minimum tolerance (see
  /// IntegratorBase::get_working_minimum_step_size());
  ///
  /// @returns the isolation window if the Simulator should be isolating
  ///          witness-triggered events in time, or returns empty otherwise
  ///          (indicating that any witness-triggered events should trigger
  ///          at the end of a time interval over which continuous state is
  ///          integrated).
  /// @throws std::logic_error if the accuracy is not set in the Context and
  ///         the integrator is not operating in fixed step mode (see
  ///         IntegratorBase::get_fixed_step_mode().
  optional<T> GetCurrentWitnessTimeIsolation() const;

  /// Gets a constant reference to the system.
  /// @note a mutable reference is not available.
  const System<T>& get_system() const { return system_; }

 private:
  // All constructors delegate to here.
  Simulator(const System<T>* system,
            std::unique_ptr<const System<T>> owned_system,
            std::unique_ptr<Context<T>> context);

  void HandleUnrestrictedUpdate(
      const EventCollection<UnrestrictedUpdateEvent<T>>& events);

  void HandleDiscreteUpdate(
      const EventCollection<DiscreteUpdateEvent<T>>& events);

  void HandlePublish(const EventCollection<PublishEvent<T>>& events);

  bool IntegrateContinuousState(const T& next_publish_dt,
                                const T& next_update_dt,
                                const T& time_of_next_timed_event,
                                const T& boundary_dt,
                                CompositeEventCollection<T>* events);

  void IsolateWitnessTriggers(
      const std::vector<const WitnessFunction<T>*>& witnesses,
      const VectorX<T>& w0,
      const T& t0, const VectorX<T>& x0, const T& tf,
      std::vector<const WitnessFunction<T>*>* triggered_witnesses);

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

  // Do not use this.  This is valid iff the constructor is passed a
  // unique_ptr (allowing the Simulator to maintain ownership).  Use the
  // system_ variable instead, which is valid always.
  const std::unique_ptr<const System<T>> owned_system_;

  const System<T>& system_;              // Just a reference; not owned.
  std::unique_ptr<Context<T>> context_;  // The trajectory Context.

  // Temporaries used for witness function isolation.
  std::vector<const WitnessFunction<T>*> triggered_witnesses_;
  VectorX<T> w0_, wf_;

  // Slow down to this rate if possible (user settable).
  double target_realtime_rate_{0.};

  bool publish_every_time_step_{true};

  bool publish_at_initialization_{true};

  // These are recorded at initialization or statistics reset.
  double initial_simtime_{nan()};  // Simulated time at start of period.
  TimePoint initial_realtime_;     // Real time at start of period.

  // The number of discrete updates since the last statistics reset.
  int64_t num_discrete_updates_{0};

  // The number of unrestricted updates since the last statistics reset.
  int64_t num_unrestricted_updates_{0};

  // The number of publishes since the last statistics reset.
  int64_t num_publishes_{0};

  // The number of integration substeps since the last statistics reset.
  int64_t num_substeps_taken_{0};

  // Set by Initialize() and reset by various traumas.
  bool initialization_done_{false};

  // The vector of active witness functions.
  std::unique_ptr<std::vector<const WitnessFunction<T>*>> witness_functions_;

  // Indicator for whether the Simulator needs to redetermine the active witness
  // functions.
  bool redetermine_active_witnesses_{true};

  // Per step events that are to be handled on every "major time step" (i.e.,
  // every successful completion of a substep). This collection is constructed
  // within Initialize().
  std::unique_ptr<CompositeEventCollection<T>> per_step_events_;

  // Timed events can be triggered either at a particular time (like an alarm)
  // or periodically. This collection is constructed within Initialize().
  std::unique_ptr<CompositeEventCollection<T>> timed_events_;

  // Witnessed events are triggered as a witness function crosses zero during
  // StepTo(). This collection is constructed within Initialize().
  std::unique_ptr<CompositeEventCollection<T>> witnessed_events_;

  // Indicates when a timed or witnessed event needs to be handled on the next
  // call to StepTo().
  bool timed_or_witnessed_event_triggered_{false};

  // The time that the next timed event is to be handled. This value is set in
  // both Initialize() and StepTo().
  T next_timed_event_time_{std::numeric_limits<double>::quiet_NaN()};

  // Pre-allocated temporaries for updated discrete states.
  std::unique_ptr<DiscreteValues<T>> discrete_updates_;

  // Pre-allocated temporaries for states from unrestricted updates.
  std::unique_ptr<State<T>> unrestricted_updates_;

  // Pre-allocated temporary for ContinuousState passed to event handlers after
  // witness function triggering.
  std::unique_ptr<ContinuousState<T>> event_handler_xc_;

  // Mapping of witness functions to pre-allocated events.
  std::unordered_map<const WitnessFunction<T>*, std::unique_ptr<Event<T>>>
      witness_function_events_;
};

template <typename T>
Simulator<T>::Simulator(const System<T>& system,
                        std::unique_ptr<Context<T>> context)
    : Simulator(&system, nullptr, std::move(context)) {}

template <typename T>
Simulator<T>::Simulator(std::unique_ptr<const System<T>> owned_system,
                        std::unique_ptr<Context<T>> context) :
    Simulator(nullptr, std::move(owned_system), std::move(context)) {}

template <typename T>
Simulator<T>::Simulator(const System<T>* system,
                        std::unique_ptr<const System<T>> owned_system,
                        std::unique_ptr<Context<T>> context)
    : owned_system_(std::move(owned_system)),
      system_(owned_system_ ? *owned_system_ : *system),
      context_(std::move(context)) {
  // Setup defaults that should be generally reasonable.
  const double max_step_size = 0.1;
  const double initial_step_size = 1e-4;
  const double default_accuracy = 1e-4;

  // Create a context if necessary.
  if (!context_) context_ = system_.CreateDefaultContext();

  // Create a default integrator and initialize it.
  integrator_ = std::unique_ptr<IntegratorBase<T>>(
      new RungeKutta3Integrator<T>(system_, context_.get()));
  integrator_->request_initial_step_size_target(initial_step_size);
  integrator_->set_maximum_step_size(max_step_size);
  integrator_->set_target_accuracy(default_accuracy);
  integrator_->Initialize();

  // Allocate the necessary temporaries for storing state in update calls
  // (which will then be transferred back to system state).
  discrete_updates_ = system_.AllocateDiscreteVariables();
  unrestricted_updates_ = context_->CloneState();

  // Allocate the vector of active witness functions.
  witness_functions_ = std::make_unique<
      std::vector<const WitnessFunction<T>*>>();

  // Allocate the necessary temporary for witness-based event handling.
  event_handler_xc_ = system_.AllocateTimeDerivatives();
}

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {
// This function computes the previous (i.e., that which is one step closer to
// negative infinity) *non-denormalized* (i.e., either zero or normalized)
// floating-point number from `value`. nexttoward() provides very similar
// functionality except for its proclivity for producing denormalized numbers
// that typically only result from arithmetic underflow and are hence dangerous
// to use in further floating point operations. Thus,
// GetPreviousNormalizedValue() acts like nexttoward(value, -inf) but without
// producing denormalized numbers.
// @param value a floating point value that is not infinity or NaN. Denormalized
//        inputs are treated as zero to attain consistent behavior regardless
//        of the setting of the FPU "treat denormalized numbers as zero" mode
//        (which can be activated through linking with shared libraries that
//        use gcc's -ffast-math option).
template <class T>
T GetPreviousNormalizedValue(const T& value) {
  using std::nexttoward;
  using std::abs;

  // There are three distinct cases to be handled:
  //     -∞        -10⁻³⁰⁸  0      10⁻³⁰⁸      ∞
  //     |-----------|------|------|----------|
  // (a) ^           ^              ^         ^   [-∞, 10⁻³⁰⁸] ∪ (10³⁰⁸, ∞]
  // (b)              ^           ^               (-10⁻³⁰⁸, 10⁻³⁰⁸)
  // (c)                           ^              10⁻³⁰⁸

  // Treat denormalized numbers as zero. This code is designed to produce the
  // same outputs for `value` regardless of the setting of the FPU's DAZ ("treat
  // denormalized numbers as zero") mode.
  const double min_normalized = std::numeric_limits<double>::min();
  const T& value_mod = (abs(value) < min_normalized) ? 0.0 : value;

  // Treat zero (b) and DBL_MIN (c) specially, since nexttoward(value, -inf)
  // returns denormalized numbers for these two values.
  if (value_mod == 0.0)
    return -std::numeric_limits<double>::min();
  if (value_mod == min_normalized)
    return 0.0;

  // Case (a) uses nexttoward(.).
  const long double inf = std::numeric_limits<long double>::infinity();
  const double prev_value = nexttoward(value, -inf);
  DRAKE_DEMAND(
      std::fpclassify(ExtractDoubleOrThrow(prev_value)) == FP_NORMAL ||
      std::fpclassify(ExtractDoubleOrThrow(prev_value)) == FP_ZERO);
  return prev_value;
}
}  // namespace internal
#endif

template <typename T>
void Simulator<T>::Initialize() {
  // TODO(sherm1) Modify Context to satisfy constraints.
  // TODO(sherm1) Invoke System's initial conditions computation.

  // Initialize the integrator.
  integrator_->Initialize();

  // Restore default values.
  ResetStatistics();

  // Process all the initialization events.
  auto init_events = system_.AllocateCompositeEventCollection();
  system_.GetInitializationEvents(*context_, init_events.get());

  // Do unrestricted updates first.
  HandleUnrestrictedUpdate(init_events->get_unrestricted_update_events());
  // Do restricted (discrete variable) updates next.
  HandleDiscreteUpdate(init_events->get_discrete_update_events());

  // Gets all per-step events to be handled.
  per_step_events_ = system_.AllocateCompositeEventCollection();
  DRAKE_DEMAND(per_step_events_ != nullptr);
  system_.GetPerStepEvents(*context_, per_step_events_.get());

  // Allocate timed events collection.
  timed_events_ = system_.AllocateCompositeEventCollection();
  DRAKE_DEMAND(timed_events_ != nullptr);

  // Ensure that CalcNextUpdateTime() can return the current time by perturbing
  // current time as slightly toward negative infinity as we can allow.
  const T current_time = context_->get_time();
  const T slightly_before_current_time = internal::GetPreviousNormalizedValue(
      current_time);
  context_->set_time(slightly_before_current_time);

  // Get the next timed event.
  next_timed_event_time_ =
      system_.CalcNextUpdateTime(*context_, timed_events_.get());

  // Reset the context time.
  context_->set_time(current_time);

  // Indicate a timed event is to be handled, if appropriate.
  timed_or_witnessed_event_triggered_ =
      (next_timed_event_time_ == current_time);

  // Allocate the witness function collection.
  witnessed_events_ = system_.AllocateCompositeEventCollection();

  // Do any publishes last. Merge the initialization events with per-step
  // events and current_time timed events (if any). We expect all initialization
  // events to precede any per-step or timed events in the merged collection.
  // Note that per-step and timed discrete/unrestricted update events are *not*
  // processed here; just publish events.
  init_events->Merge(*per_step_events_);
  if (timed_or_witnessed_event_triggered_) {
    init_events->Merge(*timed_events_);
  }
  HandlePublish(init_events->get_publish_events());

  // TODO(siyuan): transfer publish entirely to individual systems.
  // Do a force-publish before the simulation starts.
  if (publish_at_initialization_) {
    system_.Publish(*context_);
    ++num_publishes_;
  }

  // Initialize runtime variables.
  initialization_done_ = true;
}

// Processes UnrestrictedUpdateEvent events.
template <typename T>
void Simulator<T>::HandleUnrestrictedUpdate(
    const EventCollection<UnrestrictedUpdateEvent<T>>& events) {
  if (events.HasEvents()) {
    // First, compute the unrestricted updates into a temporary buffer.
    system_.CalcUnrestrictedUpdate(*context_, events,
        unrestricted_updates_.get());
    // TODO(edrumwri): simply swap the states for additional speed.
    // Now write the update back into the context.
    State<T>& x = context_->get_mutable_state();
    x.CopyFrom(*unrestricted_updates_);
    ++num_unrestricted_updates_;

    // Mark the witness function vector as needing to be redetermined.
    redetermine_active_witnesses_ = true;
  }
}

// Processes DiscreteEvent events.
template <typename T>
void Simulator<T>::HandleDiscreteUpdate(
    const EventCollection<DiscreteUpdateEvent<T>>& events) {
  if (events.HasEvents()) {
    // First, compute the discrete updates into a temporary buffer.
    system_.CalcDiscreteVariableUpdates(*context_, events,
        discrete_updates_.get());
    // Then, write them back into the context.
    DiscreteValues<T>& xd = context_->get_mutable_discrete_state();
    xd.CopyFrom(*discrete_updates_);
    ++num_discrete_updates_;
  }
}

// Processes Publish events.
template <typename T>
void Simulator<T>::HandlePublish(
    const EventCollection<PublishEvent<T>>& events) {
  if (events.HasEvents()) {
    system_.Publish(*context_, events);
    ++num_publishes_;
  }
}

// TODO(edrumwri): Consider adding a special function to "complete" a simulation
//                 by calling, in effect, StepTo(get_context().get_time()) to
//                 process any events that have triggered but not been processed
//                 at the conclusion to a StepTo() call. At the present time, we
//                 believe the issue of remaining, unprocessed events is rarely
//                 likely to be important and can be circumvented in multiple
//                 ways, like calling StepTo(get_context().get_time()).
template <typename T>
void Simulator<T>::StepTo(const T& boundary_time) {
  if (!initialization_done_) Initialize();

  DRAKE_THROW_UNLESS(boundary_time >= context_->get_time());

  // Integrate until desired interval has completed.
  auto merged_events = system_.AllocateCompositeEventCollection();
  DRAKE_DEMAND(timed_events_ != nullptr);
  DRAKE_DEMAND(witnessed_events_ != nullptr);
  DRAKE_DEMAND(merged_events != nullptr);

  // Clear events for the loop iteration.
  merged_events->Clear();
  merged_events->Merge(*per_step_events_);

  // Merge in timed and witnessed events, if necessary.
  if (timed_or_witnessed_event_triggered_) {
    merged_events->Merge(*timed_events_);
    merged_events->Merge(*witnessed_events_);
  }

  while (true) {
    // Starting a new substep on the trajectory.
    const T step_start_time = context_->get_time();
    SPDLOG_TRACE(log(), "Starting a simulation substep at {}", step_start_time);

    // Delay to match target realtime rate if requested and possible.
    PauseIfTooFast();

    // The general policy here is to do actions in decreasing order of
    // "violence" to the state, i.e. unrestricted -> discrete -> continuous ->
    // publish. The "timed" actions happen before the "per step" ones.

    // Do unrestricted updates first.
    HandleUnrestrictedUpdate(merged_events->get_unrestricted_update_events());
    // Do restricted (discrete variable) updates next.
    HandleDiscreteUpdate(merged_events->get_discrete_update_events());

    // How far can we go before we have to handle timed events?
    next_timed_event_time_ =
        system_.CalcNextUpdateTime(*context_, timed_events_.get());
    DRAKE_DEMAND(next_timed_event_time_ >= step_start_time);

    // Determine whether the set of events requested by the System at
    // next_timed_event_time includes an Update action, a Publish action, or
    // both.
    T next_update_dt = std::numeric_limits<double>::infinity();
    T next_publish_dt = std::numeric_limits<double>::infinity();
    if (timed_events_->HasDiscreteUpdateEvents() ||
        timed_events_->HasUnrestrictedUpdateEvents()) {
      next_update_dt = next_timed_event_time_ - step_start_time;
    }
    if (timed_events_->HasPublishEvents()) {
      next_publish_dt = next_timed_event_time_ - step_start_time;
    }

    // Get the dt that gets to the boundary time.
    const T boundary_dt = boundary_time - step_start_time;

    // Integrate the continuous state forward in time.
    timed_or_witnessed_event_triggered_ = IntegrateContinuousState(
        next_publish_dt,
        next_update_dt,
        next_timed_event_time_,
        boundary_dt,
        witnessed_events_.get());

    // Update the number of simulation substeps taken.
    ++num_substeps_taken_;

    // TODO(sherm1) Constraint projection goes here.

    // Clear events for the next loop iteration.
    merged_events->Clear();

    // Merge in per-step events.
    merged_events->Merge(*per_step_events_);

    // Only merge timed / witnessed events in if an event was triggered.
    if (timed_or_witnessed_event_triggered_) {
      merged_events->Merge(*timed_events_);
      merged_events->Merge(*witnessed_events_);
    }

    // Handle any publish events at the end of the loop.
    HandlePublish(merged_events->get_publish_events());

    // TODO(siyuan): transfer per step publish entirely to individual systems.
    // Allow System a chance to produce some output.
    if (get_publish_every_time_step()) {
      system_.Publish(*context_);
      ++num_publishes_;
    }

    // Break out of the loop after timed and witnessed events are merged in
    // to the event collection and after any publishes.
    if (context_->get_time() >= boundary_time)
      break;
  }

  // TODO(edrumwri): Add test coverage to complete #8490.
  redetermine_active_witnesses_ = true;
}

template <class T>
optional<T> Simulator<T>::GetCurrentWitnessTimeIsolation() const {
  using std::max;

  // TODO(edrumwri): Add ability to disable witness time isolation through
  // a Simulator setting.

  // The scale factor for witness isolation accuracy, which can allow witness
  // function zeros to be isolated more or less tightly, for positive values
  // less than one and greater than one, respectively. This number should be a
  // reasonable default that allows witness isolation accuracy to be
  // commensurate with integrator accuracy for most systems.
  const double iso_scale_factor = 0.01;

  // TODO(edrumwri): Acquire characteristic time properly from the system
  //                 (i.e., modify the System to provide this value).
  const double characteristic_time = 1.0;

  // Get the accuracy setting.
  const optional<double>& accuracy = get_context().get_accuracy();

  // Determine the length of the isolation interval.
  if (integrator_->get_fixed_step_mode()) {
    // Look for accuracy information.
    if (accuracy) {
      return max(integrator_->get_working_minimum_step_size(),
                 T(iso_scale_factor * accuracy.value() *
                     integrator_->get_maximum_step_size()));
    } else {
      return optional<T>();
    }
  }

  // Integration with error control isolation window determination.
  if (!accuracy) {
    throw std::logic_error("Integrator is not operating in fixed step mode "
                               "and accuracy is not set in the context.");
  }

  // Note: the max computation is used (here and above) because it is
  // ineffectual to attempt to isolate intervals smaller than the current time
  // in the context can allow.
  return max(integrator_->get_working_minimum_step_size(),
             iso_scale_factor * accuracy.value() * characteristic_time);
}

// Isolates the first time at one or more witness functions triggered (in the
// interval [t0, tf]), to the requisite interval length.
// @param[in,out] on entry, the set of witness functions that triggered over
//                [t0, tf]; on exit, the set of witness functions that triggered
//                over [t0, tw], where tw is the first time that any witness
//                function triggered.
// @pre The context and state are at tf and x(tf), respectively, and at least
//      one witness function has triggered over [t0, tf].
// @post The context will be isolated to the first witness function trigger(s),
//       to within the requisite interval length. It is guaranteed that all
//       triggered witness functions change sign over [t0, tw].
// @note We assume that, if a witness function triggers over an interval
//       [a, b], it also triggers over any larger interval [a, d], for d > b
//       and d ≤ the maximum integrator step size (per WitnessFunction
//       documentation, we assume that a witness function crosses zero at most
//       once over an interval of size [t0, tf]).
template <class T>
void Simulator<T>::IsolateWitnessTriggers(
    const std::vector<const WitnessFunction<T>*>& witnesses,
    const VectorX<T>& w0,
    const T& t0, const VectorX<T>& x0, const T& tf,
    std::vector<const WitnessFunction<T>*>* triggered_witnesses) {

  // Verify that the vector of triggered witnesses is non-null.
  DRAKE_DEMAND(triggered_witnesses);

  // TODO(edrumwri): Speed this process using interpolation between states,
  // more powerful root finding methods, and/or introducing the concept of
  // a dead band.

  // Will need to alter the context repeatedly.
  Context<T>& context = get_mutable_context();

  // Get the witness isolation interval length.
  const optional<T> witness_iso_len = GetCurrentWitnessTimeIsolation();

  // Check whether witness functions *are* to be isolated. If not, the witnesses
  // that were triggered on entry will be the set that is returned.
  if (!witness_iso_len)
    return;

  // Mini function for integrating the system forward in time from t0.
  std::function<void(const T&)> integrate_forward =
      [&t0, &x0, &context, this](const T& t_des) {
    const T inf = std::numeric_limits<double>::infinity();
    context.set_time(t0);
    context.get_mutable_continuous_state().SetFromVector(x0);
    T t_remaining = t_des - t0;
    while (t_remaining > 0) {
      integrator_->IntegrateAtMost(inf, inf, t_remaining);
      t_remaining = t_des - context.get_time();
    }
  };

  // Loop until the isolation window is sufficiently small.
  SPDLOG_DEBUG(drake::log(),
      "Isolating witness functions using isolation window of {} over [{}, {}]",
      witness_iso_len.value(), t0, tf);
  VectorX<T> wc(witnesses.size());
  T a = t0;
  T b = tf;
  do {
    // Compute the midpoint and evaluate the witness functions at it.
    T c = (a + b) / 2;
    SPDLOG_DEBUG(drake::log(), "Integrating forward to time {}", c);
    integrate_forward(c);

    // See whether any witness functions trigger.
    bool trigger = false;
    for (size_t i = 0; i < witnesses.size(); ++i) {
      wc[i] = get_system().CalcWitnessValue(context, *witnesses[i]);
      if (witnesses[i]->should_trigger(w0[i], wc[i]))
        trigger = true;
    }

    // If no witness function triggered, we can continue integrating forward.
    if (!trigger) {
      // NOTE: Since we're always checking that the sign changes over [t0,c],
      // it's also feasible to replace the two lines below with "a = c" without
      // violating Simulator's contract to only integrate once over the interval
      // [a, c], for some c <= b before per-step events are handled (i.e., it's
      // unacceptable to take two steps of (c - a)/2 without processing per-step
      // events first). That change would avoid handling unnecessary per-step
      // events- we know no other events are to be handled between t0 and tf-
      // but the current logic appears easier to follow.
      SPDLOG_DEBUG(drake::log(), "No witness functions triggered up to {}", c);
      triggered_witnesses->clear();
      return;
    } else {
      b = c;
    }
  } while (b - a > witness_iso_len.value());

  // Determine the set of triggered witnesses.
  triggered_witnesses->clear();
  for (size_t i = 0; i < witnesses.size(); ++i) {
    if (witnesses[i]->should_trigger(w0[i], wc[i]))
      triggered_witnesses->push_back(witnesses[i]);
  }
}

// Integrates the continuous state forward in time while also locating
// the first zero of any triggered witness functions.
// @param next_publish_dt the *time step* at which the next publish event
//        occurs.
// @param next_update_dt the *time step* at which the next update event occurs.
// @param time_of_next_event the *time* at which the next timed event occurs.
// @param boundary_dt the maximum time step to take.
// @param events a non-null collection of events, which the method will clear
//        on entry.
// @returns `true` if integration terminated on an event trigger, indicating
//          that an event needs to be handled at the state on return.
template <class T>
bool Simulator<T>::IntegrateContinuousState(
    const T& next_publish_dt, const T& next_update_dt,
    const T& time_of_next_timed_event, const T& boundary_dt,
    CompositeEventCollection<T>* events) {
  using std::abs;

  // Clear the composite event collection.
  DRAKE_ASSERT(events);
  events->Clear();

  // Save the time and current state.
  const Context<T>& context = get_context();
  const T t0 = context.get_time();
  const VectorX<T> x0 = context.get_continuous_state().CopyToVector();

  // Note: this function is only called in one place and under the conditions
  // that (1) t0 + next_update_dt equals *either* time_of_next_timed_event *or*
  // infinity and (2) t0 + next_publish_dt equals *either*
  // time_of_next_timed_event or infinity. This function should work without
  // these assumptions being valid but might benefit from additional review.
  const double inf = std::numeric_limits<double>::infinity();
  const double zero_tol = 10 * std::numeric_limits<double>::epsilon();
  DRAKE_ASSERT(next_update_dt == inf ||
      abs(t0 + next_update_dt - time_of_next_timed_event) < zero_tol);
  DRAKE_ASSERT(next_publish_dt == inf ||
      abs(t0 + next_publish_dt - time_of_next_timed_event) < zero_tol);

  // Get the set of witness functions active at the current state.
  const System<T>& system = get_system();
  if (redetermine_active_witnesses_) {
    witness_functions_->clear();
    system.GetWitnessFunctions(context, witness_functions_.get());
    redetermine_active_witnesses_ = false;
  }
  const auto& witness_functions = *witness_functions_;

  // Evaluate the witness functions.
  w0_.resize(witness_functions.size());
  for (size_t i = 0; i < witness_functions.size(); ++i)
      w0_[i] = system.CalcWitnessValue(context, *witness_functions[i]);

  // Attempt to integrate. Updates and boundary times are consciously
  // distinguished between. See internal documentation for
  // IntegratorBase::StepOnceAtMost() for more information.
  typename IntegratorBase<T>::StepResult result =
      integrator_->IntegrateAtMost(next_publish_dt, next_update_dt,
                                  boundary_dt);
  const T tf = context.get_time();

  // Evaluate the witness functions again.
  wf_.resize(witness_functions.size());
  for (size_t i =0; i < witness_functions.size(); ++i)
    wf_[i] = system.CalcWitnessValue(context, *witness_functions[i]);

  // See whether a witness function triggered.
  triggered_witnesses_.clear();
  bool witness_triggered = false;
  for (size_t i =0; i < witness_functions.size() && !witness_triggered; ++i) {
      if (witness_functions[i]->should_trigger(w0_[i], wf_[i])) {
        witness_triggered = true;
        triggered_witnesses_.push_back(witness_functions[i]);
      }
  }

  // Triggering requires isolating the witness function time.
  if (witness_triggered) {
    // Isolate the time that the witness function triggered. If witness triggers
    // are detected in the interval [t0, tf], any additional time-triggered
    // events are only relevant iff at least one witness function is
    // successfully isolated (see IsolateWitnessTriggers() for details).
    IsolateWitnessTriggers(
        witness_functions, w0_, t0, x0, tf, &triggered_witnesses_);

    // Store the state at x0 in the temporary continuous state. We only do this
    // if there are triggered witnesses (even though `witness_triggered` is
    // `true`, the witness might not have actually triggered after isolation).
    if (!triggered_witnesses_.empty())
      event_handler_xc_->SetFromVector(x0);

    // Store witness function(s) that triggered.
    for (const WitnessFunction<T>* fn : triggered_witnesses_) {
      SPDLOG_DEBUG(drake::log(), "Witness function {} crossed zero at time {}",
                   fn->description(), context.get_time());

      // Skip witness functions that have no associated event.
      if (!fn->get_event())
        continue;

      // Get the event object that corresponds to this witness function. If
      // there is none, create it.
      auto& event = witness_function_events_[fn];
      if (!event) {
        event = fn->get_event()->Clone();
        event->set_trigger_type(TriggerType::kWitness);
        event->set_event_data(std::make_unique<WitnessTriggeredEventData<T>>());
      }

      // Populate the event data.
      auto event_data = static_cast<WitnessTriggeredEventData<T>*>(
          event->get_mutable_event_data());
      event_data->set_triggered_witness(fn);
      event_data->set_t0(t0);
      event_data->set_tf(tf);
      event_data->set_xc0(event_handler_xc_.get());
      event_data->set_xcf(&context_->get_continuous_state());
      system.AddTriggeredWitnessFunctionToCompositeEventCollection(
          event.get(),
          events);
    }

    // Indicate an event should be triggered if at least one witness function
    // triggered (meaning that an event should be handled on the next simulation
    // loop). If no witness functions triggered over a smaller interval (recall
    // that we're in this if/then conditional block because a witness triggered
    // over a larger interval), we know that time advanced and that no events
    // triggered.
    return !triggered_witnesses_.empty();
  }

  // No witness function triggered; handle integration as usual.
  // Updates and boundary times are consciously distinguished between. See
  // internal documentation for IntegratorBase::StepOnceAtMost() for more
  // information.
  switch (result) {
    case IntegratorBase<T>::kReachedUpdateTime:
    case IntegratorBase<T>::kReachedPublishTime:
      // Next line sets the time to the exact event time rather than
      // introducing rounding error by summing the context time + dt.
      context_->set_time(time_of_next_timed_event);
      return true;            // Timed event hit.
      break;

    case IntegratorBase<T>::kTimeHasAdvanced:
    case IntegratorBase<T>::kReachedBoundaryTime:
      return false;           // Did not hit a time for a timed event.
      break;

    default:DRAKE_ABORT_MSG("Unexpected integrator result.");
  }

  // TODO(sherm1) Constraint projection goes here.

  // Should never get here.
  DRAKE_ABORT();
  return false;
}

}  // namespace systems
}  // namespace drake
