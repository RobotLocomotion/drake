#pragma once

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/extract_double.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/simulator_status.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace systems {

namespace internal {
// Default value of target_realtime_rate_.
const double kDefaultTargetRealtimeRate = 0.0;

// Default integrator used by a Simulator.
const char* const kDefaultIntegratorName = "runge_kutta3";

// Default value of both publish_every_time_step_ and
// publish_at_initialization_.
const bool kDefaultPublishEveryTimeStep = false;
}  // namespace internal

/** @ingroup simulation
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
"Step()" that the %Simulator uses to incrementally advance the system
trajectory (time t and state x). The Simulator's AdvanceTo() method will be
defined in terms of Step below. In general, the length of a step is not known a
priori and is determined by the Step() algorithm. Each step consists of zero or
more unrestricted updates, followed by zero or more discrete updates, followed
by (possibly zero-length) continuous time and state advancement, followed by
zero or more publishes, and then a call to the monitor() function if one has
been defined. Updates, publishes, and the monitor can report errors or detect a
termination condition; that is not shown in the pseudocode below.

The pseudocode will clarify the effects on time and state of each of the update
stages above. This algorithm is given a starting Context value `{tₛ, x⁻(tₛ)}`
and returns an end Context value `{tₑ, x⁻(tₑ)}`, where tₑ is _no later_ than a
given tₘₐₓ.
```
// Advance the trajectory (time and state) from start value {tₛ, x⁻(tₛ)} to an
// end value {tₑ, x⁻(tₑ)}, where tₛ ≤ tₑ ≤ tₘₐₓ.
procedure Step(tₛ, x⁻(tₛ), tₘₐₓ)

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
  CallMonitor(tₑ, x⁻(tₑ))

  return {tₑ, x⁻(tₑ)}
```

We can use the notation and pseudocode to flesh out the AdvanceTo(),
AdvancePendingEvents(), and Initialize() functions. Termination and error
conditions detected by event handlers or the monitor are reported as status
returns from these methods.
```
// Advance the simulation until time tₘₐₓ.
procedure AdvanceTo(tₘₐₓ) → status
  t ← current_time
  while t < tₘₐₓ
    {tₑ, x⁻(tₑ)} ← Step(t, x⁻(t), tₘₐₓ)
    {t, x⁻(t)} ← {tₑ, x⁻(tₑ)}
  endwhile

// AdvancePendingEvents() is an advanced method, not commonly used.
// Perform just the start-of-step update to advance from x⁻(t) to x⁺(t).
procedure AdvancePendingEvents() → status
  t ≜ current_time, x⁻(t) ≜ current_state
  x⁺(t) ← DoAnyPendingUpdates(t, x⁻(t)) as in Step()
  x(t) ← x⁺(t)  // No continuous update needed.
  DoAnyPublishes(t, x(t))
  CallMonitor(t, x(t))

// Update time and state to {t₀, x⁻(t₀)}, which is the starting value of the
// trajectory, and thus the value the Context should contain at the start of the
// first simulation step.
procedure Initialize(t₀, x₀) → status
  x⁺(t₀) ← DoAnyInitializationUpdates as in Step()
  x⁻(t₀) ← x⁺(t₀)  // No continuous update needed.

  // ----------------------------------
  // Time and state are at {t₀, x⁻(t₀)}
  // ----------------------------------

  DoAnyPublishes(t₀, x⁻(t₀))
  CallMonitor(t₀, x⁻(t₀))
```
Initialize() can be viewed as a "0ᵗʰ step" that occurs before the first
Step() call as described above. Like Step(), Initialize() first
performs pending updates (in this case only initialization events can be
"pending"). Time doesn't advance so there is no continuous update phase and
witnesses cannot trigger. Finally, again like Step(), the initial trajectory
point `{t₀, x⁻(t₀)}` is provided to the handlers for any triggered publish
events. That includes initialization publish events, per-step publish events,
and periodic or timed publish events that trigger at t₀, followed by a call
to the monitor() function if one has been defined (a monitor is semantically
identical to a per-step publish).

@tparam_nonsymbolic_scalar
*/
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

  // TODO(sherm1) Make Initialize() attempt to satisfy constraints.
  // TODO(sherm1) Add a ReInitialize() or Resume() method that is called
  //              automatically by AdvanceTo() if the Context has changed.
  /// Prepares the %Simulator for a simulation. In order, the sequence of
  /// actions taken here are:
  /// - The active integrator's Initialize() method is invoked.
  /// - Statistics are reset.
  /// - Initialization update events are triggered and handled to produce the
  ///   initial trajectory value `{t₀, x(t₀)}`.
  /// - Then that initial value is provided to the handlers for any publish
  ///   events that have triggered, including initialization and per-step
  ///   publish events, periodic or other time-triggered publish events
  ///   that are scheduled for the initial time t₀, and finally a call to the
  ///   monitor() function if one has been defined.
  ///
  /// See the class documentation for more information. We recommend calling
  /// Initialize() explicitly prior to beginning a simulation so that error
  /// conditions will be discovered early. However, Initialize() will be called
  /// automatically by the first AdvanceTo() call if it hasn't already been
  /// called.
  ///
  /// @note If you make a change to the Context or to Simulator options between
  /// AdvanceTo() calls you should consider whether to call Initialize() before
  /// resuming; AdvanceTo() will not do that automatically for you. Whether to
  /// do so depends on whether you want the above initialization operations
  /// performed. For example, if you changed the time you will likely want the
  /// time-triggered events to be recalculated in case one is due at the new
  /// starting time.
  ///
  /// @warning Initialize() does not automatically attempt to satisfy System
  /// constraints -- it is up to you to make sure that constraints are
  /// satisfied by the initial conditions.
  ///
  /// This method will throw `std::logic_error` if the combination of options
  /// doesn't make sense. Other failures are possible from the System and
  /// integrator in use.
  ///
  /// @retval status A SimulatorStatus object indicating success, termination,
  ///                or an error condition as reported by event handlers or
  ///                the monitor function.
  /// @see AdvanceTo(), AdvancePendingEvents(), SimulatorStatus
  SimulatorStatus Initialize();

  /// Advances the System's trajectory until `boundary_time` is reached in
  /// the context or some other termination condition occurs. A variety of
  /// `std::runtime_error` conditions are possible here, as well as error
  /// conditions that may be thrown by the System when it is asked to perform
  /// computations. Be sure to enclose your simulation in a `try-catch` block
  /// and display the `what()` message.
  ///
  /// We recommend that you call Initialize() prior to making the first call
  /// to AdvanceTo(). However, if you don't it will be called for you the first
  /// time that you attempt a step, possibly resulting in unexpected error
  /// conditions. See documentation for `Initialize()` for the error conditions
  /// it might produce.
  ///
  /// @warning You should consider calling Initialize() if you alter the
  /// the Context or Simulator options between successive AdvanceTo() calls. See
  /// Initialize() for more information.
  ///
  /// @param boundary_time The maximum time to which the trajectory will be
  ///     advanced by this call to %AdvanceTo(). The method may return earlier
  ///     if an event or the monitor function requests termination or reports
  ///     an error condition.
  /// @retval status A SimulatorStatus object indicating success, termination,
  ///     or an error condition as reported by event handlers or the monitor
  ///     function. The time in the context will be set either to the
  ///     boundary_time or the time a termination or error was first detected.
  ///
  /// @pre The internal Context satisfies all System constraints or will after
  ///      pending Context updates are performed.
  /// @see Initialize(), AdvancePendingEvents(), SimulatorStatus
  SimulatorStatus AdvanceTo(const T& boundary_time);

  /// (Advanced) Handles discrete and abstract state update events that are
  /// pending from the previous AdvanceTo() call, without advancing time.
  /// See the %Simulator class description for details about how %Simulator
  /// advances time and handles events. In the terminology used there, this
  /// method advances the internal Context from `{t, x⁻(t)}` to `{t, x⁺(t)}`.
  ///
  /// Normally, these update events would be handled at the start of the next
  /// AdvanceTo() call, so this method is rarely needed. It can be useful
  /// at the end of a simulation or to get intermediate results when you are
  /// specifically interested in the `x⁺(t)` result.
  ///
  /// This method is equivalent to `AdvanceTo(current_time)`, where
  /// `current_time=simulator.get_context().get_time())`. If there are no
  /// pending events, nothing happens except possibly a final per-step publish
  /// call (if enabled) followed by a call to the monitor() function (if one
  /// has been provided).
  ///
  /// @retval status A SimulatorStatus object indicating success, termination,
  ///                or an error condition as reported by event handlers or
  ///                the monitor function.
  /// @see AdvanceTo(), Initialize(), SimulatorStatus
  SimulatorStatus AdvancePendingEvents() {
    return AdvanceTo(get_context().get_time());
  }

  /// Provides a monitoring function that will be invoked at the end of
  /// every step. (See the Simulator class documentation for a precise
  /// definition of "step".) A monitor() function can be used to capture the
  /// trajectory, to terminate the simulation, or to detect error conditions.
  /// The monitor() function is invoked by the %Simulator with a Context whose
  /// value is a point along the simulated trajectory. The monitor can be any
  /// functor and should capture any System references it needs to operate
  /// correctly.
  ///
  /// A monitor() function behaves the same as would a per-step Publish event
  /// handler included in the top-level System or Diagram being simulated. As in
  /// the case of Publish(), the monitor is called at the end of every step
  /// taken internally by AdvanceTo(), and also at the end of Initialize() and
  /// AdvancePendingEvents(). (See the Simulator class documentation for more
  /// detail about what happens when in these methods.) The monitor receives the
  /// top-level (root) Context, from which any sub-Context can be obtained using
  /// `subsystem.GetMyContextFromRoot()`, provided the necessary subsystem
  /// reference has been captured for use in the monitor.
  ///
  /// #### Examples
  /// Output time and continuous states whenever the trajectory is advanced:
  /// @code
  /// simulator.set_monitor([](const Context<T>& root_context) {
  ///   std::cout << root_context.get_time() << " "
  ///             << root_context.get_continuous_state_vector()
  ///             << std::endl;
  ///   return EventStatus::Succeeded();
  /// });
  /// @endcode
  ///
  /// Terminate early but successfully on a condition in a subsystem of the
  /// System diagram being simulated:
  /// @code
  /// simulator.set_monitor([&my_subsystem](const Context<T>& root_context) {
  ///   const Context<T>& subcontext =
  ///       my_subsystem.GetMyContextFromRoot(root_context);
  ///   if (my_subsystem.GoalReached(subcontext)) {
  ///     return EventStatus::ReachedTermination(my_subsystem,
  ///         "Simulation achieved the desired goal.");
  ///   }
  ///   return EventStatus::Succeeded();
  /// });
  /// @endcode
  /// In the above case, the Simulator's AdvanceTo() method will return early
  /// when the subsystem reports that it has reached its goal. The returned
  /// status will indicate the termination reason, and a human-readable
  /// termination message containing the message provided by the monitor can be
  /// obtained with status.FormatMessage().
  ///
  /// Failure due to plant center of mass falling below a threshold:
  /// @code
  /// simulator.set_monitor([&plant](const Context<T>& root_context) {
  ///   const Context<T>& plant_context =
  ///       plant.GetMyContextFromRoot(root_context);
  ///   const Vector3<T> com = plant.CalcCenterOfMassPosition(plant_context);
  ///   if (com[2] < 0.1) {  // Check z height of com.
  ///     return EventStatus::Failed(plant, "System fell over.");
  ///   }
  ///   return EventStatus::Succeeded();
  /// });
  /// @endcode
  /// In the above case the Simulator's AdvanceTo() method will throw an
  /// std::runtime_error containing a human-readable message including
  /// the text provided in the monitor.
  ///
  /// @note monitor() is called every time the trajectory is advanced by a step,
  /// which can mean it is called many times during a single AdvanceTo() call.
  ///
  /// @note The presence of a monitor has no effect on the step sizes taken,
  /// so a termination or error condition will be discovered only when first
  /// observed after a step is complete; it will not be further localized. Use
  /// witness-triggered events instead if you need precise isolation.
  void set_monitor(std::function<EventStatus(const Context<T>&)> monitor) {
    monitor_ = std::move(monitor);
  }

  /// Removes the monitoring function if there is one.
  /// @see set_monitor()
  void clear_monitor() { monitor_ = nullptr; }

  /// Obtains a reference to the monitoring function, which may be empty.
  /// @see set_monitor()
  const std::function<EventStatus(const Context<T>&)>& get_monitor() const {
    return monitor_;
  }

  // TODO(sherm1): Provide options for issuing a warning or aborting the
  // simulation if the desired rate cannot be achieved.
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

  /// Sets whether the simulation should trigger a forced-Publish event on the
  /// System under simulation at the end of every trajectory-advancing step.
  /// Specifically, that means the System::Publish() event dispatcher will be
  /// invoked on each subsystem of the System and passed the current Context
  /// and a forced-publish Event. If a subsystem has declared a forced-publish
  /// event handler, that will be called. Otherwise, nothing will happen unless
  /// the DoPublish() dispatcher has been overridden.
  ///
  /// Enabling this option does not cause a forced-publish to be triggered at
  /// initialization; if you want that you should also call
  /// `set_publish_at_initialization(true)`. If you want a forced-publish at the
  /// end of every step, you will usually also want one at the end of
  /// initialization, requiring both options to be enabled.
  ///
  /// @see LeafSystem::DeclareForcedPublishEvent()
  void set_publish_every_time_step(bool publish) {
    publish_every_time_step_ = publish;
  }

  /// Sets whether the simulation should trigger a forced-Publish at the end
  /// of Initialize(). See set_publish_every_time_step() documentation for
  /// more information.
  void set_publish_at_initialization(bool publish) {
    publish_at_initialization_ = publish;
  }

  /// Returns true if the set_publish_every_time_step() option has been
  /// enabled. By default, returns false.
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

  /// Gets the number of steps since the last Initialize() call. (We're
  /// not counting the Initialize() 0-length "step".) Note that every
  /// AdvanceTo() call can potentially take many steps.
  int64_t get_num_steps_taken() const { return num_steps_taken_; }

  /// Gets the number of discrete variable updates performed since the last
  /// Initialize() call.
  int64_t get_num_discrete_updates() const { return num_discrete_updates_; }

  /// Gets the number of "unrestricted" updates performed since the last
  /// Initialize() call.
  int64_t get_num_unrestricted_updates() const {
    return num_unrestricted_updates_; }

  /// Gets a reference to the integrator used to advance the continuous aspects
  /// of the system.
  const IntegratorBase<T>& get_integrator() const { return *integrator_.get(); }

  /// Gets a reference to the mutable integrator used to advance the continuous
  /// state of the system.
  IntegratorBase<T>& get_mutable_integrator() { return *integrator_.get(); }

  template <class U>
  DRAKE_DEPRECATED(
      "2020-08-01",
      "Use void or max-step-size version of reset_integrator() instead.")
  U* reset_integrator(std::unique_ptr<U> integrator) {
    if (!integrator)
      throw std::logic_error("Integrator cannot be null.");
    initialization_done_ = false;
    integrator_ = std::move(integrator);
    return static_cast<U*>(integrator_.get());
  }

  template <class U, typename... Args>
  DRAKE_DEPRECATED(
      "2020-05-01",
      "Use void or max-step-size version of reset_integrator() instead.")
  U* reset_integrator(Args&&... args) {
    auto integrator = std::make_unique<U>(std::forward<Args>(args)...);
    integrator->reset_context(&get_mutable_context());
    return reset_integrator(std::move(integrator));
  }

  /// Resets the integrator with a new one using factory construction.
  /// @code
  /// simulator.reset_integrator<RungeKutta3Integrator<double>>().
  /// @endcode
  /// Resetting the integrator resets the %Simulator such that it needs to be
  /// initialized again -- see Initialize() for details.
  /// @note Integrator needs a constructor of the form
  ///       Integrator(const System&, Context*); this
  ///       constructor is usually associated with error-controlled integrators.
  template <class Integrator>
  Integrator& reset_integrator() {
    static_assert(
        std::is_constructible<Integrator, const System<T>&, Context<T>*>::value,
        "Integrator needs a constructor of the form "
        "Integrator::Integrator(const System&, Context*); this "
        "constructor is usually associated with error-controlled integrators.");
    integrator_ =
        std::make_unique<Integrator>(get_system(), &get_mutable_context());
    initialization_done_ = false;
    return *static_cast<Integrator*>(integrator_.get());
  }

  /// Resets the integrator with a new one using factory construction and a
  /// maximum step size argument (which is required for constructing fixed-step
  /// integrators).
  /// @code
  /// simulator.reset_integrator<RungeKutta2Integrator<double>>(0.1).
  /// @endcode
  /// @see argument-less version of reset_integrator() for note about
  ///      initialization.
  /// @note Integrator needs a constructor of the form
  ///       Integrator(const System&, const T&, Context*); this
  ///       constructor is usually associated with fixed-step integrators (i.e.,
  ///       integrators which do not support error estimation).
  template <class Integrator>
  Integrator& reset_integrator(const T max_step_size) {
    static_assert(
        std::is_constructible<Integrator, const System<T>&, double,
                              Context<T>*>::value,
        "Integrator needs a constructor of the form "
        "Integrator::Integrator(const System&, const T&, Context*); this "
        "constructor is usually associated with fixed-step integrators.");
    integrator_ = std::make_unique<Integrator>(get_system(), max_step_size,
                                      &get_mutable_context());
    initialization_done_ = false;
    return *static_cast<Integrator*>(integrator_.get());
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
  std::optional<T> GetCurrentWitnessTimeIsolation() const;

  /// Gets a constant reference to the system.
  /// @note a mutable reference is not available.
  const System<T>& get_system() const { return system_; }

 private:
  enum TimeOrWitnessTriggered {
    kNothingTriggered = 0b00,
    kTimeTriggered = 0b01,
    kWitnessTriggered = 0b10,
    kBothTriggered = 0b11
  };

  // All constructors delegate to here.
  Simulator(
      const System<T>* system,
      std::unique_ptr<const System<T>> owned_system,
      std::unique_ptr<Context<T>> context);

  void HandleUnrestrictedUpdate(
      const EventCollection<UnrestrictedUpdateEvent<T>>& events);

  void HandleDiscreteUpdate(
      const EventCollection<DiscreteUpdateEvent<T>>& events);

  void HandlePublish(const EventCollection<PublishEvent<T>>& events);

  // Invoke the monitor() if there is one. If it wants termination we'll
  // update the Simulator status accordingly. If it reports failure,
  // currently we just throw.
  // TODO(sherm1) Add an option where the Simulator returns failed status
  // rather than throwing.
  void CallMonitorUpdateStatusAndMaybeThrow(SimulatorStatus* status) {
    DRAKE_DEMAND(status);
    if (!get_monitor()) return;
    const EventStatus monitor_status = get_monitor()(*context_);
    if (monitor_status.severity() == EventStatus::kReachedTermination) {
      status->SetReachedTermination(ExtractDoubleOrThrow(context_->get_time()),
                                    monitor_status.system(),
                                    monitor_status.message());
      return;
    }
    if (monitor_status.severity() == EventStatus::kFailed) {
      status->SetEventHandlerFailed(ExtractDoubleOrThrow(context_->get_time()),
                                    monitor_status.system(),
                                    monitor_status.message());
      throw std::runtime_error(status->FormatMessage());
    }
    // For any other condition, leave the status unchanged.
  }

  TimeOrWitnessTriggered IntegrateContinuousState(
      const T& next_publish_dt,
      const T& next_update_dt,
      const T& time_of_next_timed_event,
      const T& boundary_dt,
      CompositeEventCollection<T>* events);

  // Private methods related to witness functions.
  void IsolateWitnessTriggers(
      const std::vector<const WitnessFunction<T>*>& witnesses,
      const VectorX<T>& w0,
      const T& t0, const VectorX<T>& x0, const T& tf,
      std::vector<const WitnessFunction<T>*>* triggered_witnesses);
  void PopulateEventDataForTriggeredWitness(
      const T& t0, const T& tf, const WitnessFunction<T>* witness,
      Event<T>* event, CompositeEventCollection<T>* events) const;
  static bool DidWitnessTrigger(
    const std::vector<const WitnessFunction<T>*>& witness_functions,
    const VectorX<T>& w0,
    const VectorX<T>& wf,
    std::vector<const WitnessFunction<T>*>* triggered_witnesses);
  VectorX<T> EvaluateWitnessFunctions(
    const std::vector<const WitnessFunction<T>*>& witness_functions,
    const Context<T>& context) const;
  void RedetermineActiveWitnessFunctionsIfNecessary();

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
  double target_realtime_rate_{internal::kDefaultTargetRealtimeRate};

  bool publish_every_time_step_{internal::kDefaultPublishEveryTimeStep};

  bool publish_at_initialization_{internal::kDefaultPublishEveryTimeStep};

  // These are recorded at initialization or statistics reset.
  double initial_simtime_{nan()};  // Simulated time at start of period.
  TimePoint initial_realtime_;     // Real time at start of period.

  // The number of discrete updates since the last statistics reset.
  int64_t num_discrete_updates_{0};

  // The number of unrestricted updates since the last statistics reset.
  int64_t num_unrestricted_updates_{0};

  // The number of publishes since the last statistics reset.
  int64_t num_publishes_{0};

  // The number of integration steps since the last statistics reset.
  int64_t num_steps_taken_{0};

  // Set by Initialize() and reset by various traumas.
  bool initialization_done_{false};

  // The vector of active witness functions.
  std::unique_ptr<std::vector<const WitnessFunction<T>*>> witness_functions_;

  // Indicator for whether the Simulator needs to redetermine the active witness
  // functions.
  bool redetermine_active_witnesses_{true};

  // Per step events that are to be handled on every "major time step" (i.e.,
  // every successful completion of a step). This collection is constructed
  // within Initialize().
  std::unique_ptr<CompositeEventCollection<T>> per_step_events_;

  // Timed events can be triggered either at a particular time (like an alarm)
  // or periodically. This collection is constructed within Initialize().
  std::unique_ptr<CompositeEventCollection<T>> timed_events_;

  // Witnessed events are triggered as a witness function crosses zero during
  // AdvanceTo(). This collection is constructed within Initialize().
  std::unique_ptr<CompositeEventCollection<T>> witnessed_events_;

  // Indicates when a timed or witnessed event needs to be handled on the next
  // call to AdvanceTo().
  TimeOrWitnessTriggered time_or_witness_triggered_{
      TimeOrWitnessTriggered::kNothingTriggered
  };

  // The time that the next timed event is to be handled. This value is set in
  // both Initialize() and AdvanceTo().
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

  // Optional monitor() method to capture trajectory, terminate, or fail.
  std::function<EventStatus(const Context<T>&)> monitor_;
};

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

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::Simulator)
