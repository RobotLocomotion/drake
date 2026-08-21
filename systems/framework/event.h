#pragma once

#include <limits>
#include <memory>
#include <unordered_set>
#include <utility>
#include <variant>

#include "drake/common/drake_copyable.h"
#include "drake/common/value.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/event_status.h"

namespace drake {
namespace systems {

// Forward declaration to avoid circular dependencies.
template <typename T>
class System;

/** @defgroup events_description System Events
    @ingroup systems

 This page describes how Drake Systems can respond (through an Event) to changes
 ("triggers") in time, state, and inputs.

 The state of simple dynamical systems, like the ODE ẋ = x, can be
 propagated through time using straightforward numerical integration. More
 sophisticated systems (e.g., systems modeled using piecewise differential
 algebraic equations, systems dependent upon mouse button clicks, and
 @ref discrete_systems) require more sophisticated state updating mechanisms.
 We call those state updates "events", the conditions that cause these events
 "triggers", and the mechanisms that compute the updates "handlers". We discuss
 these concepts in detail on this page. The Simulator class documentation
 describes the technical process underlying how events are handled in great
 detail.

 ### Exposition

 Events occur between discrete, finite advancements of systems' time
 and state. In the absence of events, a simulator would happily advance time and
 any continuous state without stopping. The occurrence of an event pauses the
 time and state evolution in order to allow a system to change its state
 discontinuously, to communicate with the "outside world", and even to just
 count the number of event occurrences.

 ### Types of events and triggers

 The diagram below distinguishes between the condition that is responsible for
 detecting the event (the "trigger") and the action that is taken when the event
 is dispatched (the "event handler").

                                -- > handler1
     triggers  -- >  dispatcher -- > handler2
             Events             -- > etc.

 Handler actions fall into several categories based on event type, as
 described next.

 #### %Event types

 Events are grouped by the component(s) of a system's State that can be altered:

 - "publish" events can modify no state: they are useful for broadcasting data
    outside of the novel system and any containing diagrams, for terminating
    a simulation, for detecting errors, and for forcing boundaries between
    integration steps.

 - "discrete update" events can alter the discrete state of a system.

 - "unrestricted update" events can alter every component of state but time:
    continuous state, discrete state, and abstract state.

 Note that continuous state is nominally updated through the
 process of solving an ODE initial value problem (i.e., "integrating") rather
 than through a specialized event.

 Updates are performed in a particular sequence. For example, unrestricted
 updates are performed before discrete updates. The Simulator documentation
 describes the precise update ordering.

 #### %Event triggers

 Events can be triggered in various ways including:
 - upon initialization
 - as a certain time is crossed (whether once or repeatedly, i.e.,
   periodically)
 - per simulation step
 - as a WitnessFunction crosses zero
 - "by force", e.g., the system's CalcUnrestrictedUpdate() function is invoked
   by some user code

 ### How events are handled

 State advances with time in dynamical systems. If the dynamical system is
 simulated, then Drake's Simulator, or another solver (see @ref event_glossary
 "glossary") is responsible for detecting when events trigger, dispatching the
 appropriate handler function, and updating the state as time advances. The
 update functions modify only copies of state so that every update function in a
 class (e.g., all unrestricted updates) sees the same pre-update state
 regardless of the sequence of event updates).

 Events can also be dispatched manually ("by force"), i.e., outside of a solver.
 As noted above, one could call CalcUnrestrictedUpdate() to determine how a
 system's state would change and, optionally, update that state manually. Here
 is a simple example illustrating a forced publish:
 ```
   SystemX y;
   std::unique_ptr<Context<T>> context = y.CreateDefaultContext();
   y.Publish(*context);
 ```

 ### Information for leaf system authors

 #### Declaring update functions

 The way to update state through events is to declare an update handler in your
 LeafSystem-derived-class.

 A number of convenience functions are available in LeafSystem for declaring
 various trigger and event update combinations; see, e.g.,
 LeafSystem::DeclarePeriodicPublishEvent(),
 LeafSystem::DeclarePerStepDiscreteUpdateEvent(), and
 LeafSystem::DeclareInitializationUnrestrictedUpdateEvent().

 The following sample code shows how to declare a publish event that is
 triggered at regular time intervals:
 ```
   template <typename T>
   class MySystem : public LeafSystem<T> {
    MySystem() {
      const double period = 1.0;
      const double offset = 0.0;
      this->DeclarePeriodicPublishEvent(period, offset, &MySystem::MyPublish);
    }

    // Called once per second when MySystem is simulated.
    EventStatus MyPublish(const Context<T>&) const { ... }
   };
 ```

 #### Trigger data carried with Events

 It can be impractical or infeasible to create a different handler function for
 every possible trigger that a leaf system might need to consider. The
 alternative is to create a single event handler and map multiple triggers to
 it via multiple %Event objects that specify the same handler. Then the handler
 may need a way to determine which condition triggered it.

 For this purpose, every %Event stores the type of trigger associated with it
 and, if relevant, some extra data that provides greater insight into why the
 event handler was invoked. The latter is stored in an `std::variant` field
 that is currently defined for periodic-triggered timing information and for
 witness-triggered localization information. For example:
 ```
   template <typename T>
   class MySystem : public LeafSystem<T> {
    MySystem() {
      const double period1 = 1.0;
      const double period2 = 2.0;
      const double offset = 0.0;

      // Declare a publish event with one period.
      this->DeclarePeriodicEvent(period1, offset, PublishEvent<T>(
          TriggerType::kPeriodic,
          [this](const Context<T>& context, const PublishEvent<T>& event) ->
              EventStatus {
            return MyPublish(context, event);
          }));

      // Declare a second publish event with another period.
      this->DeclarePeriodicEvent(period2, offset, PublishEvent<T>(
          TriggerType::kPeriodic,
          [this](const Context<T>& context, const PublishEvent<T>& event) ->
              EventStatus {
            return MyPublish(context, event);
          }));
    }

    // A single update handler for all triggered events.
    EventStatus MyPublish(const Context<T>&, const PublishEvent<T>& e) const {
      if (e.get_trigger_type() == TriggerType::kPeriodic) {
        std::cout << "Event period: "
            << e.template get_event_data<PeriodicEventData>()->period_sec()
            << std::endl;
      }
    }
   };
 ```
 @see PeriodicEventData, WitnessTriggeredEventData

 #### %Event status

 %Event handlers can return an EventStatus type to modulate the behavior of a
 solver. Returning EventStatus::kFailed from the event handler indicates
 that the event handler was unable to update the state (because, e.g., the
 simulation step was too big) and thus the solver should take corrective action.
 Or an event handler can return EventStatus::kReachedTermination to indicate
 that the solver should stop computing; this is useful if the event handler
 detects that a simulated walking robot has fallen over and that the end of a
 reinforcement learning episode has been observed, for example.

 ### Glossary
 @anchor event_glossary

  - **dispatch**: the process of the solver collecting events that trigger
                  simultaneously and then distributing those events to their
                  handlers.
  - **forced event**: when an event is triggered manually through user
                      code rather than by a solver.
  - **handle/handler**: an event is "handled" when the triggering condition has
                        been identified and the "handler" function is called.
  - **solver**: a process that controls or tracks the time and state evolution
                of a System.
  - **trigger**: the condition responsible for causing an event.

 */

template <class T>
class WitnessFunction;

// Forward declaration of the event container classes.
// -- Containers for homogeneous events.
template <typename EventType>
class EventCollection;
template <typename EventType>
class LeafEventCollection;

// -- Containers for heterogeneous events.
template <typename T>
class CompositeEventCollection;
template <typename T>
class LeafCompositeEventCollection;

/**
 * An event data variant describing an event that recurs on a fixed period. The
 * events are triggered at time = offset_sec + i * period_sec, where i is a
 * non-negative integer.
 */
class PeriodicEventData {
 public:
  PeriodicEventData() {}
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PeriodicEventData);

  /// Gets the period with which this event should recur.
  double period_sec() const { return period_sec_; }

  /// Sets the period with which this event should recur.
  void set_period_sec(double period_sec) { period_sec_ = period_sec; }

  /// Gets the time after zero when this event should first occur.
  double offset_sec() const { return offset_sec_; }

  /// Sets the time after zero when this event should first occur.
  void set_offset_sec(double offset_sec) { offset_sec_ = offset_sec; }

  bool operator==(const PeriodicEventData& other) const {
    return other.period_sec() == period_sec() &&
           other.offset_sec() == offset_sec();
  }

 private:
  double period_sec_{0.0};
  double offset_sec_{0.0};
};

/**
 * An event data variant for storing data from a witness function triggering to
 * be passed to event handlers. A witness function isolates time to a (typically
 * small) window during which the witness function crosses zero. The time and
 * state at both sides of this window are passed to the event handler so that
 * the system can precisely determine the reason that the witness function
 * triggered.
 */
template <class T>
class WitnessTriggeredEventData {
 public:
  WitnessTriggeredEventData() {}
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(WitnessTriggeredEventData);

  /// Gets the witness function that triggered the event handler.
  const WitnessFunction<T>* triggered_witness() const {
    return triggered_witness_;
  }

  /// Sets the witness function that triggered the event handler.
  void set_triggered_witness(const WitnessFunction<T>* triggered_witness) {
    triggered_witness_ = triggered_witness;
  }

  /// Gets the time at the left end of the window. Default is NaN (which
  /// indicates that the value is invalid).
  const T& t0() const { return t0_; }

  /// Sets the time at the left end of the window. Note that `t0` should be
  /// smaller than `tf` after both values are set.
  void set_t0(const T& t0) { t0_ = t0; }

  /// Gets the time at the right end of the window. Default is NaN (which
  /// indicates that the value is invalid).
  const T& tf() const { return tf_; }

  /// Sets the time at the right end of the window. Note that `tf` should be
  /// larger than `t0` after both values are set.
  void set_tf(const T& tf) { tf_ = tf; }

  /// Gets a pointer to the continuous state at the left end of the isolation
  /// window.
  const ContinuousState<T>* xc0() const { return xc0_; }

  /// Sets a pointer to the continuous state at the left end of the isolation
  /// window.
  void set_xc0(const ContinuousState<T>* xc0) { xc0_ = xc0; }

  /// Gets a pointer to the continuous state at the right end of the isolation
  /// window.
  const ContinuousState<T>* xcf() const { return xcf_; }

  /// Sets a pointer to the continuous state at the right end of the isolation
  /// window.
  void set_xcf(const ContinuousState<T>* xcf) { xcf_ = xcf; }

 private:
  const WitnessFunction<T>* triggered_witness_{nullptr};
  T t0_{std::numeric_limits<double>::quiet_NaN()};
  T tf_{std::numeric_limits<double>::quiet_NaN()};
  const ContinuousState<T>* xc0_{nullptr};
  const ContinuousState<T>* xcf_{nullptr};
};

/**
 * Predefined types of triggers for events. Used at run time to determine why
 * the associated event has occurred.
 */
enum class TriggerType {
  kUnknown,

  /**
   * This trigger indicates that an associated event is triggered at system
   * initialization.
   */
  kInitialization,

  /**
   * This trigger indicates that an associated event is triggered by directly
   * calling the corresponding public system API for event handling (e.g.
   * Publish(context)).
   */
  kForced,

  /**
   * This trigger indicates that an associated event is triggered by the
   * system proceeding to a single, arbitrary time. Timed events are commonly
   * created in System::CalcNextUpdateTime().
   */
  kTimed,

  /**
   * This type indicates that an associated event is triggered by the system
   * proceeding to a time t ∈ {tᵢ = t₀ + p * i} for some period p, time
   * offset t₀, and i is a non-negative integer. @see PeriodicEventData.
   * Periodic events are commonly created in System::CalcNextUpdateTime().
   */
  kPeriodic,

  /**
   * This trigger indicates that an associated event is triggered whenever a
   * `solver` takes a `step`. A `solver` is an abstract construct that
   * controls or tracks the time and state evolution of a System. A simulator is
   * a `solver`- it advances time a finite duration by integrating a system,
   * modifying its state accordingly- as is a process that receives some numeric
   * state from IPC that is then used to, e.g., update abstract state.
   * Steps may occur at irregular time intervals: a step typically coincides
   * with a point in time where it is advantageous to poll for events, like
   * immediately after an integrator has advanced time and state.
   *
   * Per-step events are most commonly created in System::GetPerStepEvents(). A
   * very common use of such per-step events is to update a discrete or abstract
   * state variable that changes whenever the continuous state advances;
   * examples are computing the "min" or "max" of some state variable, recording
   * a signal in a delay buffer, or publishing. Per-step events are also useful
   * to implement feedback controllers interfaced with physical devices; the
   * controller can be implemented in the event handler, and the "step" would
   * correspond to receiving sensory data from the hardware.
   */
  kPerStep,

  /**
   * This trigger indicates that an associated event is triggered by the zero
   * crossing of a witness function. @see WitnessTriggeredEventData.
   */
  kWitness,
};

/**
 * This set-type alias provides a convenient API vocabulary for systems to
 * specify multiple trigger types.
 */
using TriggerTypeSet = std::unordered_set<TriggerType, DefaultHash>;

/** Abstract base class that represents an event. The base event contains two
main pieces of information: an enum trigger type and an optional attribute
that can be used to explain why the event is triggered.

Concrete derived classes contain a function pointer to an optional callback that
handles the event. No-op is the default handling behavior.  The System framework
supports three concrete event types: PublishEvent, DiscreteUpdateEvent, and
UnrestrictedUpdateEvent distinguished by their callback functions' write access
level to the State.

The most common and convenient use of events and callbacks will happen via the
LeafSystem Declare*Event() methods. To that end, the callback signature always
passes the `const %System<T>&` as the first argument, so that %LeafSystem does
not need to capture `this` into the lambda; typically only a pointer-to-member-
function is captured. Capturing any more than that would defeat std::function's
small buffer optimization and cause heap allocations when scheduling events.

%Event handling occurs during a simulation of a system. The logic that describes
when particular event types are handled is described in the class documentation
for Simulator. */
template <typename T>
class Event {
 public:
  virtual ~Event();

  // TODO(eric.cousineau): Deprecate and remove this alias.
  using TriggerType = systems::TriggerType;

  /// Returns `true` if this is a DiscreteUpdateEvent.
  virtual bool is_discrete_update() const = 0;

  /**
   * Clones this instance.
   */
  std::unique_ptr<Event> Clone() const {
    return std::unique_ptr<Event>(DoClone());
  }

  /**
   * Returns the trigger type.
   */
  TriggerType get_trigger_type() const { return trigger_type_; }

  /**
   * Returns true if this event has associated data of the given
   * `EventDataType`.
   *
   * @tparam EventDataType the expected data type for an event that has this
   *     trigger type (PeriodicEventData or WitnessTriggeredEventData).
   */
  template <typename EventDataType>
  bool has_event_data() const {
    return std::holds_alternative<EventDataType>(event_data_);
  }

  /**
   * Returns a const pointer to the event data. The returned value
   * can be nullptr, which means this event does not have any associated
   * data of the given `EventDataType`.
   *
   * @tparam EventDataType the expected data type for an event that has this
   *     trigger type (PeriodicEventData or WitnessTriggeredEventData).
   */
  template <typename EventDataType>
  const EventDataType* get_event_data() const {
    return std::get_if<EventDataType>(&event_data_);
  }

  /**
   * Returns a mutable pointer to the event data. The returned value
   * can be nullptr, which means this event does not have any associated
   * data of the given `EventDataType`.
   *
   * @tparam EventDataType the expected data type for an event that has this
   *     trigger type (PeriodicEventData or WitnessTriggeredEventData).
   */
  template <typename EventDataType>
  EventDataType* get_mutable_event_data() {
    return std::get_if<EventDataType>(&event_data_);
  }

#if !defined(DRAKE_DOXYGEN_CXX)
  /* (Internal use only) Sets the trigger type. */
  void set_trigger_type(const TriggerType trigger_type) {
    trigger_type_ = trigger_type;
  }

  /* (Internal use only) Sets data to one of the available variants. */
  template <typename EventDataType>
  void set_event_data(EventDataType data) {
    event_data_ = std::move(data);
  }
#endif

  /**
   * Adds a clone of `this` event to the event collection `events`, with
   * the given trigger type. If `this` event has an unknown trigger type, then
   * any trigger type is acceptable. Otherwise the given trigger type must
   * match the trigger type stored in `this` event.
   * @pre `trigger_type` must match the current trigger type unless that is
   *      unknown.
   * @pre `events` must not be null.
   */
  void AddToComposite(TriggerType trigger_type,
                      CompositeEventCollection<T>* events) const {
    DRAKE_DEMAND(events != nullptr);
    DRAKE_DEMAND(trigger_type_ == TriggerType::kUnknown ||
                 trigger_type_ == trigger_type);
    DoAddToComposite(trigger_type, &*events);
  }

  /**
   * Provides an alternate signature for adding an Event that already has the
   * correct trigger type set. Must not have an unknown trigger type.
   */
  void AddToComposite(CompositeEventCollection<T>* events) const {
    DRAKE_DEMAND(events != nullptr);
    DRAKE_DEMAND(trigger_type_ != TriggerType::kUnknown);
    DoAddToComposite(trigger_type_, &*events);
  }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Event);

  /** Constructs an empty Event. */
  Event() = default;

#if !defined(DRAKE_DOXYGEN_CXX)
  /* (Internal use only) */
  explicit Event(const TriggerType& trigger) : trigger_type_(trigger) {}
#endif

  /**
   * Derived classes must implement this to add a clone of this Event to
   * the event collection and unconditionally set its trigger type.
   */
  virtual void DoAddToComposite(TriggerType trigger_type,
                                CompositeEventCollection<T>* events) const = 0;

  /**
   * Derived classes must implement this method to clone themselves. Any
   * Event-specific data is cloned using the Clone() method. Data specific
   * to the class derived from Event must be cloned by the implementation.
   */
  [[nodiscard]] virtual Event* DoClone() const = 0;

 private:
  TriggerType trigger_type_{TriggerType::kUnknown};
  std::variant<std::monostate, PeriodicEventData, WitnessTriggeredEventData<T>>
      event_data_;
};

/**
 * Structure for comparing two PeriodicEventData objects for use in a map
 * container, using an arbitrary comparison method.
 */
struct PeriodicEventDataComparator {
  bool operator()(const PeriodicEventData& a,
                  const PeriodicEventData& b) const {
    if (a.period_sec() == b.period_sec())
      return a.offset_sec() < b.offset_sec();
    return a.period_sec() < b.period_sec();
  }
};

/**
 * This class represents a publish event. It has an optional callback function
 * to do custom handling of this event. @see System::Publish for more details.
 * @see LeafSystem for more convenient interfaces to publish events via the
 * Declare*PublishEvent() methods.
 */
template <typename T>
class PublishEvent final : public Event<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PublishEvent);
  bool is_discrete_update() const override { return false; }

  /** Constructs an empty PublishEvent. */
  PublishEvent() = default;

  /** Constructs a PublishEvent with the given callback function. */
  explicit PublishEvent(
      const std::function<EventStatus(const System<T>&, const Context<T>&,
                                      const PublishEvent<T>&)>& callback)
      : callback_(callback) {}

#if !defined(DRAKE_DOXYGEN_CXX)
  /* (Internal use only) */
  explicit PublishEvent(const TriggerType& trigger_type) {
    this->set_trigger_type(trigger_type);
  }
  /* (Internal use only) */
  template <typename Function>
  PublishEvent(const TriggerType& trigger_type, const Function& callback) {
    this->set_trigger_type(trigger_type);
    callback_ = callback;
  }
#endif

  // TODO(jwnimmer-tri) Annotate [[nodiscard]] on the return value.
  // Possibly fix CamelCase at the same time, e.g., InvokeCallback().
  /**
   * Calls the optional callback or system callback function, if one exists,
   * with @p system, @p context, and `this`.
   */
  EventStatus handle(const System<T>& system, const Context<T>& context) const {
    return (callback_ != nullptr) ? callback_(system, context, *this)
                                  : EventStatus::DidNothing();
  }

 private:
  void DoAddToComposite(TriggerType trigger_type,
                        CompositeEventCollection<T>* events) const final {
    PublishEvent event(*this);
    event.set_trigger_type(trigger_type);
    events->AddPublishEvent(std::move(event));
  }

  // Clones PublishEvent-specific data.
  [[nodiscard]] PublishEvent<T>* DoClone() const final {
    return new PublishEvent(*this);
  }

  // Optional callback function that handles this event.
  // It is valid for no callback to be set.
  std::function<EventStatus(const System<T>&, const Context<T>&,
                            const PublishEvent<T>&)>
      callback_;
};

/**
 * This class represents a discrete update event. It has an optional callback
 * function to do custom handling of this event, and that can write updates to a
 * mutable, non-null DiscreteValues object.  @see LeafSystem for more convenient
 * interfaces to discrete update events via the Declare*DiscreteUpdateEvent()
 * methods.
 */
template <typename T>
class DiscreteUpdateEvent final : public Event<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiscreteUpdateEvent);
  bool is_discrete_update() const override { return true; }

  /** Constructs an empty DiscreteUpdateEvent. */
  DiscreteUpdateEvent() = default;

  /** Constructs a DiscreteUpdateEvent with the given callback function. */
  explicit DiscreteUpdateEvent(
      const std::function<EventStatus(const System<T>&, const Context<T>&,
                                      const DiscreteUpdateEvent<T>&,
                                      DiscreteValues<T>*)>& callback)
      : callback_(callback) {}

#if !defined(DRAKE_DOXYGEN_CXX)
  /* (Internal use only) */
  explicit DiscreteUpdateEvent(const TriggerType& trigger_type) {
    this->set_trigger_type(trigger_type);
  }
  /* (Internal use only) */
  template <typename Function>
  DiscreteUpdateEvent(const TriggerType& trigger_type,
                      const Function& callback) {
    this->set_trigger_type(trigger_type);
    callback_ = callback;
  }
#endif

  // TODO(jwnimmer-tri) Annotate [[nodiscard]] on the return value.
  // Possibly fix CamelCase at the same time, e.g., InvokeCallback().
  /**
   * Calls the optional callback function, if one exists, with @p system, @p
   * context, `this` and @p discrete_state.
   */
  EventStatus handle(const System<T>& system, const Context<T>& context,
                     DiscreteValues<T>* discrete_state) const {
    return (callback_ != nullptr)
               ? callback_(system, context, *this, discrete_state)
               : EventStatus::DidNothing();
  }

 private:
  void DoAddToComposite(TriggerType trigger_type,
                        CompositeEventCollection<T>* events) const final {
    DiscreteUpdateEvent<T> event(*this);
    event.set_trigger_type(trigger_type);
    events->AddDiscreteUpdateEvent(std::move(event));
  }

  // Clones DiscreteUpdateEvent-specific data.
  [[nodiscard]] DiscreteUpdateEvent<T>* DoClone() const final {
    return new DiscreteUpdateEvent(*this);
  }

  // Optional callback functions that handle this event.
  // It is valid for no callback to be set.
  std::function<EventStatus(const System<T>&, const Context<T>&,
                            const DiscreteUpdateEvent<T>&, DiscreteValues<T>*)>
      callback_;
};

/**
 * This class represents an unrestricted update event. It has an optional
 * callback function to do custom handling of this event, and that can write
 * updates to a mutable, non-null State object. @see LeafSystem for more
 * convenient interfaces to unrestricted update events via the
 * Declare*UnrestrictedUpdateEvent() methods.
 */
template <typename T>
class UnrestrictedUpdateEvent final : public Event<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UnrestrictedUpdateEvent);
  bool is_discrete_update() const override { return false; }

  /** Constructs an empty UnrestrictedUpdateEvent. */
  UnrestrictedUpdateEvent() = default;

  /** Constructs an UnrestrictedUpdateEvent with the given callback function. */
  explicit UnrestrictedUpdateEvent(
      const std::function<EventStatus(const System<T>&, const Context<T>&,
                                      const UnrestrictedUpdateEvent<T>&,
                                      State<T>*)>& callback)
      : callback_(callback) {}

#if !defined(DRAKE_DOXYGEN_CXX)
  /* (Internal use only) */
  explicit UnrestrictedUpdateEvent(const TriggerType& trigger_type) {
    this->set_trigger_type(trigger_type);
  }
  /* (Internal use only) */
  template <typename Function>
  UnrestrictedUpdateEvent(const TriggerType& trigger_type,
                          const Function& callback) {
    this->set_trigger_type(trigger_type);
    callback_ = callback;
  }
#endif

  // TODO(jwnimmer-tri) Annotate [[nodiscard]] on the return value.
  // Possibly fix CamelCase at the same time, e.g., InvokeCallback().
  /**
   * Calls the optional callback function, if one exists, with @p system, @p
   * context, `this` and @p state.
   */
  EventStatus handle(const System<T>& system, const Context<T>& context,
                     State<T>* state) const {
    return (callback_ != nullptr) ? callback_(system, context, *this, state)
                                  : EventStatus::DidNothing();
  }

 private:
  void DoAddToComposite(TriggerType trigger_type,
                        CompositeEventCollection<T>* events) const final {
    UnrestrictedUpdateEvent<T> event(*this);
    event.set_trigger_type(trigger_type);
    events->AddUnrestrictedUpdateEvent(std::move(event));
  }

  // Clones event data specific to UnrestrictedUpdateEvent.
  UnrestrictedUpdateEvent<T>* DoClone() const final {
    return new UnrestrictedUpdateEvent(*this);
  }

  // Optional callback function that handles this event.
  // It is valid for no callback to be set.
  std::function<EventStatus(const System<T>&, const Context<T>&,
                            const UnrestrictedUpdateEvent<T>&, State<T>*)>
      callback_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::WitnessTriggeredEventData);

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Event);

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::PublishEvent);

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteUpdateEvent);

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::UnrestrictedUpdateEvent);
