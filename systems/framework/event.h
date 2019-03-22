#pragma once

#include <limits>
#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_nodiscard.h"
#include "drake/common/value.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/event_status.h"

namespace drake {
namespace systems {

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
 * Base class for storing trigger-specific data to be passed to event handlers.
 */
class EventData {
 public:
  EventData() {}
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EventData);
  virtual ~EventData() {}
  virtual std::unique_ptr<EventData> Clone() const {
    return std::unique_ptr<EventData>(DoClone());
  }

 protected:
  DRAKE_NODISCARD virtual EventData* DoClone() const = 0;
};

/**
 * A token describing an event that recurs on a fixed period. The events are
 * triggered at time = offset_sec + i * period_sec, where i is a non-negative
 * integer.
 */
class PeriodicEventData : public EventData {
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

 private:
  DRAKE_NODISCARD EventData* DoClone() const override {
    PeriodicEventData* clone = new PeriodicEventData;
    clone->period_sec_ = period_sec_;
    clone->offset_sec_ = offset_sec_;
    return clone;
  }

  double period_sec_{0.0};
  double offset_sec_{0.0};
};

/**
 * Class for storing data from a witness function triggering to be passed
 * to event handlers. A witness function isolates the time to a (typically
 * small) window during which the witness function crosses zero. The time and
 * state at both sides of this window are passed to the event handler so that
 * the system can precisely determine the reason that the witness function
 * triggered.
 */
template <class T>
class WitnessTriggeredEventData : public EventData {
 public:
  WitnessTriggeredEventData() {}
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(WitnessTriggeredEventData);

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
  DRAKE_NODISCARD EventData* DoClone() const override {
    WitnessTriggeredEventData<T>* clone = new WitnessTriggeredEventData;
    clone->triggered_witness_ = triggered_witness_;
    clone->t0_ = t0_;
    clone->tf_ = tf_;
    clone->xc0_ = xc0_;
    clone->xcf_ = xcf_;
    return clone;
  }

  const WitnessFunction<T>* triggered_witness_{nullptr};
  T t0_{std::numeric_limits<double>::quiet_NaN()};
  T tf_{std::numeric_limits<double>::quiet_NaN()};
  const ContinuousState<T>* xc0_{nullptr};
  const ContinuousState<T>* xcf_{nullptr};
};

DRAKE_DEFINE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN_T(WitnessTriggeredEventData)

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
   * controls the time and state evolution of a System. For example, a
   * simulator is a `solver`. Its `step` advances time a finite duration by
   * integrating a system, modifying its state accordingly. Per-step events
   * are most commonly created in System::GetPerStepEvents(). A very common
   * use of such per-step events is to update a discrete or abstract state
   * variable that changes whenever the continuous state advances; examples
   * are computing the "min" or "max" of some state variable, recording a
   * signal in a delay buffer, or publishing. Per-step events are also useful
   * to implement feedback controllers interfaced with physical devices; the
   * controller can be implemented in the event handler, and the "step" would
   * correspond to receiving sensory data from the hardware.
   */
  kPerStep,

  /**
   * This trigger indicates that an associated event is triggered by the zero
   * crossing of a witness function.
   */
  kWitness,
};

/**
 * Abstract base class that represents an event. The base event contains two
 * main pieces of information: an enum trigger type and an optional attribute
 * of AbstractValue that can be used to explain why the event is triggered.
 * Derived classes should contain a function pointer to an optional callback
 * function that handles the event. No-op is the default handling behavior.
 * Currently, the System framework only supports three concrete event types:
 * PublishEvent, DiscreteUpdateEvent, and UnrestrictedUpdateEvent distinguished
 * by their callback functions' access level to the context.
 *
 * Event handling occurs during a simulation of a system. The logic that
 * describes when particular event types are handled is described in the
 * class documentation for Simulator.
 */
template <typename T>
class Event {
 public:
  /// Constructs an Event with no trigger type and no event data.
  Event() { trigger_type_ = TriggerType::kUnknown; }
  void operator=(const Event&) = delete;
  Event(Event&&) = delete;
  void operator=(Event&&) = delete;

  // TODO(eric.cousineau): Deprecate and remove this alias.
  using TriggerType = systems::TriggerType;

  /// Returns `true` if this is a DiscreteUpdateEvent.
  virtual bool is_discrete_update() const = 0;

  /**
   * An object passed
   */
  virtual ~Event() {}

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
   * Returns true if this event has associated data.
   */
  bool has_event_data() const { return event_data_ != nullptr; }

  /**
   * Returns a const pointer to the event data. The returned value
   * can be nullptr, which means this event does not have any associated
   * data.
   */
  const EventData* get_event_data() const { return event_data_.get(); }

  /**
   * Returns a mutable pointer to the event data. The returned value
   * can be nullptr, which means this event does not have any associated
   * data.
   */
  EventData* get_mutable_event_data() { return event_data_.get(); }

  // Note: Users should not be calling this.
  #if !defined(DRAKE_DOXYGEN_CXX)
  // Sets the trigger type.
  void set_trigger_type(const TriggerType trigger_type) {
    trigger_type_ = trigger_type; }

  // Sets and transfers the ownership of @p data.
  void set_event_data(std::unique_ptr<EventData> data) {
    event_data_ = std::move(data);
  }
  #endif

  /**
   * Adds a clone of `this` event to the event collection `events`, with
   * the given trigger type. If `this` event has an unknown trigger type, then
   * any trigger type is acceptable. Otherwise the given trigger type must
   * match match the trigger type stored in `this` event.
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
  Event(const Event& other) : trigger_type_(other.trigger_type_) {
    if (other.event_data_ != nullptr)
      set_event_data(other.event_data_->Clone());
  }

  // Note: Users should not be calling this.
  #if !defined(DRAKE_DOXYGEN_CXX)
  // Constructs an Event with the specified @p trigger.
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
  DRAKE_NODISCARD virtual Event* DoClone() const = 0;

 private:
  TriggerType trigger_type_;
  std::unique_ptr<EventData> event_data_{nullptr};
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
 * to do custom handling of this event given const Context and const
 * PublishEvent object references. @see System::Publish for more details.
 */
template <typename T>
class PublishEvent final : public Event<T> {
 public:
  void operator=(const PublishEvent&) = delete;
  PublishEvent(PublishEvent&&) = delete;
  void operator=(PublishEvent&&) = delete;
  bool is_discrete_update() const override { return false; }

  /**
   * Callback function that processes a publish event.
   */
  typedef std::function<void(const Context<T>&, const PublishEvent<T>&)>
      PublishCallback;

  /// Makes a PublishEvent with no trigger type, no event data, and
  /// no specified callback function.
  PublishEvent() : Event<T>() {}

  /// Makes a PublishEvent with no trigger type, no event data, and
  /// the specified callback function.
  explicit PublishEvent(const PublishCallback& callback)
      : Event<T>(), callback_(callback) {}

  // Note: Users should not be calling these.
  #if !defined(DRAKE_DOXYGEN_CXX)
  // Makes a PublishEvent with `trigger_type`, no event data, and
  // callback function `callback`, which can be null.
  PublishEvent(const TriggerType& trigger_type,
               const PublishCallback& callback)
      : Event<T>(trigger_type), callback_(callback) {}

  // Makes a PublishEvent with `trigger_type`, no event data, and
  // no specified callback function.
  explicit PublishEvent(const TriggerType& trigger_type)
      : Event<T>(trigger_type) {}
  #endif

  /**
   * Calls the optional callback function, if one exists, with @p context and
   * `this`.
   */
  void handle(const Context<T>& context) const {
    if (callback_ != nullptr) callback_(context, *this);
  }

 private:
  PublishEvent(const PublishEvent&);

  void DoAddToComposite(TriggerType trigger_type,
                        CompositeEventCollection<T>* events) const final {
    auto event = std::unique_ptr<PublishEvent<T>>(this->DoClone());
    event->set_trigger_type(trigger_type);
    events->add_publish_event(std::move(event));
  }

  // Clones PublishEvent-specific data.
  DRAKE_NODISCARD PublishEvent<T>* DoClone() const final {
    return new PublishEvent(*this);
  }

  // Optional callback function that handles this publish event.
  PublishCallback callback_{nullptr};
};

// Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728 which
// should be moved back into the class definition once we no longer need to
// support GCC versions prior to 6.3.
template <typename T>
PublishEvent<T>::PublishEvent(const PublishEvent<T>&) = default;

/**
 * This class represents a discrete update event. It has an optional callback
 * function to do custom handling of this event given const Context and
 * const DiscreteUpdateEvent references, and writes the updates to a mutable,
 * non-null DiscreteValues object.
 */
template <typename T>
class DiscreteUpdateEvent final : public Event<T> {
 public:
  void operator=(const DiscreteUpdateEvent&) = delete;
  DiscreteUpdateEvent(DiscreteUpdateEvent&&) = delete;
  void operator=(DiscreteUpdateEvent&&) = delete;
  bool is_discrete_update() const override { return true; }

  /**
   * Callback function that processes a discrete update event.
   */
  typedef std::function<void(const Context<T>&, const DiscreteUpdateEvent<T>&,
                             DiscreteValues<T>*)>
      DiscreteUpdateCallback;

  /// Makes a DiscreteUpdateEvent with no trigger type, no event data, and
  /// no specified callback function.
  DiscreteUpdateEvent() : Event<T>() {}

  /// Makes a DiscreteUpdateEvent with no trigger type, no event data, and
  /// the specified callback function.
  explicit DiscreteUpdateEvent(const DiscreteUpdateCallback& callback)
      : Event<T>(), callback_(callback) {}

  // Note: Users should not be calling these.
  #if !defined(DRAKE_DOXYGEN_CXX)
  // Makes a DiscreteUpdateEvent with `trigger_type` with no event data and
  // the callback function `callback`.
  // `callback` can be null.
  DiscreteUpdateEvent(const TriggerType& trigger_type,
                      const DiscreteUpdateCallback& callback)
      : Event<T>(trigger_type), callback_(callback) {}

  // Makes a DiscreteUpdateEvent with @p trigger_type with no event data and
  // no specified callback function.
  explicit DiscreteUpdateEvent(
      const TriggerType& trigger_type)
      : DiscreteUpdateEvent(trigger_type, nullptr) {}
  #endif

  /**
   * Calls the optional callback function, if one exists, with @p context,
   * 'this' and @p discrete_state.
   */
  void handle(const Context<T>& context,
              DiscreteValues<T>* discrete_state) const {
    if (callback_ != nullptr) callback_(context, *this, discrete_state);
  }

 private:
  DiscreteUpdateEvent(const DiscreteUpdateEvent&);

  void DoAddToComposite(TriggerType trigger_type,
                        CompositeEventCollection<T>* events) const final {
    auto event = std::unique_ptr<DiscreteUpdateEvent<T>>(this->DoClone());
    event->set_trigger_type(trigger_type);
    events->add_discrete_update_event(std::move(event));
  }

  // Clones DiscreteUpdateEvent-specific data.
  DRAKE_NODISCARD DiscreteUpdateEvent<T>* DoClone() const final {
    return new DiscreteUpdateEvent(this->get_trigger_type(), callback_);
  }

  // Optional callback function that handles this discrete update event.
  DiscreteUpdateCallback callback_{nullptr};
};

// Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728 which
// should be moved back into the class definition once we no longer need to
// support GCC versions prior to 6.3.
template <typename T>
DiscreteUpdateEvent<T>::DiscreteUpdateEvent(
    const DiscreteUpdateEvent<T>&) = default;

/**
 * This class represents an unrestricted update event. It has an optional
 * callback function to do custom handling of this event given const Context and
 * const UnrestrictedUpdateEvent object references, and writes the updates to a
 * mutable, non-null State object.
 */
template <typename T>
class UnrestrictedUpdateEvent final : public Event<T> {
 public:
  void operator=(const UnrestrictedUpdateEvent&) = delete;
  UnrestrictedUpdateEvent(UnrestrictedUpdateEvent&&) = delete;
  void operator=(UnrestrictedUpdateEvent&&) = delete;
  bool is_discrete_update() const override { return false; }

  /**
   * Callback function that processes an unrestricted update event.
   */
  typedef std::function<void(const Context<T>&,
                             const UnrestrictedUpdateEvent<T>&, State<T>*)>
      UnrestrictedUpdateCallback;

  /// Makes an UnrestrictedUpdateEvent with no trigger type, no event data, and
  /// no specified callback function.
  UnrestrictedUpdateEvent() : Event<T>() {}

  /// Makes a UnrestrictedUpdateEvent with no trigger type, no event data, and
  /// the specified callback function.
  explicit UnrestrictedUpdateEvent(const UnrestrictedUpdateCallback& callback)
      : Event<T>(), callback_(callback) {}

  // Note: Users should not be calling these.
  #if !defined(DRAKE_DOXYGEN_CXX)
  // Makes an UnrestrictedUpdateEvent with `trigger_type` and callback function
  // `callback`. `callback` can be null.
  UnrestrictedUpdateEvent(const TriggerType& trigger_type,
                          const UnrestrictedUpdateCallback& callback)
      : Event<T>(trigger_type), callback_(callback) {}

  // Makes an UnrestrictedUpateEvent with @p trigger_type, no optional data, and
  // no callback function.
  explicit UnrestrictedUpdateEvent(
      const TriggerType& trigger_type)
      : UnrestrictedUpdateEvent(trigger_type, nullptr) {}
  #endif

  /**
   * Calls the optional callback function, if one exists, with @p context,
   * `this` and @p discrete_state.
   */
  void handle(const Context<T>& context, State<T>* state) const {
    if (callback_ != nullptr) callback_(context, *this, state);
  }

 private:
  UnrestrictedUpdateEvent(const UnrestrictedUpdateEvent&);

  void DoAddToComposite(TriggerType trigger_type,
                        CompositeEventCollection<T>* events) const final {
    auto event = std::unique_ptr<UnrestrictedUpdateEvent<T>>(this->DoClone());
    event->set_trigger_type(trigger_type);
    events->add_unrestricted_update_event(std::move(event));
  }

  // Clones event data specific to UnrestrictedUpdateEvent.
  UnrestrictedUpdateEvent<T>* DoClone() const final {
    return new UnrestrictedUpdateEvent(*this);
  }

  // Optional callback function that handles this unrestricted update event.
  UnrestrictedUpdateCallback callback_{nullptr};
};

// Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728 which
// should be moved back into the class definition once we no longer need to
// support GCC versions prior to 6.3.
template <typename T>
UnrestrictedUpdateEvent<T>::UnrestrictedUpdateEvent(
    const UnrestrictedUpdateEvent<T>&) = default;

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::WitnessTriggeredEventData)

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Event)

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::PublishEvent)

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteUpdateEvent)

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::UnrestrictedUpdateEvent)
