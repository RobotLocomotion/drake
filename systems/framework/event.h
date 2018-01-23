#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

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
 * Abstract base class that represents an event. The base event contains two
 * main pieces of information: an enum trigger type and an optional attribute
 * of AbstractValue that can be used to explain why the event is triggered.
 * Derived classes should contain a function pointer to an optional callback
 * function that handles the event. No-op is the default handling behavior.
 * Currently, the System framework only supports three concrete event types:
 * PublishEvent, DiscreteUpdateEvent, and UnrestrictedUpdateEvent distinguished
 * by their callback functions' access level to the context.
 */
template <typename T>
class Event {
 public:
  void operator=(const Event&) = delete;
  Event(Event&&) = delete;
  void operator=(Event&&) = delete;

  /// Returns `true` if this is a DiscreteUpdateEvent.
  virtual bool is_discrete_update() const = 0;

  /**
   * Predefined types of triggers. Used at run time to determine why the
   * associated event has occurred.
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
     * offset t₀, and i is a non-negative integer. @see PeriodicAttribute.
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
     * crossing of a witness function. Witness events are commonly created by
     * WitnessFunction::AddEvent().
     */
    kWitness,
  };

  /**
   * A token describing an event that recurs on a fixed period. The events are
   * triggered at time = offset_sec + i * period_sec, where i is a non-negative
   * integer.
   */
  struct PeriodicAttribute {
    /**
     * The period with which this event should recur.
     */
    double period_sec{0.0};
    /**
     * The time after zero when this event should first occur.
     */
    double offset_sec{0.0};
  };

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
   * Returns true if this event has an associated attribute.
   */
  bool has_attribute() const { return attribute_ != nullptr; }

  /**
   * Returns a const pointer to the AbstractValue attribute. The returned value
   * can be nullptr, which means this event does not have an associated
   * attribute.
   */
  const AbstractValue* get_attribute() const { return attribute_.get(); }

  /**
   * Returns a const reference to the underlying attribute.
   *
   * @throws std::logic_error if there is no underlying attribute.
   * @throws std::bad_cast if @tparam DataType does not match the underlying
   *         attribute type.
   */
  template <typename DataType>
  const DataType& get_attribute() const {
    if (attribute_ == nullptr) throw std::logic_error("Data is null.");
    return attribute_->GetValueOrThrow<DataType>();
  }

  /**
   * Sets and transfers the ownership of @p attribute.
   */
  void set_attribute(std::unique_ptr<AbstractValue> attribute) {
    attribute_ = std::move(attribute);
  }

  /**
   * Adds `this` event to the event collection @p events. See derived
   * implementations for more details.
   */
  virtual void add_to_composite(CompositeEventCollection<T>* events) const = 0;

 protected:
  Event(const Event& other) : trigger_type_(other.trigger_type_) {
    if (other.attribute_ != nullptr)
      set_attribute(other.attribute_->Clone());
  }

  /**
   * Constructs an Event with the specified @p trigger.
   */
  explicit Event(const TriggerType& trigger) : trigger_type_(trigger) {}

  /**
   * Derived classes must implement this method to clone themselves. Any
   * Event-specific data is cloned using the Clone() method. Data specific
   * to the class derived from Event must be cloned by the implementation.
   */
  virtual Event* DoClone() const = 0;

 private:
  const TriggerType trigger_type_;
  std::unique_ptr<AbstractValue> attribute_{nullptr};
};

/// Structure for comparing two PeriodicAttributes for use in a map container,
/// using an arbitrary comparison method.
template <class T>
struct PeriodicAttributeComparator {
  bool operator()(const typename Event<T>::PeriodicAttribute& a,
    const typename Event<T>::PeriodicAttribute& b) const {
      if (a.period_sec == b.period_sec)
        return a.offset_sec < b.offset_sec;
      return a.period_sec < b.period_sec;
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

  /**
   * Makes a PublishEvent with @p trigger_type, no optional data, and
   * callback function @p callback, which can be null.
   */
  PublishEvent(const typename Event<T>::TriggerType& trigger_type,
               PublishCallback callback)
      : Event<T>(trigger_type), callback_(callback) {}

  /**
   * Makes a PublishEvent with @p trigger_type, no optional data, and
   * no specified callback function.
   */
  explicit PublishEvent(const typename Event<T>::TriggerType& trigger_type)
      : Event<T>(trigger_type) {}

  /**
   * Assuming that @p events is not null, this function makes a deep copy of
   * this event and adds the deep copy to @p events's collection of publish
   * events.
   */
  void add_to_composite(CompositeEventCollection<T>* events) const override {
    DRAKE_DEMAND(events != nullptr);
    events->add_publish_event(
        std::unique_ptr<PublishEvent<T>>(this->DoClone()));
  }

  /**
   * Calls the optional callback function, if one exists, with @p context and
   * `this`.
   */
  void handle(const Context<T>& context) const {
    if (callback_ != nullptr) callback_(context, *this);
  }

 private:
  PublishEvent(const PublishEvent&) = default;

  // Clones PublishEvent-specific data.
  PublishEvent<T>* DoClone() const override { return new PublishEvent(*this); }

  // Optional callback function that handles this publish event.
  PublishCallback callback_{nullptr};
};

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

  /**
   * Makes a DiscreteUpdateEvent with @p trigger_type with no optional data and
   * the callback function @p callback.
   * @p callback can be null.
   */
  DiscreteUpdateEvent(const typename Event<T>::TriggerType& trigger_type,
                      DiscreteUpdateCallback callback)
      : Event<T>(trigger_type), callback_(callback) {}

  /**
   * Makes a DiscreteUpdateEvent with @p trigger_type with no optional data and
   * no specified callback function.
   */
  explicit DiscreteUpdateEvent(
      const typename Event<T>::TriggerType& trigger_type)
      : DiscreteUpdateEvent(trigger_type, nullptr) {}

  /**
   * Assuming that @p events is not null, this function makes a deep copy of
   * this event and adds the deep copy to @p events's collection of discrete
   * update events.
   */
  void add_to_composite(CompositeEventCollection<T>* events) const override {
    DRAKE_DEMAND(events != nullptr);
    events->add_discrete_update_event(
        std::unique_ptr<DiscreteUpdateEvent<T>>(this->DoClone()));
  }

  /**
   * Calls the optional callback function, if one exists, with @p context,
   * 'this' and @p discrete_state.
   */
  void handle(const Context<T>& context,
              DiscreteValues<T>* discrete_state) const {
    if (callback_ != nullptr) callback_(context, *this, discrete_state);
  }

 private:
  DiscreteUpdateEvent(const DiscreteUpdateEvent&) = default;

  // Clones DiscreteUpdateEvent-specific data.
  DiscreteUpdateEvent<T>* DoClone() const override {
    return new DiscreteUpdateEvent(this->get_trigger_type(), callback_);
  }

  // Optional callback function that handles this discrete update event.
  DiscreteUpdateCallback callback_{nullptr};
};

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

  /**
   * Makes an UnrestrictedUpdateEvent with @p trigger_type and callback function
   * @p callback. @p callback can be null.
   */
  UnrestrictedUpdateEvent(const typename Event<T>::TriggerType& trigger_type,
                          UnrestrictedUpdateCallback callback)
      : Event<T>(trigger_type), callback_(callback) {}

  /**
   * Makes an UnrestrictedUpateEvent with @p trigger_type, no optional data, and
   * no callback function.
   */
  explicit UnrestrictedUpdateEvent(
      const typename Event<T>::TriggerType& trigger_type)
      : UnrestrictedUpdateEvent(trigger_type, nullptr) {}

  /**
   * Assuming that @p events is not null, this function makes a deep copy of
   * this event and adds the deep copy to @p events's collection of unrestricted
   * update events.
   */
  void add_to_composite(CompositeEventCollection<T>* events) const override {
    DRAKE_DEMAND(events != nullptr);
    events->add_unrestricted_update_event(
        std::unique_ptr<UnrestrictedUpdateEvent<T>>(this->DoClone()));
  }

  /**
   * Calls the optional callback function, if one exists, with @p context,
   * `this` and @p discrete_state.
   */
  void handle(const Context<T>& context, State<T>* state) const {
    if (callback_ != nullptr) callback_(context, *this, state);
  }

 private:
  UnrestrictedUpdateEvent(const UnrestrictedUpdateEvent&) = default;

  // Clones event data specific to UnrestrictedUpdateEvent.
  UnrestrictedUpdateEvent<T>* DoClone() const override {
    return new UnrestrictedUpdateEvent(*this);
  }

  // Optional callback function that handles this unrestricted update event.
  UnrestrictedUpdateCallback callback_{nullptr};
};

}  // namespace systems
}  // namespace drake
