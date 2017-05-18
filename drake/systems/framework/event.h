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
 * main pieces of information: a reason for the event occurring (i.e., a
 * "trigger") and additional data that can be passed from the System to its
 * event handler (to mitigate or eliminate recomputation). The reason consists
 * of a enum trigger type and an optional attribute of AbstractValue. Derived
 * classes should contain a function pointer to an optional callback function
 * for handling the event.
 */
template <typename T>
class Event {
 public:
  /**
   * Predefined types of triggers. Used at run time to determine why the
   * associated event has occurred.
   */
  enum class TriggerType {
    kUnknown,

    /**
     * This trigger indicates that an associated event is triggered by directly
     * calling the corresponding public system API for event handling (e.g.
     * Publish(context)).
     */
    kForced,

    /**
     * This trigger indicates that an associated event is triggered by the
     * system proceeding to a particular time.
     */
    kTimed,

    /**
     * This trigger indicates that an associated event is triggered by the
     * system proceeding to a time that recurs periodically.
     */
    kPeriodic,

    /**
     * This trigger indicates that an associated event is triggered whenever a
     * "solver" takes a "step". A `solver` is an abstract construct that
     * controls the time and state evolution of a System. Simulator is an
     * "solver", and a "step" in the Simulator corresponds event handling
     * and integration (over some finite time) of a system of ordinary
     * differential equations. Per-step-events are most commonly created in
     * System::GetPerStepEvents(). A very common use of such per-step-events is
     * to update a discrete or abstract state variable that changes
     * whenever the continuous state advances; examples are computing the "min"
     * or "max" of some state variable, recording a signal in a delay buffer, or
     * publishing. Per-step-events are also useful to implement feedback
     * controllers interfaced with physical devices; the controller can be
     * implemented in the event handler, and the "step" would correspond to
     * receiving sensory data from the hardware.
     */
    kPerStep,

    /**
     * This trigger indicates that an associated event is triggered by a witness
     * function.
     */
    kWitness,
  };

  /**
   * A token describing an event that recurs on a fixed period.
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

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Event)

  virtual ~Event() {}

  /**
   * Clones this instance.
   */
  std::unique_ptr<Event> Clone() const {
    std::unique_ptr<Event> clone(DoClone());
    clone->trigger_type_ = trigger_type_;
    if (data_ != nullptr) clone->set_data(data_->Clone());
    if (attribute_ != nullptr) clone->set_attribute(attribute_->Clone());
    return clone;
  }

  /**
   * Returns the trigger type.
   */
  TriggerType get_trigger_type() const { return trigger_type_; }

  /**
   * Returns a const pointer to the AbstractValue data.
   */
  const AbstractValue* get_data() const { return data_.get(); }

  /**
   * Returns a const reference to the underlying data.
   *
   * @throws std::runtime_error if there is no underlying data.
   * @throws std::bad_cast if @tparam DataType does not match the underlying
   *         data type.
   */
  template <typename DataType>
  const DataType& get_data() const {
    if (data_ == nullptr)
      throw std::runtime_error("Data is null.");
    return data_->GetValue<DataType>();
  }

  /**
   * Sets and transfers the ownership of @p data.
   */
  void set_data(std::unique_ptr<AbstractValue> data) {
    data_ = std::move(data);
  }

  /**
   * Returns a const pointer to the AbstractValue attribute.
   */
  const AbstractValue* get_attribute() const { return attribute_.get(); }

  /**
   * Returns a const reference to the underlying attribute.
   *
   * @throws std::runtime_error if there is no underlying attribute.
   * @throws std::bad_cast if @tparam DataType does not match the underlying
   *         attribute type.
   */
  template <typename DataType>
  const DataType& get_attribute() const {
    if (attribute_ == nullptr)
      throw std::runtime_error("Data is null.");
    return attribute_->GetValue<DataType>();
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
  TriggerType trigger_type_;
  std::unique_ptr<AbstractValue> attribute_{nullptr};

  // Owned AbstractData.
  std::unique_ptr<AbstractValue> data_{nullptr};
};

/**
 * This class represents a publish event. It has an optional callback function
 * to do custom handling of this event given const Context and const
 * PublishEvent object references.
 */
template <typename T>
class PublishEvent : public Event<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PublishEvent)

  // Callback function that process a publish event.
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
    Event<T>* clone = this->Clone().release();
    PublishEvent<T>* publish_clone = static_cast<PublishEvent<T>*>(clone);
    events->add_publish_event(std::unique_ptr<PublishEvent<T>>(publish_clone));
  }

  /**
   * Optional callback function that handles this publish event.
   */
  PublishCallback callback_{nullptr};

 private:
  /**
   * Clones PublishEvent-specific data.
   */
  PublishEvent<T>* DoClone() const override {
    return new PublishEvent(this->get_trigger_type(), callback_);
  }
};

/**
 * This class represents a discrete update event. It has an optional callback
 * function to do custom handling of this event given const Context and
 * const DiscreteUpdateEvent references, and writes the updates to a mutable,
 * non-null DiscreteValues object.
 */
template <typename T>
class DiscreteUpdateEvent : public Event<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteUpdateEvent)

  // Callback function that process a discrete update event.
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
    Event<T>* clone = this->Clone().release();
    DiscreteUpdateEvent<T>* du_clone =
        static_cast<DiscreteUpdateEvent<T>*>(clone);
    events->add_discrete_update_event(
        std::unique_ptr<DiscreteUpdateEvent<T>>(du_clone));
  }

  /**
   * Optional callback function that handles this discrete update event.
   */
  DiscreteUpdateCallback callback_{nullptr};

 private:
  /**
   * Clones DiscreteUpdateEvent-specific data.
   */
  DiscreteUpdateEvent<T>* DoClone() const override {
    return new DiscreteUpdateEvent(this->get_trigger_type(), callback_);
  }
};

/**
 * This class represents an unrestricted update event. It has an optional
 * callback function to do custom handling of this event given const Context and
 * const UnrestrictedUpdateEvent object references, and writes the updates to a
 * mutable, non-null State object.
 */
template <typename T>
class UnrestrictedUpdateEvent : public Event<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnrestrictedUpdateEvent)

  // Callback function that process an unrestricted update event.
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
    Event<T>* clone = this->Clone().release();
    UnrestrictedUpdateEvent<T>* uu_clone =
        static_cast<UnrestrictedUpdateEvent<T>*>(clone);
    events->add_unrestricted_update_event(
        std::unique_ptr<UnrestrictedUpdateEvent<T>>(uu_clone));
  }

  /**
   * Optional callback function that handles this unrestricted update event.
   */
  UnrestrictedUpdateCallback callback_{nullptr};

 private:
  /**
   * Clones event data specific to UnrestrictedUpdateEvent.
   */
  UnrestrictedUpdateEvent<T>* DoClone() const override {
    return new UnrestrictedUpdateEvent(this->get_trigger_type(), callback_);
  }
};

}  // namespace systems
}  // namespace drake
