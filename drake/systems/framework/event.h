#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

// Forward declaration of the event container classes.
// Containers for homogeneous event type.
template <typename EventType>
class EventCollection;
template <typename EventType>
class LeafEventCollection;

// Containers for heterogeneous event types.
template <typename T>
class CompositeEventCollection;
template <typename T>
class LeafCompositeEventCollection;

/**
 * Base class that represents an event. The base event contains two main pieces
 * of information: a reason for event occurrence and additional data that needs
 * to be passed to the event handler. Both are encapsulated by the Trigger
 * class. Derived classes should contain an optional function pointer to the
 * callback function that handles the event.
 */
template <typename T>
class Event {
 public:
   /**
   * Predefined types of triggers. Used at run time to determine why the
   * associated event has occurred. Any user introduced triggers need to have
   * their unique type ids defined here as well.
   */
  enum class TriggerType {
    kUnknown = 0,

    /**
     * This trigger means that any associated event is triggered by a timer.
     */
    kTimed = 2,

    /**
     * This trigger means that any associated event is triggered by a periodic
     * timer.
     */
    kPeriodic = 3,

    /**
     * This trigger means that any associated event is triggered whenever the
     * `solver` takes a `step`. A `solver` can be any code that controls the
     * time and state evolution of a System. The Simulator is an instance of a
     * `solver`, and a `step` in the Simulator case corresponds to its
     * underlying Integrator taking a major time step. Per step events are
     * most commonly triggered in System::GetPerStepEvents(). A very common use
     * of this is to update a discrete or abstract state variable that changes
     * whenever the trajectory advances, such as the "min" or "max" of some
     * quantity, recording of a signal in a delay buffer, or publishing.
     * Another example is to implement feedback controllers interfaced with
     * physical devices. The controller can be implemented in the event handler,
     * and the `step` corresponds to receiving sensor data from the hardware.
     */
    kPerStep = 4,

    kWitness = 5,
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Event)

  virtual ~Event() {}

  /**
   * Clones this instance.
   */
  std::unique_ptr<Event> Clone() const {
    std::unique_ptr<Event> clone = DoClone();
    clone->trigger_type_ = trigger_type_;
    if (data_ != nullptr)
      clone->set_data(data_->Clone());
    return clone;
  }

  /**
   * Derived classes must implement this method.
   */
  virtual std::unique_ptr<Event> DoClone() const = 0;

  /**
   * Returns the trigger type.
   */
  TriggerType get_trigger_type() const { return trigger_type_; }

  /**
   * Sets the trigger type.
   */
  void set_trigger_type(const TriggerType& t) const { trigger_type_ = t; }

  /**
   * Returns a const pointer to the AbstractValue.
   */
  const AbstractValue* get_data() const { return data_.get(); }

  /**
   * Returns a const reference to the underlying data.
   *
   * @throw std::bad_cast if @tparam DataType does not match the underlying
   * data type.
   */
  template <typename DataType>
  const DataType& get_data() const {
    return data_->GetValue<DataType>();
  }

  /**
   * Sets and transfers the ownership of @p data.
   */
  void set_data(std::unique_ptr<AbstractValue> data) {
    data_ = std::move(data);
  }

  /**
   * Adds this to @p events. See derived implementation for more details.
   */
  virtual void add_to_composite(CompositeEventCollection<T>* events) const = 0;

 protected:
  /**
   * Constructs an Event with @p trigger. Ownership of @p trigger is transfered
   * to this. @p trigger cannot be null.
   */
  explicit Event(const TriggerType& trigger)
      : trigger_type_(trigger) {
  }

 private:
  TriggerType trigger_type_;

  // Owned AbstractData.
  std::unique_ptr<AbstractValue> data_{nullptr};
};

/**
 * This class represents a publish event. It has an optional callback function
 * to do custom handling of this event given a const Context and const Trigger.
 */
template <typename T>
class PublishEvent : public Event<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PublishEvent)

  // Callback function that process a publish event.
  typedef std::function<void(const Context<T>&, const typename Event<T>::TriggerType&)>
      PublishCallback;

  /**
   * Makes a Trigger of type @p trigger_type with no optional data, and
   * uses it with @p callback to construct a PublishEvent. @p callback can
   * be null.
   */
  PublishEvent(const typename Event<T>::TriggerType& trigger_type, PublishCallback callback)
      : Event<T>(trigger_type), callback_(callback) {}

  /**
   * Makes a Trigger of type @p trigger_type with no optional data, and
   * uses it to construct a PublishEvent with no specified callback function.
   */
  explicit PublishEvent(const typename Event<T>::TriggerType& trigger_type)
      : Event<T>(trigger_type) {}

  /**
   * Assuming that @p events is not null and is of type LeafEventCollection<T>*,
   * this function makes a deep copy of this event and uses
   * LeafEventCollection::add_event(std::unique_ptr<PublishEvent<T>>) to insert
   * the deep copy into @p events's collection of publish events. This aborts if
   * the assumptions are not met.
   */
  void add_to_composite(CompositeEventCollection<T>* events) const override {
    Event<T>* clone = this->Clone().release();
    PublishEvent<T>* publish_clone = static_cast<PublishEvent<T>*>(clone);
    events->add_publish_event(std::unique_ptr<PublishEvent<T>>(publish_clone));
  }

  /**
   * Clones this instance.
   */
  std::unique_ptr<Event<T>> DoClone() const override {
    PublishEvent* clone = new PublishEvent(this->get_trigger_type(), callback_);
    return std::unique_ptr<Event<T>>(clone);
  }

  /**
   * Optional callback function that handles this publish event.
   */
  PublishCallback callback_{nullptr};
};

/**
 * This class represents a discrete update event. It has an optional callback
 * function to do custom handling of this event given a const Context, a const
 * Trigger and writes the updates to a mutable DiscreteValues.
 */
template <typename T>
class DiscreteUpdateEvent : public Event<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteUpdateEvent)

  // Callback function that process a discrete update event.
  typedef std::function<void(const Context<T>&, const typename Event<T>::TriggerType&,
                             DiscreteValues<T>*)>
      DiscreteUpdateCallback;

  /**
   * Makes a Trigger of type @p trigger_type with no optional data, and uses
   * it with @p callback to construct a DiscreteUpdateEvent. @p callback can
   * be null.
   */
  DiscreteUpdateEvent(const typename Event<T>::TriggerType& trigger_type,
                      DiscreteUpdateCallback callback)
      : Event<T>(trigger_type), callback_(callback) {
  }

  /**
   * Makes a Trigger of type @p trigger_type with no optional data, and uses
   * it to construct a DiscreteUpdateEvent with no specified callback function.
   */
  explicit DiscreteUpdateEvent(const typename Event<T>::TriggerType& trigger_type)
      : DiscreteUpdateEvent(trigger_type, nullptr) {}

  /**
   * Assuming that @p events is not null and is of type LeafEventCollection<T>*,
   * this function makes a deep copy of this event and uses
   * LeafEventCollection::add_event(std::unique_ptr<DiscreteUpdateEvent<T>>)
   * to insert the deep copy into @p events's collection of discrete update
   * events. This aborts if the assumptions are not met.
   */
  void add_to_composite(CompositeEventCollection<T>* events) const override {
    Event<T>* clone = this->Clone().release();
    DiscreteUpdateEvent<T>* du_clone = static_cast<DiscreteUpdateEvent<T>*>(clone);
    events->add_discrete_update_event(std::unique_ptr<DiscreteUpdateEvent<T>>(du_clone));
  }

  /**
   * Clones this instance.
   */
  std::unique_ptr<Event<T>> DoClone() const override {
    DiscreteUpdateEvent* clone =
        new DiscreteUpdateEvent(this->get_trigger_type(), callback_);
    return std::unique_ptr<Event<T>>(clone);
  }

  /**
   * Optional callback function that handles this discrete update event.
   */
  DiscreteUpdateCallback callback_{nullptr};
};

/**
 * This class represents a unrestricted update event. It has an optional
 * callback function to do custom handling of this event given a const Context,
 * a const Trigger and writes the updates to a mutable State.
 */
template <typename T>
class UnrestrictedUpdateEvent : public Event<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnrestrictedUpdateEvent)

  // Callback function that process an unrestricted update event.
  typedef std::function<void(const Context<T>&, const typename Event<T>::TriggerType&, State<T>*)>
      UnrestrictedUpdateCallback;

  /**
   * Makes a Trigger of type @p trigger_type with no optional data, and uses it
   * with @p callback to construct a UnrestrictedUpdateEvent. @p callback can
   * be null.
   */
  UnrestrictedUpdateEvent(const typename Event<T>::TriggerType& trigger_type,
                          UnrestrictedUpdateCallback callback)
      : Event<T>(trigger_type), callback_(callback) {}

  /**
   * Makes a Trigger of type @p trigger_type with no optional data, and uses it
   * to construct a UnrestrictedUpdateEvent with no specified callback function.
   */
  explicit UnrestrictedUpdateEvent(const typename Event<T>::TriggerType& trigger_type)
      : UnrestrictedUpdateEvent(trigger_type, nullptr) {}

  /**
   * Assuming that @p events is not null and is of type LeafEventCollection<T>*,
   * this function makes a deep copy of this event and uses
   * LeafEventCollection::add_event(std::unique_ptr<UnrestrictedUpdateEvent<T>>)
   * to insert the deep copy into @p events's collection of unrestricted update
   * events. This aborts if the assumptions are not met.
   */
  void add_to_composite(CompositeEventCollection<T>* events) const override {
    Event<T>* clone = this->Clone().release();
    UnrestrictedUpdateEvent<T>* uu_clone = static_cast<UnrestrictedUpdateEvent<T>*>(clone);
    events->add_unrestricted_update_event(std::unique_ptr<UnrestrictedUpdateEvent<T>>(uu_clone));
  }

  /**
   * Clones this instance.
   */
  std::unique_ptr<Event<T>> DoClone() const override {
    UnrestrictedUpdateEvent* clone =
        new UnrestrictedUpdateEvent(this->get_trigger_type(), callback_);
    return std::unique_ptr<Event<T>>(clone);
  }

  /**
   * Optional callback function that handles this unrestricted update event.
   */
  UnrestrictedUpdateCallback callback_{nullptr};
};

}  // namespace systems
}  // namespace drake
