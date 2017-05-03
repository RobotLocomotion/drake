#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/trigger.h"
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
class CombinedEventCollection;
template <typename T>
class LeafCombinedEventCollection;

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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Event)

  virtual ~Event() {}

  /**
   * Clones this instance.
   */
  virtual std::unique_ptr<Event> Clone() const = 0;

  /**
   * Returns a const reference to the trigger.
   */
  const Trigger& get_trigger() const { return *trigger_; }

  /**
   * Returns a mutable pointer to the trigger.
   */
  Trigger* get_mutable_trigger() { return trigger_.get(); }

  /**
   * Adds this to @p events. See derived implementation for more details.
   */
  virtual void add_to_combined(CombinedEventCollection<T>* events) const = 0;

 protected:
  /**
   * Constructs an Event with @p trigger. Ownership of @p trigger is transfered
   * to this. @p trigger cannot be null.
   */
  explicit Event(std::unique_ptr<Trigger> trigger)
      : trigger_(std::move(trigger)) {
    DRAKE_DEMAND(trigger_ != nullptr);
  }

 private:
  // Owned trigger.
  std::unique_ptr<Trigger> trigger_;
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
  typedef std::function<void(const Context<T>&, const Trigger&)>
      PublishCallback;

  /**
   * Constructs a PublishEvent with Trigger @p trigger and callback function
   * @p callback. Ownership of @p trigger is transfered to this. @p callback
   * can be null, but @p trigger cannot be null.
   */
  PublishEvent(std::unique_ptr<Trigger> trigger, PublishCallback callback)
      : Event<T>(std::move(trigger)), Handle(callback) {}

  /**
   * Makes a Trigger of type @p trigger_type with no optional data, and
   * uses it with @p callback to construct a PublishEvent. @p callback can
   * be null.
   */
  PublishEvent(Trigger::TriggerType trigger_type, PublishCallback callback)
      : PublishEvent(std::make_unique<Trigger>(trigger_type), callback) {}

  /**
   * Makes a Trigger of type @p trigger_type with no optional data, and
   * uses it to construct a PublishEvent with no specified callback function.
   */
  explicit PublishEvent(Trigger::TriggerType trigger_type)
      : PublishEvent(trigger_type, nullptr) {}

  /**
   * Constructs a PublishEvent with Trigger @p trigger and no specified callback
   * function. @p trigger cannot be null.
   */
  explicit PublishEvent(std::unique_ptr<Trigger> trigger)
      : PublishEvent(std::move(trigger), nullptr) {}

  /**
   * Assuming that @p events is not null and is of type LeafEventCollection<T>*,
   * this function makes a deep copy of this event and uses
   * LeafEventCollection::add_event(std::unique_ptr<PublishEvent<T>>) to insert
   * the deep copy into @p events's collection of publish events. This aborts if
   * the assumptions are not met.
   */
  void add_to_combined(CombinedEventCollection<T>* events) const override {
    LeafCombinedEventCollection<T>* leaf_events =
        dynamic_cast<LeafCombinedEventCollection<T>*>(events);
    DRAKE_DEMAND(leaf_events != nullptr);

    auto me =
        std::make_unique<PublishEvent<T>>(this->get_trigger().Clone(), Handle);
    leaf_events->add_publish_event(std::move(me));
  }

  /**
   * Clones this instance.
   */
  std::unique_ptr<Event<T>> Clone() const override {
    PublishEvent* clone = new PublishEvent(this->get_trigger().Clone(), Handle);
    return std::unique_ptr<Event<T>>(clone);
  }

  /**
   * Optional callback function that handles this publish event.
   */
  PublishCallback Handle{nullptr};
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
  typedef std::function<void(const Context<T>&, const Trigger&,
                             DiscreteValues<T>*)>
      DiscreteUpdateCallback;

  /**
   * Constructs a DiscreteUpdateEvent with Trigger @p trigger and callback
   * function @p callback. Ownership of @p trigger is transfered to this.
   * @p callback can be null, but @p trigger cannot be null.
   */
  DiscreteUpdateEvent(std::unique_ptr<Trigger> trigger,
                      DiscreteUpdateCallback callback)
      : Event<T>(std::move(trigger)), Handle(callback) {}

  /**
   * Makes a Trigger of type @p trigger_type with no optional data, and uses
   * it with @p callback to construct a DiscreteUpdateEvent. @p callback can
   * be null.
   */
  DiscreteUpdateEvent(Trigger::TriggerType trigger_type,
                      DiscreteUpdateCallback callback)
      : DiscreteUpdateEvent(std::make_unique<Trigger>(trigger_type), callback) {
  }

  /**
   * Makes a Trigger of type @p trigger_type with no optional data, and uses
   * it to construct a DiscreteUpdateEvent with no specified callback function.
   */
  explicit DiscreteUpdateEvent(Trigger::TriggerType trigger_type)
      : DiscreteUpdateEvent(trigger_type, nullptr) {}

  /**
   * Constructs a DiscreteUpdateEvent with Trigger @p trigger and no specified
   * callback function. @p trigger cannot be null.
   */
  explicit DiscreteUpdateEvent(std::unique_ptr<Trigger> trigger)
      : DiscreteUpdateEvent(std::move(trigger), nullptr) {}

  /**
   * Assuming that @p events is not null and is of type LeafEventCollection<T>*,
   * this function makes a deep copy of this event and uses
   * LeafEventCollection::add_event(std::unique_ptr<DiscreteUpdateEvent<T>>)
   * to insert the deep copy into @p events's collection of discrete update
   * events. This aborts if the assumptions are not met.
   */
  void add_to_combined(CombinedEventCollection<T>* events) const override {
    LeafCombinedEventCollection<T>* leaf_events =
        dynamic_cast<LeafCombinedEventCollection<T>*>(events);
    DRAKE_DEMAND(leaf_events != nullptr);

    auto me = std::make_unique<DiscreteUpdateEvent<T>>(
        this->get_trigger().Clone(), Handle);
    leaf_events->add_discrete_update_event(std::move(me));
  }

  /**
   * Clones this instance.
   */
  std::unique_ptr<Event<T>> Clone() const override {
    DiscreteUpdateEvent* clone =
        new DiscreteUpdateEvent(this->get_trigger().Clone(), Handle);
    return std::unique_ptr<Event<T>>(clone);
  }

  /**
   * Optional callback function that handles this discrete update event.
   */
  DiscreteUpdateCallback Handle{nullptr};
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
  typedef std::function<void(const Context<T>&, const Trigger&, State<T>*)>
      UnrestrictedUpdateCallback;

  /**
   * Constructs a UnrestrictedUpdateEvent with Trigger @p trigger and callback
   * function @p callback. Ownership of @p trigger is transfered to this.
   * @p callback can be null, but @p trigger cannot be null.
   */
  UnrestrictedUpdateEvent(std::unique_ptr<Trigger> trigger,
                          UnrestrictedUpdateCallback callback)
      : Event<T>(std::move(trigger)), Handle(callback) {}

  /**
   * Makes a Trigger of type @p trigger_type with no optional data, and uses it
   * with @p callback to construct a UnrestrictedUpdateEvent. @p callback can
   * be null.
   */
  UnrestrictedUpdateEvent(Trigger::TriggerType trigger_type,
                          UnrestrictedUpdateCallback callback)
      : UnrestrictedUpdateEvent(std::make_unique<Trigger>(trigger_type),
                                callback) {}

  /**
   * Makes a Trigger of type @p trigger_type with no optional data, and uses it
   * to construct a UnrestrictedUpdateEvent with no specified callback function.
   */
  explicit UnrestrictedUpdateEvent(Trigger::TriggerType trigger_type)
      : UnrestrictedUpdateEvent(trigger_type, nullptr) {}

  /**
   * Constructs a UnrestrictedUpdateEvent with Trigger @p trigger and no
   * specified callback function. @p trigger cannot be null.
   */
  explicit UnrestrictedUpdateEvent(std::unique_ptr<Trigger> trigger)
      : UnrestrictedUpdateEvent(std::move(trigger), nullptr) {}

  /**
   * Assuming that @p events is not null and is of type LeafEventCollection<T>*,
   * this function makes a deep copy of this event and uses
   * LeafEventCollection::add_event(std::unique_ptr<UnrestrictedUpdateEvent<T>>)
   * to insert the deep copy into @p events's collection of unrestricted update
   * events. This aborts if the assumptions are not met.
   */
  void add_to_combined(CombinedEventCollection<T>* events) const override {
    LeafCombinedEventCollection<T>* leaf_events =
        dynamic_cast<LeafCombinedEventCollection<T>*>(events);
    DRAKE_DEMAND(leaf_events != nullptr);

    auto me = std::make_unique<UnrestrictedUpdateEvent<T>>(
        this->get_trigger().Clone(), Handle);
    leaf_events->add_unrestricted_update_event(std::move(me));
  }

  /**
   * Clones this instance.
   */
  std::unique_ptr<Event<T>> Clone() const override {
    UnrestrictedUpdateEvent* clone =
        new UnrestrictedUpdateEvent(this->get_trigger().Clone(), Handle);
    return std::unique_ptr<Event<T>>(clone);
  }

  /**
   * Optional callback function that handles this unrestricted update event.
   */
  UnrestrictedUpdateCallback Handle{nullptr};
};

}  // namespace systems
}  // namespace drake
