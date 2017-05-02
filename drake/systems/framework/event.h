#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/trigger.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

// Forward declaration of event container classes.
class EventInfo;
template <typename T> class LeafEventInfo;

/**
 * Base class that represents an event. The base event contains two main pieces
 * of information: a reason for event occurrence and additional data that needs
 * to be passed to the event handler. Both are encapsulated by the Trigger
 * class.
 */
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
   * Adds this to @p events. See derived implementation for more details.
   */
  virtual void add_to(EventInfo* events) const = 0;

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

template <typename T>
class PublishEvent : public Event {
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
      : Event(std::move(trigger)), Handle(callback) {}

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
  PublishEvent(Trigger::TriggerType trigger_type)
      : PublishEvent(trigger_type, nullptr) {}

  /**
   * Constructs a PublishEvent with Trigger @p trigger and no specified callback
   * function. @p trigger cannot be null.
   */
  PublishEvent(std::unique_ptr<Trigger> trigger)
      : PublishEvent(std::move(trigger), nullptr) {}

  /**
   * Assuming that @p events is not null and is of type LeafEventInfo<T>*,
   * this function makes a deep copy of this event and uses
   * LeafEventInfo::add_event(std::unique_ptr<PublishEvent<T>>) to insert the
   * deep copy into @p events's collection of publish events. This aborts if
   * the assumptions are not met.
   */
  void add_to(EventInfo* events) const override {
    LeafEventInfo<T>* leaf_events = dynamic_cast<LeafEventInfo<T>*>(events);
    DRAKE_DEMAND(leaf_events != nullptr);

    auto me = std::make_unique<PublishEvent<T>>(get_trigger().Clone(), Handle);
    leaf_events->add_event(std::move(me));
  }

  /**
   * Clones this instance.
   */
  std::unique_ptr<Event> Clone() const override {
    PublishEvent* clone = new PublishEvent(get_trigger().Clone(), Handle);
    return std::unique_ptr<Event>(clone);
  }

  PublishCallback Handle{nullptr};
};

template <typename T>
class DiscreteUpdateEvent : public Event {
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
      : Event(std::move(trigger)), Handle(callback) {}

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
  DiscreteUpdateEvent(Trigger::TriggerType trigger_type)
      : DiscreteUpdateEvent(trigger_type, nullptr) {}

  /**
   * Constructs a DiscreteUpdateEvent with Trigger @p trigger and no specified
   * callback function. @p trigger cannot be null.
   */
  DiscreteUpdateEvent(std::unique_ptr<Trigger> trigger)
      : DiscreteUpdateEvent(std::move(trigger), nullptr) {}

  /**
   * Assuming that @p events is not null and is of type LeafEventInfo<T>*,
   * this function makes a deep copy of this event and uses
   * LeafEventInfo::add_event(std::unique_ptr<DiscreteUpdateEvent<T>>)
   * to insert the deep copy into @p events's collection of discrete update
   * events. This aborts if the assumptions are not met.
   */
  void add_to(EventInfo* events) const override {
    LeafEventInfo<T>* leaf_events = dynamic_cast<LeafEventInfo<T>*>(events);
    DRAKE_DEMAND(leaf_events != nullptr);

    auto me =
        std::make_unique<DiscreteUpdateEvent<T>>(get_trigger().Clone(), Handle);
    leaf_events->add_event(std::move(me));
  }

  /**
   * Clones this instance.
   */
  std::unique_ptr<Event> Clone() const override {
    DiscreteUpdateEvent* clone =
        new DiscreteUpdateEvent(get_trigger().Clone(), Handle);
    return std::unique_ptr<Event>(clone);
  }

  DiscreteUpdateCallback Handle{nullptr};
};

template <typename T>
class UnrestrictedUpdateEvent : public Event {
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
      : Event(std::move(trigger)), Handle(callback) {}

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
  UnrestrictedUpdateEvent(Trigger::TriggerType trigger_type)
      : UnrestrictedUpdateEvent(trigger_type, nullptr) {}

  /**
   * Constructs a UnrestrictedUpdateEvent with Trigger @p trigger and no
   * specified callback function. @p trigger cannot be null.
   */
  UnrestrictedUpdateEvent(std::unique_ptr<Trigger> trigger)
      : UnrestrictedUpdateEvent(std::move(trigger), nullptr) {}

  /**
   * Assuming that @p events is not null and is of type LeafEventInfo<T>*,
   * this function makes a deep copy of this event and uses
   * LeafEventInfo::add_event(std::unique_ptr<UnrestrictedUpdateEvent<T>>)
   * to insert the deep copy into @p events's collection of unrestricted update
   * events. This aborts if the assumptions are not met.
   */
  void add_to(EventInfo* events) const override {
    LeafEventInfo<T>* leaf_events = dynamic_cast<LeafEventInfo<T>*>(events);
    DRAKE_DEMAND(leaf_events != nullptr);

    auto me = std::make_unique<UnrestrictedUpdateEvent<T>>(
        get_trigger().Clone(), Handle);
    leaf_events->add_event(std::move(me));
  }

  /**
   * Clones this instance.
   */
  std::unique_ptr<Event> Clone() const override {
    UnrestrictedUpdateEvent* clone =
        new UnrestrictedUpdateEvent(get_trigger().Clone(), Handle);
    return std::unique_ptr<Event>(clone);
  }

  UnrestrictedUpdateCallback Handle{nullptr};
};

}  // namespace systems
}  // namespace drake
