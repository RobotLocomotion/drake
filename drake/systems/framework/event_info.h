#pragma once

#include <map>
#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/trigger.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/state.h"

namespace drake {
namespace systems {

class EventInfo;
template <typename T> class LeafEventInfo;

class Event {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Event)

  virtual ~Event() {}

  std::unique_ptr<Event> Clone() const {
    std::unique_ptr<Event> clone = this->DoClone();
    return clone;
  }

  const Trigger& get_trigger() const { return *trigger_; }

  virtual void add_to(EventInfo* events) const = 0;

 protected:
  explicit Event(std::unique_ptr<Trigger> trigger) : trigger_(std::move(trigger)) {}

  virtual std::unique_ptr<Event> DoClone() const = 0;

 private:
  std::unique_ptr<Trigger> trigger_;
};

template <typename T>
class PublishEvent : public Event {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PublishEvent)
  typedef std::function<void(const Context<T>&, const Trigger&)> PublishCallback;

  PublishEvent(std::unique_ptr<Trigger> trigger, PublishCallback callback)
      : Event(std::move(trigger)), Handle(callback) {}

  PublishEvent(Trigger::TriggerType trigger_type, PublishCallback callback)
      : PublishEvent(std::make_unique<Trigger>(trigger_type), callback) {}

  PublishEvent(Trigger::TriggerType trigger_type)
      : PublishEvent(trigger_type, nullptr) {}

  PublishEvent(std::unique_ptr<Trigger> trigger)
      : PublishEvent(std::move(trigger), nullptr) {}

  PublishCallback Handle{nullptr};

  void add_to(EventInfo* events) const override {
    LeafEventInfo<T>* leaf_events = dynamic_cast<LeafEventInfo<T>*>(events);
    DRAKE_DEMAND(leaf_events != nullptr);

    auto me = std::make_unique<PublishEvent<T>>(get_trigger().Clone(), Handle);
    leaf_events->add_event(std::move(me));
  }

 private:
  std::unique_ptr<Event> DoClone() const override {
    PublishEvent* clone = new PublishEvent(get_trigger().Clone(), Handle);
    return std::unique_ptr<Event>(clone);
  }
};

template <typename T>
class DiscreteUpdateEvent : public Event {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteUpdateEvent)
  typedef std::function<void(const Context<T>&, const Trigger&, DiscreteValues<T>*)> DiscreteUpdateCallback;

  DiscreteUpdateEvent(std::unique_ptr<Trigger> trigger, DiscreteUpdateCallback callback)
      : Event(std::move(trigger)), Handle(callback) {}

  DiscreteUpdateEvent(Trigger::TriggerType trigger_type, DiscreteUpdateCallback callback)
      : DiscreteUpdateEvent(std::make_unique<Trigger>(trigger_type), callback) {}

  DiscreteUpdateEvent(Trigger::TriggerType trigger_type)
      : DiscreteUpdateEvent(trigger_type, nullptr) {}

  DiscreteUpdateEvent(std::unique_ptr<Trigger> trigger)
      : DiscreteUpdateEvent(std::move(trigger), nullptr) {}

  DiscreteUpdateCallback Handle{nullptr};

  void add_to(EventInfo* events) const override {
    LeafEventInfo<T>* leaf_events = dynamic_cast<LeafEventInfo<T>*>(events);
    DRAKE_DEMAND(leaf_events != nullptr);

    auto me = std::make_unique<DiscreteUpdateEvent<T>>(get_trigger().Clone(), Handle);
    leaf_events->add_event(std::move(me));
  }

 private:
  std::unique_ptr<Event> DoClone() const override {
    DiscreteUpdateEvent* clone = new DiscreteUpdateEvent(get_trigger().Clone(), Handle);
    return std::unique_ptr<Event>(clone);
  }
};

template <typename T>
class UnrestrictedUpdateEvent : public Event {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnrestrictedUpdateEvent)
  typedef std::function<void(const Context<T>&, const Trigger&, State<T>*)> UnrestrictedUpdateCallback;

  UnrestrictedUpdateEvent(std::unique_ptr<Trigger> trigger, UnrestrictedUpdateCallback callback)
      : Event(std::move(trigger)), Handle(callback) {}

  UnrestrictedUpdateEvent(Trigger::TriggerType trigger_type, UnrestrictedUpdateCallback callback)
      : UnrestrictedUpdateEvent(std::make_unique<Trigger>(trigger_type), callback) {}

  UnrestrictedUpdateEvent(Trigger::TriggerType trigger_type)
      : UnrestrictedUpdateEvent(trigger_type, nullptr) {}

  UnrestrictedUpdateEvent(std::unique_ptr<Trigger> trigger)
      : UnrestrictedUpdateEvent(std::move(trigger), nullptr) {}

  UnrestrictedUpdateCallback Handle{nullptr};

  void add_to(EventInfo* events) const override {
    LeafEventInfo<T>* leaf_events = dynamic_cast<LeafEventInfo<T>*>(events);
    DRAKE_DEMAND(leaf_events != nullptr);

    auto me = std::make_unique<UnrestrictedUpdateEvent<T>>(get_trigger().Clone(), Handle);
    leaf_events->add_event(std::move(me));
  }

 private:
  std::unique_ptr<Event> DoClone() const override  {
    UnrestrictedUpdateEvent* clone = new UnrestrictedUpdateEvent(get_trigger().Clone(), Handle);
    return std::unique_ptr<Event>(clone);
  }
};

/**
 * Base class that represents all events at a particular time for System.
 * There are several predefined event and trigger types. To represent a concrete
 * event, a pair of event type and Trigger is necessary. The former specifies
 * the event type, and the latter captures all the details of the actual event.
 *
 * For each event type, the System API provides a unique customizable function
 * for handling all the simultaneous events of that type, such as
 * System::DoPublish(context, triggers), where `triggers` represents the
 * individual events that should be considered to have occurred simultaneously.
 * The user is responsible for overriding such functions to handle each event
 * in the desired order. For example, suppose publish type events are being
 * handled, which are represented by
 * `triggers` = {kPerStep, kPeriodic}. Depending on the desired behavior, the
 * user has the freedom to ignore both, handle only one, or both in any
 * arbitrary order. For each type of events at any given moment in time, its
 * handler should only be invoked once.
 *
 * The System API also provides several functions for customizable event
 * generation such as System::DoCalcNextUpdateTime() or
 * System::DoGetPerStepEvents(). These functions can generate any number of
 * events of arbitrary types, and the resulting events are stored in separate
 * EventInfo instances. Before calling the event handlers, all these EventInfo
 * objects need to be merged, so that the handlers have a complete set of
 * the simultaneous events.
 *
 * Here is a complete example. For some LeafSystem `sys` at time `t`,
 * its System::DoCalcNextUpdateTime() generates the following
 * events (`event_info1`):
 * <pre>
 *   kPublish: {trigger1(kPeriodic)}
 *   kDiscreteUpdate: {trigger2(kPeriodic)}
 * </pre>
 * It also has per step events (`event_info2`) generated by its
 * System::DoGetPerStepEvents():
 * <pre>
 *   kPublish: {trigger3(kPerStep)}
 *   kUnrestrictedUpdate: {trigger4(kPerStep)}
 * </pre>
 * Simultaneous `event_info1` and `event_info2` are then merged into
 * `all_event_info`:
 * <pre>
 *   kPublish: {trigger1, trigger3}
 *   kDiscreteUpdate: {trigger2}
 *   kUnrestrictedUpdate: {trigger4}
 * </pre>
 *
 * To handle these events:
 * <pre>
 *   sys.CalcUnrestrictedUpdate(context, all_event_info, state);
 *   sys.CalcDiscreteVariableUpdates(context, all_event_info, discrete_state);
 *   sys.Publish(context, all_event_info)
 * </pre>
 * For a LeafSystem, this is equivalent to:
 * <pre>
 *   sys.DoCalcUnrestrictedUpdate(context, {trigger4}, state);
 *   sys.DoCalcDiscreteVariableUpdates(context, {trigger2}, discrete_state);
 *   sys.DoPublish(context, {trigger1, trigger3})
 * </pre>
 */
class EventInfo {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EventInfo)

  virtual ~EventInfo() {}

  void SetFrom(const EventInfo& other) {
    this->Clear();
    this->Merge(other);
  }

  /**
   * Merges @p other's event information into this. See derived DoMerge() for
   * more details.
   */
  void Merge(const EventInfo& other) {
    if (&other == this) return;
    DoMerge(&other);
  }

  /**
   * Clears all the triggers associated with all the event types.
   */
  virtual void Clear() = 0;

  /**
   * Returns true if an event of @p event_type exists.
   */
  virtual bool HasPublishEvents() const = 0;

  virtual bool HasDiscreteUpdateEvents() const = 0;

  virtual bool HasUnrestrictedUpdateEvents() const = 0;

  /**
   * Returns true if no event exists.
   */
  virtual bool HasNoEvents() const = 0;

  virtual void print() const = 0;

 protected:
  /**
   * Constructor only accessible by derived class.
   */
  EventInfo() = default;

  /**
   * Derived implementation can assume that @p is not null, and it is does not
   * equal to this.
   */
  virtual void DoMerge(const EventInfo* other) = 0;
};

/**
 * A concrete class that holds event related information for a Diagram.
 * For each sub system in the corresponding Diagram, a derived EventInfo
 * instance is maintained internally.
 */
class DiagramEventInfo final : public EventInfo {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramEventInfo)

  /**
   * Constructor. Note that this constructor only resizes the containers, but
   * does not allocate any derived EventInfo instances.
   *
   * @note Users should almost never call this explicitly. Use
   * System::AllocateEventInfo() instead.
   *
   * @param num_sub_systems Number of sub systems in the corresponding Diagram.
   */
  explicit DiagramEventInfo(int num_sub_systems)
      : EventInfo(),
        sub_event_info_(num_sub_systems),
        owned_sub_event_info_(num_sub_systems) {}

  /**
   * Returns the number of constituent EventInfo that correspond to each sub
   * system.
   */
  int num_sub_event_info() const {
    return static_cast<int>(sub_event_info_.size());
  }

  /**
   * Transfers @p sub_event_info ownership to this, and associate it with sub
   * system identified by @p index.
   */
  void set_and_own_sub_event_info(int index,
                                  std::unique_ptr<EventInfo> sub_event_info);

  /**
   * Returns a const pointer to sub system's EventInfo at @p index.
   */
  const EventInfo* get_sub_event_info(int index) const;

  /**
   * Returns a mutable pointer to sub system's EventInfo at @p index.
   */
  EventInfo* get_mutable_sub_event_info(int index);

  /**
   * Goes through each sub event info and clears its content.
   */
  void Clear() override;

  /**
   * Returns true if any sub event info contains @p event_type.
   */
  bool HasPublishEvents() const override;

  bool HasDiscreteUpdateEvents() const override;

  bool HasUnrestrictedUpdateEvents() const override;

  /**
   * Returns true if all sub event info are empty.
   */
  bool HasNoEvents() const override;

  void print() const override {
    int ctr = 0;
    for (const EventInfo* info : sub_event_info_) {
      std::cout << "sub system " << ctr << ":\n\t";
      info->print();
      std::cout << "\n";
      ctr++;
    }
  }

 protected:
  // These are protected for doxygen.

  /**
   * Goes through each sub event info and merges in the corresponding one in
   * @p other_info. Assumes that @p other_info is an instance of
   * DiagramEventInfo and has the same number of sub event info. Aborts
   * otherwise.
   */
  void DoMerge(const EventInfo* other_info) override;

 private:
  std::vector<EventInfo*> sub_event_info_;
  std::vector<std::unique_ptr<EventInfo>> owned_sub_event_info_;

  template <typename T> friend class Diagram;
};

/**
 * A concrete class that holds event related information for a LeafSystem.
 * This class is essentially a map from EventType to a list of active Triggers.
 * <pre>
 *   event_type1: {trigger1, trigger2, ...}
 *   event_type2: {trigger3, trigger4, ...}
 *   ...
 * </pre>
 * Unique event type is assumed, however multiple events of the same type
 * occurring simultaneously can be represented by associating multiple
 * Triggers with that event type.
 */
template <typename T>
class LeafEventInfo final : public EventInfo {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeafEventInfo)

  /**
   * Constructor.
   */
  LeafEventInfo() = default;

  /**
   * Returns all the triggers that are associated to @p event_type. Aborts if
   * @p event_type does not exist.
   */
  const std::vector<const PublishEvent<T>*>& get_publish_events() const {
    return publish_events_;
  }

  const std::vector<const DiscreteUpdateEvent<T>*>& get_discrete_update_events() const {
    return discrete_update_events_;
  }

  const std::vector<const UnrestrictedUpdateEvent<T>*>& get_unrestricted_update_events() const {
    return unrestricted_update_events_;
  }

  void add_event(std::unique_ptr<PublishEvent<T>> event) {
    owned_publish_events_.push_back(std::move(event));
    publish_events_.push_back(owned_publish_events_.back().get());
  }

  void add_event(std::unique_ptr<DiscreteUpdateEvent<T>> event) {
    owned_discrete_update_events_.push_back(std::move(event));
    discrete_update_events_.push_back(owned_discrete_update_events_.back().get());
  }

  void add_event(std::unique_ptr<UnrestrictedUpdateEvent<T>> event) {
    owned_unrestricted_update_events_.push_back(std::move(event));
    unrestricted_update_events_.push_back(owned_unrestricted_update_events_.back().get());
  }

  bool HasPublishEvents() const override {
    return !publish_events_.empty();
  }

  bool HasDiscreteUpdateEvents() const override {
    return !discrete_update_events_.empty();
  }

  bool HasUnrestrictedUpdateEvents() const override {
    return !unrestricted_update_events_.empty();
  }

  /**
   * Returns true if no event exists.
   */
  bool HasNoEvents() const override {
    return (publish_events_.empty() && discrete_update_events_.empty() && unrestricted_update_events_.empty());
  }

  /**
   * Clears all events.
   */
  void Clear() override {
    owned_publish_events_.clear();
    owned_discrete_update_events_.clear();
    owned_unrestricted_update_events_.clear();
    publish_events_.clear();
    discrete_update_events_.clear();
    unrestricted_update_events_.clear();
  }

  void print() const override {
    std::cout << "pub size: " << publish_events_.size() <<
        ", discrete size: " << discrete_update_events_.size() <<
        ", unrestricted size: " << unrestricted_update_events_.size() << std::endl;
  }

 protected:
  // These are protected for doxygen.

  /**
   * For each event type in @p other_info, adds all its associated triggers to
   * this. Assumes that @p other_info is an instance of LeafEventInfo. Aborts
   * otherwise.
   *
   * Here is an example. Suppose this has the following event types and their
   * associated triggers:
   * <pre>
   *   event_type1: {trigger1, trigger2, trigger3}
   *   event_type2: {trigger4, trigger5}
   * </pre>
   * @p other_info has:
   * <pre>
   *   event_type1: {trigger6}
   *   event_type3: {trigger7, trigger8}
   * </pre>
   * After calling DoMerge(other_info), this looks like this:
   * <pre>
   *   event_type1: {trigger1, trigger2, trigger3, trigger6}
   *   event_type2: {trigger4, trigger5}
   *   event_type3: {trigger7, trigger8}
   * </pre>
   */
  void DoMerge(const EventInfo* other_info) override {
    const LeafEventInfo* other = dynamic_cast<const LeafEventInfo*>(other_info);
    DRAKE_DEMAND(other != nullptr);

    const std::vector<const PublishEvent<T>*>& other_publish =
        other->get_publish_events();
    for (const PublishEvent<T>* other_event : other_publish) {
      other_event->add_to(this);
    }

    const std::vector<const DiscreteUpdateEvent<T>*>& other_discrete_update = other->get_discrete_update_events();
    for (const DiscreteUpdateEvent<T>* other_event : other_discrete_update) {
      other_event->add_to(this);
    }

    const std::vector<const UnrestrictedUpdateEvent<T>*>& other_unrestricted_update = other->get_unrestricted_update_events();
    for (const UnrestrictedUpdateEvent<T>* other_event : other_unrestricted_update) {
      other_event->add_to(this);
    }
  }

 private:
  std::vector<std::unique_ptr<PublishEvent<T>>> owned_publish_events_;

  std::vector<std::unique_ptr<DiscreteUpdateEvent<T>>> owned_discrete_update_events_;

  std::vector<std::unique_ptr<UnrestrictedUpdateEvent<T>>> owned_unrestricted_update_events_;

  std::vector<const PublishEvent<T>*> publish_events_;

  std::vector<const DiscreteUpdateEvent<T>*> discrete_update_events_;

  std::vector<const UnrestrictedUpdateEvent<T>*> unrestricted_update_events_;
};

}  // namespace systems
}  // namespace drake
