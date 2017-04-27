#pragma once

#include <list>
#include <map>
#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/trigger.h"

namespace drake {
namespace systems {

/**
 * Base class that represents all events at a particular time for System.
 * There are several predefined event and trigger types. To represent a concrete
 * event, a pair of event type and Trigger is necessary. The former specifies
 * the event type, and the latter captures all the details of the actual event.
 *
 * For each event type, the System API provides a unique customizable function
 * for handling all the simultaneous events of that type, such as
 * System::DoPublish(context, triggers), where `triggers` represents the
 * individual events. The user is responsible for overriding such functions to
 * handle each event in the desired order. For example, suppose publish type
 * events are being handled, which are represented by
 * `triggers` = {kPerStep, kPeriodic}. Depending on the desired behavior, the
 * user has the freedom to ignore both, handle only one, or both in any
 * arbitrary order. For each type of events at any given moment in time, its
 * handler should only be invoked once.
 *
 * The System API also provides several functions for customizable event
 * triggering such as System::DoCalcNextUpdateTime() or
 * System::DoGetPerStepEvents(). These functions can trigger any number of
 * events of arbitrary types, and the resulting events are stored in separate
 * EventInfo instances. Before calling the event handlers, all these EventInfo
 * need to be merged, so that the handlers have a complete set of events.
 *
 * Here is a complete example. For some LeafSystem `sys`, its
 * System::DoCalcNextUpdateTime() triggers the following events (`event_info1`):
 * <pre>
 *   kPublish: {trigger1(kPeriodic)}
 *   kDiscreteUpdate: {trigger2(kPeriodic)}
 * </pre>
 * It also has per step events (`event_info2`) triggered by its
 * System::DoGetPerStepEvents():
 * <pre>
 *   kPublish: {trigger3(kPerStep)}
 *   kUnrestrictedUpdate: {trigger4(kPerStep)}
 * </pre>
 * `event_info1` and `event_info2` are then merged into `all_event_info`:
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

  /**
   * Types of discrete-time events. Different event types trigger different
   * event handlers, which have different write access levels to the state.
   * For instance, publish events may not modify any state at all, discrete
   * update events may modify the discrete state only, and unrestricted update
   * events may modify any state.
   */
  enum EventType {
    /// A default value that causes the handler to abort.
    kUnknownEvent = 0,

    /// On publish actions, state does not change.
    kPublish = 1,

    /// On discrete updates, discrete state may change.
    kDiscreteUpdate = 2,

    /// On unrestricted updates, the state variables may change arbitrarily.
    kUnrestrictedUpdate = 3,
  };

  virtual ~EventInfo() {}

  /**
   * Merges @p other's event information into this. See derived DoMerge() for
   * more details. @p other cannot be null.
   */
  void Merge(const EventInfo* other) {
    DRAKE_DEMAND(other != nullptr);
    if (other == this) return;
    DoMerge(other);
  }

  /**
   * Clears all the triggers associated with all the event types.
   */
  virtual void Clear() = 0;

  /**
   * Returns true if an event of @p event_type exists.
   */
  virtual bool HasEvent(EventType event_type) const = 0;

  /**
   * Returns true if no event exists.
   */
  virtual bool HasNoEvents() const = 0;

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
  bool HasEvent(EventType event_type) const override;

  /**
   * Returns true if all sub event info are empty.
   */
  bool HasNoEvents() const override;

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
  const std::vector<const Trigger*>& get_triggers(EventType event_type) const;

  /**
   * Adds a trigger to @p event_type. @p event_type will also be added if this
   * does not have a @p event_type event. Ownership of @p trigger is transfered.
   */
  void add_trigger(EventType event_type, std::unique_ptr<Trigger> trigger);

  /**
   * Returns true if an event of @p event_type exists.
   */
  bool HasEvent(EventType event_type) const override {
    return events_.find(event_type) != events_.end();
  }

  /**
   * Returns true if no event exists.
   */
  bool HasNoEvents() const override { return events_.empty(); }

  /**
   * Clears all events.
   */
  void Clear() override { events_.clear(); }

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
  void DoMerge(const EventInfo* other_info) override;

 private:
  // List just for holding the unique pointers.
  std::list<std::unique_ptr<Trigger>> owned_triggers_;

  // A map from event type to its associated triggers. Pointers point to
  // owned_triggers_.
  std::map<EventType, std::vector<const Trigger*>> events_;
};

}  // namespace systems
}  // namespace drake
