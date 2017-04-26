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
 * Base class that captures discrete-time events for System. There are several
 * predefined event types. For each event type, the System API provides
 * functions for event generation and handling. For example, for a publish
 * event EventType::kPublish, System::CalcNextUpdateTime() generates it, and
 * System::Publish() is the handler. For each LeafSystem at any given point in
 * time, the event type is unique, but each type can have an arbitrary number
 * of Triggers associated with it, which represent individual instances of
 * events of that type.
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
   * Merges @p other's event information, assuming that @p other has the same
   * topology as this.
   */
  void Merge(const EventInfo* other) {
    if (other == this) return;
    DoMerge(other);
  }

  /**
   * Clears all event information.
   */
  void Clear() { DoClear(); }

  /**
   * Returns true if an event of @p event_type exists.
   */
  bool HasEvent(EventType event_type) const { return DoHasEvent(event_type); }

  /**
   * Returns true if no event exists.
   */
  bool IsEmpty() const { return DoIsEmpty(); }

 protected:
  /**
   * Constructor only accessible by derived class.
   */
  EventInfo() = default;

  virtual void DoMerge(const EventInfo* other) = 0;
  virtual void DoClear() = 0;
  virtual bool DoHasEvent(EventType event_type) const = 0;
  virtual bool DoIsEmpty() const = 0;
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
   * does not allocate any derived EventInfo instances. Users should never call
   * this explicitly. Instead, they should call System::AllocateEventInfo() on
   * the given Diagram.
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

 private:
  // Goes through each sub event info and merges it with the corresponding
  // one in @p other_info. Assumes that @p other_info is an instance of
  // DiagramEventInfo and has the same dimension. Aborts otherwise.
  void DoMerge(const EventInfo* other_info) override;

  //
  // Goes through each sub event info and clears its content.
  void DoClear() override;

  // Returns true if any sub event info contains @p event_type.
  bool DoHasEvent(EventType event_type) const override;

  // Returns true if all sub event info are empty.
  bool DoIsEmpty() const override;

  std::vector<EventInfo*> sub_event_info_;
  std::vector<std::unique_ptr<EventInfo>> owned_sub_event_info_;
};

/**
 * A concrete class that holds event related information for a LeafSystem.
 * This class essentially is a map from EventType to a list of active Triggers.
 * Unique event type is assumed, however multiple events of the same type
 * occurring simultaneously can be represented by associating multiple
 * Triggers with that event type.
 */
class LeafEventInfo final : public EventInfo {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeafEventInfo)

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

 private:
  // For each event type in @p other_info, adds all its associated triggers to
  // this. Assumes that @p other_info is an instance of LeafEventInfo. Aborts
  // otherwise.
  void DoMerge(const EventInfo* other_info) override;

  // Returns true if an event of @p event_type exists.
  bool DoHasEvent(EventType event_type) const override {
    return events_.find(event_type) != events_.end();
  }

  // Returns true if no event exists.
  bool DoIsEmpty() const override { return events_.empty(); }

  // Clears all events.
  void DoClear() override { events_.clear(); }

  // List just for holding the unique pointers.
  std::list<std::unique_ptr<Trigger>> owned_triggers_;

  // A map from event type to its associated triggers. Pointers point to
  // owned_triggers_.
  std::map<EventType, std::vector<const Trigger*>> events_;
};

}  // namespace systems
}  // namespace drake
