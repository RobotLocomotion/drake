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
 * Base class that holds event related information.
 */
class EventInfo {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EventInfo)

  enum EventType {
    kUnknownEvent = 0,
    kPublish = 1,
    kDiscreteUpdate = 2,
    kUnrestrictedUpdate = 3,
  };

  EventInfo() = default;

  virtual ~EventInfo() {}

  virtual void merge(const EventInfo* other) = 0;

  virtual void clear() = 0;

  virtual bool has_event(EventType event) const = 0;

  virtual bool empty() const = 0;

  virtual void print() const = 0;
};

class DiagramEventInfo : public EventInfo {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramEventInfo)

  explicit DiagramEventInfo(int size)
      : EventInfo(), sub_event_info_(size), owned_sub_event_info_(size) {}

  int size() const { return static_cast<int>(sub_event_info_.size()); }

  void set_and_own_sub_event(int index,
                             std::unique_ptr<EventInfo> sub_event_info);

  const EventInfo* get_sub_event(int index) const;

  EventInfo* get_mutable_sub_event(int index);

  void merge(const EventInfo* other_info) final;

  void clear() final;

  bool has_event(EventType event) const final;

  bool empty() const final;

  void print() const final;

 private:
  std::vector<EventInfo*> sub_event_info_;
  std::vector<std::unique_ptr<EventInfo>> owned_sub_event_info_;
};

/**
 * Holds event related information for LeafSystem
 */
class LeafEventInfo : public EventInfo {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeafEventInfo)

  LeafEventInfo() = default;

  /**
   * Merges with @p other_info, which must also be of type LeafEventInfo.
   */
  void merge(const EventInfo* other_info) final;

  bool has_event(EventType event) const final;

  bool empty() const final;

  /**
   * Returns the TriggerType that's associated with @p event, or
   * TriggerType::kUnknownTrigger if @p event doesn't exsit.
   */
  const std::vector<const Trigger*>& get_triggers(EventType event) const;

  /**
   * If this does not have @p event, @p event and @p trigger will be insterted.
   * Otherwise, @p trigger will be | to the exsiting triggers that corresponds
   * to @p event.
   */
  void add_trigger(EventType event, std::unique_ptr<Trigger> trigger);

  void clear() final;

  void print() const final;

 private:
  std::list<std::unique_ptr<Trigger>> owned_triggers_;
  std::map<EventType, std::vector<const Trigger*>> events_;
};

}  // namespace systems
}  // namespace drake
