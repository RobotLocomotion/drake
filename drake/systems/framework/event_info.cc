#include "drake/systems/framework/event_info.h"

#include <utility>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

void DiagramEventInfo::set_and_own_sub_event_info(
    int index, std::unique_ptr<EventInfo> sub_event_info) {
  DRAKE_DEMAND(index >= 0 && index < num_sub_event_info());
  owned_sub_event_info_[index] = std::move(sub_event_info);
  sub_event_info_[index] = owned_sub_event_info_[index].get();
}

const EventInfo* DiagramEventInfo::get_sub_event_info(int index) const {
  DRAKE_DEMAND(index >= 0 && index < num_sub_event_info());
  return sub_event_info_[index];
}

EventInfo* DiagramEventInfo::get_mutable_sub_event_info(int index) {
  DRAKE_DEMAND(index >= 0 && index < num_sub_event_info());
  return sub_event_info_[index];
}

void DiagramEventInfo::DoMerge(const EventInfo* other_info) {
  const DiagramEventInfo* other =
      dynamic_cast<const DiagramEventInfo*>(other_info);
  DRAKE_DEMAND(other != nullptr);
  DRAKE_DEMAND(num_sub_event_info() == other->num_sub_event_info());

  for (int i = 0; i < num_sub_event_info(); i++) {
    sub_event_info_[i]->Merge(*(other->get_sub_event_info(i)));
  }
}

void DiagramEventInfo::Clear() {
  for (EventInfo* sub_event : sub_event_info_) {
    sub_event->Clear();
  }
}

bool DiagramEventInfo::HasEvent(EventType event_type) const {
  for (const EventInfo* sub_event : sub_event_info_) {
    if (sub_event->HasEvent(event_type)) return true;
  }
  return false;
}

bool DiagramEventInfo::HasNoEvents() const {
  for (const EventInfo* sub_event : sub_event_info_) {
    if (!sub_event->HasNoEvents()) return false;
  }
  return true;
}

void LeafEventInfo::DoMerge(const EventInfo* other_info) {
  const LeafEventInfo* other = dynamic_cast<const LeafEventInfo*>(other_info);
  DRAKE_DEMAND(other != nullptr);

  for (const auto& other_pair : other->events_) {
    for (const auto& trigger_handler_pair : other_pair.second) {
      add_trigger(trigger_handler_pair.first->Clone(),
                  trigger_handler_pair.second->Clone());
    }
  }
}

const std::vector<std::pair<const Trigger*, const Handler*>>&
LeafEventInfo::get_triggers(EventType event) const {
  auto it = events_.find(event);
  if (it == events_.end()) {
    DRAKE_ABORT_MSG("no such event");
  }
  return it->second;
}

void LeafEventInfo::add_trigger(std::unique_ptr<Trigger> trigger,
                                std::unique_ptr<Handler> handler) {
  owned_triggers_.push_back(std::move(trigger));
  owned_handlers_.push_back(std::move(handler));

  // HACK
  EventType event_type = static_cast<EventType>(owned_handlers_.back()->get_type());

  auto it = events_.find(event_type);

  TriggerHandlerPair pair(owned_triggers_.back().get(),
                          owned_handlers_.back().get());

  if (it == events_.end()) {
    std::vector<TriggerHandlerPair> triggers(1, pair);
    events_.emplace(event_type, triggers);
  } else {
    it->second.push_back(pair);
  }
}

}  // namespace systems
}  // namespace drake
