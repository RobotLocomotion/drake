#include "drake/systems/framework/event_info.h"

#include <iostream>
#include <utility>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

void DiagramEventInfo::set_and_own_sub_event(int index,
    std::unique_ptr<EventInfo> sub_event_info) {
  DRAKE_DEMAND(index >= 0 && index < size());
  owned_sub_event_info_[index] = std::move(sub_event_info);
  sub_event_info_[index] = owned_sub_event_info_[index].get();
}

const EventInfo* DiagramEventInfo::get_sub_event(int index) const {
  DRAKE_DEMAND(index >= 0 && index < size());
  return sub_event_info_[index];
}

EventInfo* DiagramEventInfo::get_mutable_sub_event(int index) {
  DRAKE_DEMAND(index >= 0 && index < size());
  return sub_event_info_[index];
}

void DiagramEventInfo::merge(const EventInfo* other_info) {
  if (other_info == this) return;

  const DiagramEventInfo* other =
    dynamic_cast<const DiagramEventInfo*>(other_info);
  if (other == nullptr)
    DRAKE_ABORT_MSG(
        "cannot merger DiagramEventInfo with non DiagramEventInfo.");

  DRAKE_DEMAND(size() == other->size());

  for (int i = 0; i < size(); i++) {
    sub_event_info_[i]->merge(other->get_sub_event(i));
  }
}

void DiagramEventInfo::clear() {
  for (int i = 0; i < size(); i++) {
    sub_event_info_[i]->clear();
  }
}

bool DiagramEventInfo::has_event(EventType event) const {
  for (int i = 0; i < size(); i++) {
    if (sub_event_info_[i]->has_event(event)) return true;
  }
  return false;
}

bool DiagramEventInfo::empty() const {
  for (int i = 0; i < size(); i++) {
    if (!sub_event_info_[i]->empty()) return false;
  }
  return true;
}

void DiagramEventInfo::print() const {
  for (int i = 0; i < size(); i++) {
    std::cout << "subsys: " << std::to_string(i) << std::endl;
    sub_event_info_[i]->print();
  }
}


void LeafEventInfo::merge(const EventInfo* other_info) {
  if (other_info == this) return;

  const LeafEventInfo* other = dynamic_cast<const LeafEventInfo*>(other_info);
  if (other == nullptr)
    DRAKE_ABORT_MSG("cannot merger LeafEventInfo with non LeafEventInfo.");

  for (const auto& other_pair : other->events_) {
    for (const auto& trigger : other_pair.second) {
      add_trigger(other_pair.first, trigger->Clone());
    }
  }
}

bool LeafEventInfo::has_event(EventType event) const {
  return events_.find(event) != events_.end();
}

bool LeafEventInfo::empty() const {
  return events_.empty();
}

const std::vector<const Trigger*>& LeafEventInfo::get_triggers(EventType event) const {
  auto it = events_.find(event);
  if (it == events_.end()) {
    DRAKE_ABORT_MSG("no such event");
  }
  return it->second;
}

void LeafEventInfo::add_trigger(EventType event, std::unique_ptr<Trigger> trigger) {
  owned_triggers_.push_back(std::move(trigger));
  auto it = events_.find(event);
  if (it == events_.end()) {
    std::vector<const Trigger*> triggers(1, owned_triggers_.back().get());
    events_.emplace(event, triggers);
  } else {
    it->second.push_back(owned_triggers_.back().get());
  }
}

void LeafEventInfo::clear() {
  events_.clear();
}

void LeafEventInfo::print() const {
  /*
  for (const auto& pair : events_) {
    std::cout << "\t"
      << "event: " << pair.first << ", trigger: " << pair.second
      << std::endl;
  }
  */
}


}  // namespace systems
}  // namespace drake
