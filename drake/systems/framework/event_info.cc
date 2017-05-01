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

bool DiagramEventInfo::HasPublishEvents() const {
  for (const EventInfo* sub_event : sub_event_info_) {
    if (sub_event->HasPublishEvents()) return true;
  }
  return false;
}

bool DiagramEventInfo::HasDiscreteUpdateEvents() const {
  for (const EventInfo* sub_event : sub_event_info_) {
    if (sub_event->HasDiscreteUpdateEvents()) return true;
  }
  return false;
}

bool DiagramEventInfo::HasUnrestrictedUpdateEvents() const {
  for (const EventInfo* sub_event : sub_event_info_) {
    if (sub_event->HasUnrestrictedUpdateEvents()) return true;
  }
  return false;
}

bool DiagramEventInfo::HasNoEvents() const {
  for (const EventInfo* sub_event : sub_event_info_) {
    if (!sub_event->HasNoEvents()) return false;
  }
  return true;
}

template class LeafEventInfo<double>;

}  // namespace systems
}  // namespace drake
