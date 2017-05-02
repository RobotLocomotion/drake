#include "drake/systems/framework/event_collection.h"

#include <utility>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

void DiagramEventCollection::set_and_own_sub_event_collection(
    int index, std::unique_ptr<EventCollection> sub_event_collection) {
  DRAKE_DEMAND(index >= 0 && index < num_sub_event_collection());
  owned_sub_event_collection_[index] = std::move(sub_event_collection);
  sub_event_collection_[index] = owned_sub_event_collection_[index].get();
}

const EventCollection* DiagramEventCollection::get_sub_event_collection(int index) const {
  DRAKE_DEMAND(index >= 0 && index < num_sub_event_collection());
  return sub_event_collection_[index];
}

EventCollection* DiagramEventCollection::get_mutable_sub_event_collection(int index) {
  DRAKE_DEMAND(index >= 0 && index < num_sub_event_collection());
  return sub_event_collection_[index];
}

void DiagramEventCollection::DoMerge(const EventCollection* other_collection) {
  const DiagramEventCollection* other =
      dynamic_cast<const DiagramEventCollection*>(other_collection);
  DRAKE_DEMAND(other != nullptr);
  DRAKE_DEMAND(num_sub_event_collection() == other->num_sub_event_collection());

  for (int i = 0; i < num_sub_event_collection(); i++) {
    sub_event_collection_[i]->Merge(*(other->get_sub_event_collection(i)));
  }
}

void DiagramEventCollection::Clear() {
  for (EventCollection* sub_event : sub_event_collection_) {
    sub_event->Clear();
  }
}

bool DiagramEventCollection::HasPublishEvents() const {
  for (const EventCollection* sub_event : sub_event_collection_) {
    if (sub_event->HasPublishEvents()) return true;
  }
  return false;
}

bool DiagramEventCollection::HasDiscreteUpdateEvents() const {
  for (const EventCollection* sub_event : sub_event_collection_) {
    if (sub_event->HasDiscreteUpdateEvents()) return true;
  }
  return false;
}

bool DiagramEventCollection::HasUnrestrictedUpdateEvents() const {
  for (const EventCollection* sub_event : sub_event_collection_) {
    if (sub_event->HasUnrestrictedUpdateEvents()) return true;
  }
  return false;
}

bool DiagramEventCollection::HasNoEvents() const {
  for (const EventCollection* sub_event : sub_event_collection_) {
    if (!sub_event->HasNoEvents()) return false;
  }
  return true;
}

template class PublishEvent<double>;
template class PublishEvent<AutoDiffXd>;

template class DiscreteUpdateEvent<double>;
template class DiscreteUpdateEvent<AutoDiffXd>;

template class UnrestrictedUpdateEvent<double>;
template class UnrestrictedUpdateEvent<AutoDiffXd>;

template class LeafEventCollection<double>;
template class LeafEventCollection<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
