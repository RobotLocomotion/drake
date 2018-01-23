#include "drake/systems/framework/dependency_tracker.h"

#include <algorithm>
#include <utility>

using std::pair;

namespace drake {
namespace systems {

CacheEntryValue DependencyTracker::dummy_cache_value_(true);

// Our associated value has initiated a change (e.g. the associated value is
// time and someone advanced time). Short circuit if this is part of a change
// event that we have already heard about. Otherwise, let the subscribers know
// that things have changed. Update statistics.
void DependencyTracker::NoteValueChange(int64_t change_event) const {
  DRAKE_ASSERT(change_event > 0);
  SPDLOG_DEBUG(log(), "Tracker '{}' value change event {} ...",
               GetPathDescription(), change_event);

  ++num_value_change_notifications_received_;
  if (last_change_event_ == change_event) {
    ++num_ignored_notifications_;
    SPDLOG_DEBUG(log(),
                 "... ignoring repeated notification same change event.");
    return;
  }
  last_change_event_ = change_event;
  NotifySubscribers(change_event, 0);
}

// A prerequisite says it has changed. Short circuit if we've already heard
// about this change event. Otherwise, notify our associated value (e.g. to
// invalidate a cache entry), and then pass on the bad news to our subscribers.
// Update statistics.
void DependencyTracker::NotePrerequisiteChange(
    int64_t change_event,
    const DependencyTracker& prerequisite,
    int depth) const {
  DRAKE_ASSERT(change_event > 0);
  DRAKE_ASSERT(HasPrerequisite(prerequisite));  // Expensive.
  SPDLOG_DEBUG(log(),
               "{}Tracker '{}': prerequisite '{}' changed (event {}) ...",
               Indent(depth), GetPathDescription(),
               prerequisite.GetPathDescription(), change_event);

  ++num_prerequisite_notifications_received_;
  if (last_change_event_ == change_event) {
    ++num_ignored_notifications_;
    SPDLOG_DEBUG(log(),
                 "{}... ignoring repeated notification same change event.",
                 Indent(depth));
    return;
  }
  last_change_event_ = change_event;
  // Update associated value if any.
  cache_value_->set_is_up_to_date(false);
  // Follow up with downstream subscribers.
  NotifySubscribers(change_event, depth);
}

void DependencyTracker::NotifySubscribers(int64_t change_event,
                                          int depth) const {
  SPDLOG_DEBUG(log(), "{}... {} downstream subscribers.{}", Indent(depth),
               num_subscribers(), num_subscribers() > 0 ? " Notifying:" : "");

  for (const DependencyTracker* subscriber : subscribers_) {
    DRAKE_ASSERT(subscriber != nullptr);
    SPDLOG_DEBUG(log(), "{}->{}", Indent(depth),
                 subscriber->GetPathDescription());
    subscriber->NotePrerequisiteChange(change_event, *this, depth + 1);
  }

  num_downstream_notifications_sent_ += num_subscribers();
}

// For debugging use, provide an indent of 2*depth characters.
/*static*/ std::string DependencyTracker::Indent(int depth) {
  std::string s;
  for (int i=0; i < depth; ++i) s += "| ";
  return s;
}

// Given a DependencyTracker that is supposed to be a prerequisite to this
// one, subscribe to it. This is done only at Context allocation and copying
// so we can afford Release-build checks and general mucking about to make
// runtime execution fast.
void DependencyTracker::SubscribeToPrerequisite(
    DependencyTracker* prerequisite) {
  DRAKE_DEMAND(prerequisite != nullptr);
  SPDLOG_DEBUG(log(), "Tracker '{}' subscribing to prerequisite '{}'",
               GetPathDescription(), prerequisite->GetPathDescription());

  // Make sure we haven't already added this prerequisite. Expensive.
  DRAKE_ASSERT(!HasPrerequisite(*prerequisite));  // Expensive.
  prerequisites_.push_back(prerequisite);

  prerequisite->AddDownstreamSubscriber(*this);
}


void DependencyTracker::AddDownstreamSubscriber(
    const DependencyTracker& subscriber) {

  // Make sure we haven't already added this subscriber. Expensive.
  DRAKE_ASSERT(!HasSubscriber(subscriber));
  // Subscriber must have *already* recorded this prerequisite. Expensive.
  DRAKE_ASSERT(subscriber.HasPrerequisite(*this));

  SPDLOG_DEBUG(log(), "Tracker '{}' adding subscriber '{}'",
               GetPathDescription(), subscriber.GetPathDescription());

  subscribers_.push_back(&subscriber);
}

namespace {
// Convenience function for linear search of a vector to see if it contains
// a given value.
template <typename T>
bool Contains(const T& value, const std::vector<T>& to_search) {
  return std::find(to_search.begin(), to_search.end(), value)
      != to_search.end();
}

// Look for the given value and erase it. Fail if not found.
template <typename T>
void Remove(const T& value, std::vector<T>* to_search) {
  auto found = std::find(to_search->begin(), to_search->end(), value);
  DRAKE_DEMAND(found != to_search->end());
  to_search->erase(found);
}

}  // namespace

// Remove a subscription that we made earlier.
void DependencyTracker::UnsubscribeFromPrerequisite(
    DependencyTracker* prerequisite) {
  DRAKE_DEMAND(prerequisite != nullptr);
  SPDLOG_DEBUG(log(), "Tracker '{}' unsubscribing from prerequisite '{}'",
               GetPathDescription(), prerequisite->GetPathDescription());

  // Make sure we have already added this prerequisite. Expensive.
  DRAKE_ASSERT(HasPrerequisite(*prerequisite));  // Expensive.
  Remove<const DependencyTracker*>(prerequisite, &prerequisites_);

  prerequisite->RemoveDownstreamSubscriber(*this);
}

void DependencyTracker::RemoveDownstreamSubscriber(
    const DependencyTracker& subscriber) {

  // Make sure we already added this subscriber. Expensive.
  DRAKE_ASSERT(HasSubscriber(subscriber));
  // Subscriber must have *already* removed this prerequisite. Expensive.
  DRAKE_ASSERT(!subscriber.HasPrerequisite(*this));

  SPDLOG_DEBUG(log(), "Tracker '{}' removing subscriber '{}'",
               GetPathDescription(), subscriber.GetPathDescription());

  Remove<const DependencyTracker*>(&subscriber, &subscribers_);
}

std::string DependencyTracker::GetPathDescription() const {
  return GetSystemPathname() + ":" + description();
}

// Figure out which list the prerequisite would be on, then see if it
// is present.
bool DependencyTracker::HasPrerequisite(
    const DependencyTracker& prerequisite) const {
  return Contains(&prerequisite, prerequisites_);
}

// Figure out which list the subscriber would be on, then see if it
// is present.
bool DependencyTracker::HasSubscriber(
    const DependencyTracker& subscriber) const {
  return Contains(&subscriber, subscribers_);
}

void DependencyTracker::RepairTrackerPointers(
    const DependencyTracker& source,
    const DependencyTracker::PointerMap& tracker_map,
    const SystemPathnameInterface* owning_subcontext, Cache* cache) {
  DRAKE_DEMAND(owning_subcontext != nullptr);
  DRAKE_DEMAND(cache != nullptr);
  owning_subcontext_ = owning_subcontext;

  // Set the cache entry pointer.
  if (source.cache_value_ == &dummy_cache_value_) {
    cache_value_ = &dummy_cache_value_;
  } else {
    const CacheIndex source_index(source.cache_value_->cache_index());
    cache_value_ = &cache->get_mutable_cache_entry_value(source_index);
    SPDLOG_DEBUG(
        log(),
        "Cloned tracker '{}' repairing cache entry {} invalidation to {:#x}.",
        GetPathDescription(), source.cache_value_->cache_index(),
        size_t(cache_value_));
  }

  // Set the subscriber pointers.
  DRAKE_DEMAND(num_subscribers() == source.num_subscribers());
  for (int i = 0; i < num_subscribers(); ++i) {
    DRAKE_ASSERT(subscribers_[i] == nullptr);
    auto map_entry = tracker_map.find(source.subscribers()[i]);
    DRAKE_DEMAND(map_entry != tracker_map.end());
    subscribers_[i] = map_entry->second;
  }

  // Set the prerequisite pointers.
  DRAKE_DEMAND(num_prerequisites() == source.num_prerequisites());
  for (int i = 0; i < num_prerequisites(); ++i) {
    DRAKE_ASSERT(prerequisites_[i] == nullptr);
    auto map_entry = tracker_map.find(source.prerequisites()[i]);
    DRAKE_DEMAND(map_entry != tracker_map.end());
    prerequisites_[i] = map_entry->second;
  }
}

void DependencyGraph::AppendToTrackerPointerMap(
    const DependencyGraph& clone,
    DependencyTracker::PointerMap* tracker_map) const {
  DRAKE_DEMAND(tracker_map != nullptr);
  DRAKE_DEMAND(clone.num_trackers() == num_trackers());
  for (DependencyTicket ticket(0); ticket < num_trackers(); ++ticket) {
    if (!has_tracker(ticket)) continue;
    (*tracker_map)[&get_tracker(ticket)] = &clone.get_tracker(ticket);
  }
}

void DependencyGraph::RepairTrackerPointers(
    const DependencyGraph& source,
    const DependencyTracker::PointerMap& tracker_map,
    const SystemPathnameInterface* owning_subcontext, Cache* new_cache) {
  DRAKE_DEMAND(owning_subcontext != nullptr);
  owning_subcontext_ = owning_subcontext;
  for (DependencyTicket ticket(0); ticket < num_trackers(); ++ticket) {
    if (!has_tracker(ticket)) continue;
    get_mutable_tracker(ticket).RepairTrackerPointers(
        source.get_tracker(ticket), tracker_map, owning_subcontext, new_cache);
  }
}

}  // namespace systems
}  // namespace drake
