#include "drake/systems/framework/dependency_tracker.h"

#include <algorithm>

#include "drake/common/unused.h"

namespace drake {
namespace systems {

namespace {
// For debugging use, provide an indent of 2*depth characters.
std::string Indent(int depth) {
  std::string s;
  for (int i = 0; i < depth; ++i) s += "| ";
  return s;
}
}  // namespace

// Our associated value has initiated a change (e.g. the associated value is
// time and someone advanced time). Short circuit if this is part of a change
// event that we have already heard about. Otherwise, let the subscribers know
// that things have changed. Update statistics.
void DependencyTracker::NoteValueChange(int64_t change_event) const {
  DRAKE_LOGGER_DEBUG("Tracker '{}' value change event {} ...",
                     GetPathDescription(), change_event);
  DRAKE_ASSERT(change_event > 0);

  ++num_value_change_notifications_received_;
  if (last_change_event_ == change_event) {
    ++num_ignored_notifications_;
    DRAKE_LOGGER_DEBUG(
        "... ignoring repeated value change notification same change event.");
    return;
  }
  last_change_event_ = change_event;
  NotifySubscribers(change_event, 0);
}

// A prerequisite says it has changed. Short circuit if we've already heard
// about this change event. Otherwise, invalidate the associated cache entry and
// then pass on the bad news to our subscribers. Update statistics.
void DependencyTracker::NotePrerequisiteChange(
    int64_t change_event,
    const DependencyTracker& prerequisite,
    int depth) const {
  unused(Indent);  // Avoid warning in non-Debug builds.
  DRAKE_LOGGER_DEBUG(
      "{}Tracker '{}': prerequisite '{}' changed (event {}) ...",
      Indent(depth), GetPathDescription(), prerequisite.GetPathDescription(),
      change_event);
  DRAKE_ASSERT(change_event > 0);
  DRAKE_ASSERT(HasPrerequisite(prerequisite));  // Expensive.

  ++num_prerequisite_notifications_received_;
  if (last_change_event_ == change_event) {
    ++num_ignored_notifications_;
    DRAKE_LOGGER_DEBUG(
        "{}... ignoring repeated prereq change notification same change event.",
        Indent(depth));
    return;
  }
  last_change_event_ = change_event;
  // Invalidate associated cache entry value if any.
  cache_value_->mark_out_of_date();
  // Follow up with downstream subscribers.
  NotifySubscribers(change_event, depth);
}

void DependencyTracker::NotifySubscribers(int64_t change_event,
                                          int depth) const {
  DRAKE_LOGGER_DEBUG("{}... {} downstream subscribers.{}", Indent(depth),
                     num_subscribers(),
                     num_subscribers() > 0 ? " Notifying:" : "");
  DRAKE_ASSERT(change_event > 0);
  DRAKE_ASSERT(depth >= 0);

  for (const DependencyTracker* subscriber : subscribers_) {
    DRAKE_ASSERT(subscriber != nullptr);
    DRAKE_LOGGER_DEBUG("{}->{}", Indent(depth),
                       subscriber->GetPathDescription());
    subscriber->NotePrerequisiteChange(change_event, *this, depth + 1);
  }

  num_downstream_notifications_sent_ += num_subscribers();
}

// Given a DependencyTracker that is supposed to be a prerequisite to this
// one, subscribe to it. This is done only at Context allocation and copying
// so we can afford Release-build checks and general mucking about to make
// runtime execution fast.
void DependencyTracker::SubscribeToPrerequisite(
    DependencyTracker* prerequisite) {
  DRAKE_DEMAND(prerequisite != nullptr);
  DRAKE_LOGGER_DEBUG("Tracker '{}' subscribing to prerequisite '{}'",
                     GetPathDescription(), prerequisite->GetPathDescription());

  // Make sure we haven't already added this prerequisite.
  DRAKE_ASSERT(!HasPrerequisite(*prerequisite));  // Expensive.
  prerequisites_.push_back(prerequisite);

  prerequisite->AddDownstreamSubscriber(*this);
}

void DependencyTracker::AddDownstreamSubscriber(
    const DependencyTracker& subscriber) {
  DRAKE_LOGGER_DEBUG("Tracker '{}' adding subscriber '{}'",
                     GetPathDescription(), subscriber.GetPathDescription());
  // Make sure we haven't already added this subscriber.
  DRAKE_ASSERT(!HasSubscriber(subscriber));  // Expensive.
  // Subscriber must have *already* recorded this prerequisite.
  DRAKE_ASSERT(subscriber.HasPrerequisite(*this));  // Expensive.

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
  DRAKE_LOGGER_DEBUG("Tracker '{}' unsubscribing from prerequisite '{}'",
                     GetPathDescription(), prerequisite->GetPathDescription());

  // Make sure we have already added this prerequisite.
  DRAKE_ASSERT(HasPrerequisite(*prerequisite));  // Expensive.
  Remove<const DependencyTracker*>(prerequisite, &prerequisites_);

  prerequisite->RemoveDownstreamSubscriber(*this);
}

void DependencyTracker::RemoveDownstreamSubscriber(
    const DependencyTracker& subscriber) {
  DRAKE_LOGGER_DEBUG("Tracker '{}' removing subscriber '{}'",
                     GetPathDescription(), subscriber.GetPathDescription());
  // Make sure we already added this subscriber.
  DRAKE_ASSERT(HasSubscriber(subscriber));  // Expensive.
  // Subscriber must have *already* removed this prerequisite.
  DRAKE_ASSERT(!subscriber.HasPrerequisite(*this));  // Expensive.

  Remove<const DependencyTracker*>(&subscriber, &subscribers_);
}

std::string DependencyTracker::GetPathDescription() const {
  return GetSystemPathname() + ":" + description();
}

bool DependencyTracker::HasPrerequisite(
    const DependencyTracker& prerequisite) const {
  return Contains(&prerequisite, prerequisites_);
}

bool DependencyTracker::HasSubscriber(
    const DependencyTracker& subscriber) const {
  return Contains(&subscriber, subscribers_);
}

void DependencyTracker::ThrowIfBadDependencyTracker(
    const internal::ContextMessageInterface* owning_subcontext,
    const CacheEntryValue* cache_value) const {
  if (owning_subcontext_ == nullptr) {
    // Can't use FormatName() here because that depends on us having an owning
    // context to talk to.
    throw std::logic_error("DependencyTracker(" + description() + ")::" +
        __func__ +
        "(): tracker has no owning subcontext.");
  }
  if (owning_subcontext && owning_subcontext_ != owning_subcontext) {
    throw std::logic_error(FormatName(__func__) + "wrong owning subcontext.");
  }
  if (cache_value_ == nullptr) {
    throw std::logic_error(
        FormatName(__func__) +
            "no associated cache entry value (should at least be a dummy).");
  }
  if (cache_value && cache_value_ != cache_value) {
    throw std::logic_error(FormatName(__func__) +
        "wrong associated cache entry value.");
  }
  if (!ticket_.is_valid()) {
    throw std::logic_error(FormatName(__func__) +
        "dependency ticket invalid.");
  }
  if (last_change_event_ < -1) {
    throw std::logic_error(FormatName(__func__) +
        "last change event has an absurd value.");
  }
  if (num_value_change_notifications_received_ < 0 ||
      num_prerequisite_notifications_received_ < 0 ||
      num_ignored_notifications_ < 0 ||
      num_downstream_notifications_sent_ < 0) {
    throw std::logic_error(FormatName(__func__) +
        "a counter has a negative value.");
  }
}

void DependencyTracker::RepairTrackerPointers(
    const DependencyTracker& source,
    const DependencyTracker::PointerMap& tracker_map,
    const internal::ContextMessageInterface* owning_subcontext, Cache* cache) {
  DRAKE_DEMAND(owning_subcontext != nullptr);
  DRAKE_DEMAND(cache != nullptr);
  owning_subcontext_ = owning_subcontext;

  // Set the cache entry pointer to refer to the new cache, either to a real
  // CacheEntryValue or the cache's dummy value.
  DRAKE_DEMAND(has_associated_cache_entry_ ==
               source.has_associated_cache_entry_);
  if (has_associated_cache_entry_) {
    const CacheIndex source_index(source.cache_value_->cache_index());
    cache_value_ = &cache->get_mutable_cache_entry_value(source_index);
    DRAKE_LOGGER_DEBUG(
        "Cloned tracker '{}' repairing cache entry {} invalidation to {:#x}.",
        GetPathDescription(), source.cache_value_->cache_index(),
        size_t(cache_value_));
  } else {
    cache_value_ = &cache->dummy_cache_entry_value();
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

  // This should never happen, but ...
  ThrowIfBadDependencyTracker();
}

void DependencyGraph::AppendToTrackerPointerMap(
    const DependencyGraph& clone,
    DependencyTracker::PointerMap* tracker_map) const {
  DRAKE_DEMAND(tracker_map != nullptr);
  DRAKE_DEMAND(clone.trackers_size() == trackers_size());
  for (DependencyTicket ticket(0); ticket < trackers_size(); ++ticket) {
    if (!has_tracker(ticket))
      continue;
    const bool added = tracker_map->emplace(&get_tracker(ticket),
                                            &clone.get_tracker(ticket)).second;
    DRAKE_DEMAND(added);  // Shouldn't have been there.
  }
}

void DependencyGraph::RepairTrackerPointers(
    const DependencyGraph& source,
    const DependencyTracker::PointerMap& tracker_map,
    const internal::ContextMessageInterface* owning_subcontext,
    Cache* new_cache) {
  DRAKE_DEMAND(owning_subcontext != nullptr);
  owning_subcontext_ = owning_subcontext;
  for (DependencyTicket ticket(0); ticket < trackers_size(); ++ticket) {
    if (!has_tracker(ticket))
      continue;
    get_mutable_tracker(ticket).RepairTrackerPointers(
        source.get_tracker(ticket), tracker_map, owning_subcontext, new_cache);
  }
}

}  // namespace systems
}  // namespace drake
