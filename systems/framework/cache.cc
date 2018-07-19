#include "drake/systems/framework/cache.h"

#include <typeindex>
#include <typeinfo>

#include "drake/systems/framework/dependency_tracker.h"

namespace drake {
namespace systems {

std::string CacheEntryValue::GetPathDescription() const {
  DRAKE_DEMAND(owning_subcontext_!= nullptr);
  return owning_subcontext_->GetSystemPathname() + ":" + description();
}

void CacheEntryValue::ThrowIfBadCacheEntryValue(
    const internal::ContextMessageInterface* owning_subcontext) const {
  if (owning_subcontext_ == nullptr) {
    // Can't use FormatName() here because that depends on us having an owning
    // context to talk to.
    throw std::logic_error("CacheEntryValue(" + description() + ")::" +
                           __func__ + "(): entry has no owning subcontext.");
  }
  if (owning_subcontext && owning_subcontext_ != owning_subcontext) {
    throw std::logic_error(FormatName(__func__) + "wrong owning subcontext.");
  }
  if ((flags_ & ~(kValueIsOutOfDate | kCacheEntryIsDisabled)) != 0) {
    throw std::logic_error(FormatName(__func__) +
                           "flags value is out of range.");
  }
  if (serial_number() < 0) {
    throw std::logic_error(FormatName(__func__) + "serial number is negative.");
  }
  if (!(cache_index_.is_valid() && ticket_.is_valid())) {
    throw std::logic_error(FormatName(__func__) +
                           "cache index or dependency ticket invalid.");
  }
}

void CacheEntryValue::ThrowIfBadOtherValue(
    const char* api,
    const std::unique_ptr<AbstractValue>* other_value_ptr) const {
  if (other_value_ptr == nullptr)
    throw std::logic_error(FormatName(api) + "null other_value pointer.");

  auto& other_value = *other_value_ptr;
  if (other_value == nullptr)
    throw std::logic_error(FormatName(api) + "other_value is empty.");

  DRAKE_DEMAND(value_ != nullptr);  // Should have been checked already.

  if (value_->type_info() != other_value->type_info()) {
    throw std::logic_error(FormatName(api) +
                           "other_value has wrong concrete type " +
                           other_value->GetNiceTypeName() + ". Expected " +
                           value_->GetNiceTypeName() + ".");
  }
}

CacheEntryValue& Cache::CreateNewCacheEntryValue(
    CacheIndex index, DependencyTicket ticket,
    const std::string& description,
    const std::set<DependencyTicket>& prerequisites,
    DependencyGraph* trackers) {
  DRAKE_DEMAND(trackers != nullptr);
  DRAKE_DEMAND(index.is_valid() && ticket.is_valid());

  // Make sure there is a place for this cache entry in the cache.
  if (index >= cache_size())
    store_.resize(index + 1);

  // Create the new cache entry value and install it into this Cache. Note that
  // indirection here means the CacheEntryValue object's address is stable
  // even when store_ is resized.
  DRAKE_DEMAND(store_[index] == nullptr);
  // Can't use make_unique because constructor is private.
  store_[index] = std::unique_ptr<CacheEntryValue>(
      new CacheEntryValue(index, ticket, description, owning_subcontext_,
                          nullptr /* no value yet */));
  CacheEntryValue& value = *store_[index];

  // Obtain a DependencyTracker for the CacheEntryValue. Normally there will be
  // no tracker associated with the given ticket. However, if this cache entry
  // corresponds to a well-known tracker (e.g. continuous derivatives xcdot)
  // that tracker will already have been created earlier and we just need to
  // point the tracker at the new cache entry value.
  DependencyTracker* tracker{};
  if (trackers->has_tracker(ticket)) {
    // Pre-existing trackers should only be present for well-known tickets.
    DRAKE_DEMAND(ticket < internal::kNextAvailableTicket);
    tracker = &trackers->get_mutable_tracker(ticket);
    tracker->set_cache_entry_value(&value);
  } else {
    // Allocate a DependencyTracker for this cache entry. Note that a pointer
    // to the new CacheEntryValue is retained so must have a lifetime matching
    // the tracker. That requires that the Cache and DependencyGraph are
    // contained in the same Context.
    tracker = &trackers->CreateNewDependencyTracker(
        ticket,
        "cache " + description,
        &value);
  }

  // Subscribe to prerequisites (trackers must already exist).
  for (auto prereq : prerequisites) {
    auto& prereq_tracker = trackers->get_mutable_tracker(prereq);
    tracker->SubscribeToPrerequisite(&prereq_tracker);
  }
  return value;
}

void Cache::DisableCaching() {
  for (auto& entry : store_)
    if (entry) entry->disable_caching();
}


void Cache::EnableCaching() {
  for (auto& entry : store_)
    if (entry) entry->enable_caching();
}

void Cache::SetAllEntriesOutOfDate() {
  for (auto& entry : store_)
    if (entry) entry->mark_out_of_date();
}

void Cache::RepairCachePointers(
    const internal::ContextMessageInterface* owning_subcontext) {
  DRAKE_DEMAND(owning_subcontext != nullptr);
  DRAKE_DEMAND(owning_subcontext_ == nullptr);
  owning_subcontext_ = owning_subcontext;
  for (auto& entry : store_)
    if (entry) entry->set_owning_subcontext(owning_subcontext);
}

}  // namespace systems
}  // namespace drake
