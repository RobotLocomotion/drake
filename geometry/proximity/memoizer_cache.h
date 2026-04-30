#pragma once

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/hash.h"

namespace drake {
namespace geometry {
namespace internal {

/* MemoizerCache provides thread-safe storage for memoizing the result of a
function call. The key API is FindOrInsert, where the user must supply a Key
(e.g., a tuple of the function's arguments) and a callback that produces the
function's T-typed result for that Key, if necessary.

A value will remain in the cache only as long as the shared_ptr returned by
FindOrInsert is retained by the user. This is not an LRU cache where values are
evicted under memory pressure, nor where unused values speculatively remain in
case they might be needed again.

@tparam Key must be copyable, equality-comparable, and hashable
@tparam T must be move-constructible and move-assignable */
template <typename Key, typename T>
class MemoizerCache final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MemoizerCache)

  MemoizerCache() = default;
  ~MemoizerCache() = default;

  /* If key already has a value stored, then returns it. Otherwise, calls
  make_value(), stores it, and returns it. If multiple threads request a missing
  key simultaneously, make_value() may be called multiple times; however, only
  one result will be cached. */
  template <typename Callable>
  std::shared_ptr<const T> FindOrInsert(const Key& key,
                                        Callable&& make_value) const {
    // If the table maps the key to an existing value, return it.
    {
      std::shared_lock read_lock(impl_->mutex);
      if (std::shared_ptr<const T> result = FindWhileAlreadyLocked(key)) {
        return result;
      }
    }

    // We need to construct and insert the new value. To reduce contention, be
    // careful to construct the new value *without* holding any lock. This has
    // the potential for duplicate work in case two threads try to memoize the
    // same thing at the same time, but that's a fair trade-off. Only one winner
    // will reach the cache, the loser will re-find and alias the winner.
    T value = make_value();

    {
      std::unique_lock write_lock(impl_->mutex);
      // Check again in case a different thread snuck it into the table while
      // our make_value() was running or while we were waiting for the lock.
      if (std::shared_ptr<const T> result = FindWhileAlreadyLocked(key)) {
        return result;
      }

      // Still not there. Move the `value` to be owned by a `struct Reaper`
      // shared pointer. It retains a weak_ptr to our Impl to that the table
      // entry can be removed when the last reference to it disappears.
      auto reaper = std::make_shared<Reaper>(key, std::move(value), impl_);

      // Use shared_ptr's aliasing constructor so the pointed-to object is the
      // `T value` inside of `struct Reaper`, but the managed object is still
      // the entire reaper.
      T* value_raw = &(reaper->value);
      std::shared_ptr<const T> result(std::move(reaper), value_raw);

      // Insert the result into the cache (as a weak_ptr) and return it.
      impl_->table.emplace(key, result);
      return result;
    }
  }

  /* Returns a copy of the entire memoized table. */
  std::unordered_map<Key, std::shared_ptr<const T>> Dump() const {
    std::unordered_map<Key, std::shared_ptr<const T>> result;
    std::shared_lock read_lock(impl_->mutex);
    result.reserve(impl_->table.size());
    for (const auto& [key, value] : impl_->table) {
      result.emplace(key, value.lock());
    }
    return result;
  }

 private:
  /* Looks up `key` in our table and returns its associated value, or nullptr
  when not found.
  @pre The impl_->mutex is already locked. */
  std::shared_ptr<const T> FindWhileAlreadyLocked(const Key& key) const {
    auto iter = impl_->table.find(key);
    if (iter != impl_->table.end()) {
      return iter->second.lock();
    }
    return {};
  }

  struct Impl {
    // All access to the `table` (but not the contained T's) is guarded by this
    // `mutex`. All MemoizerCache methods that touch the `table` must hold
    // either a reader or writer lock on this `mutex`.
    std::shared_mutex mutex;

    // Maps key → value. We use a weak_ptr here because we want our users (who
    // call FindOrInsert) to govern the lifetime of the cached entries. Only
    // values that are still retained by our users should remain in memory. The
    // weak_ptr allow us to recover the value in case a user still has it, while
    // still cleaning up after the last user finishes using their shared_ptr.
    std::unordered_map<Key, std::weak_ptr<const T>> table;
  };

  struct Reaper {
    ~Reaper() {
      if (std::shared_ptr<Impl> impl = owner.lock()) {
        std::unique_lock write_lock(impl->mutex);
        auto& table = impl->table;
        const auto iter = table.find(key);
        if (iter != table.end()) {
          // Only erase if actually empty, to avoid a potential race condition
          // where the key was re-cache while we were waiting for the lock.
          if (iter->second.expired()) {
            table.erase(iter);
          }
        }
      }
    }

    Key key;
    T value;
    std::weak_ptr<Impl> owner;
  };

  // This is never actually shared (its use_count is always 1), but we need
  // shared_ptr so that the Reaper can have a weak_ptr.
  std::shared_ptr<Impl> impl_{std::make_shared<Impl>()};
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
