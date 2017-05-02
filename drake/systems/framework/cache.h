#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

typedef int CacheTicket;

namespace internal {

/// A single cached piece of data, its validity bit, and the set of other cache
/// entries that depend on it.
class CacheEntry {
 public:
  CacheEntry();
  ~CacheEntry();

  // Implements CopyConstructible and CopyAssignable directly, and
  // MoveConstructible and MoveAssignable indirectly via copying.
  CacheEntry(const CacheEntry& other);
  CacheEntry& operator=(const CacheEntry& other);

  bool is_valid() const { return is_valid_; }
  void set_is_valid(bool valid) { is_valid_ = valid; }

  AbstractValue* value() const { return value_.get(); }

  void set_value(std::unique_ptr<AbstractValue> value) {
    value_ = std::move(value);
  }

  std::unique_ptr<AbstractValue> release_value() {
    set_is_valid(false);
    return std::move(value_);
  }

  const std::set<CacheTicket>& dependents() const { return dependents_; }
  void add_dependent(CacheTicket ticket) { dependents_.insert(ticket); }

 private:
  bool is_valid_{false};
  std::unique_ptr<AbstractValue> value_;

  // The set of cache tickets that are invalidated when the ticket corresponding
  // to this entry is invalidated. This graph is directed and acyclic.
  std::set<CacheTicket> dependents_;
};

}  // namespace internal

/// Cache is a key-value store used within the System2 framework to avoid
/// computing intermediate data multiple times during simulation or analysis.
/// It is not a general-purpose caching layer.
///
/// Every Cache will be private to a System, stored within and accessible via
/// the System's Context. Its function is to return a previously-computed
/// value X, so long as no other value upon which X depends has changed.
/// The System declares the expensive values it will compute, and their
/// dependencies, by calling MakeCacheTicket.  It provides type-erased
/// storage for the value using Init - typically just once - and thereafter
/// sets the value using Set, or retrieves it using Get.
///
/// Whenever Init or Set is called, all entries which depend on the
/// ticket being written are "invalidated". Once a ticket is invalidated,
/// calls to Get for that ticket will return nullptr until it itself is
/// written again.  For this reason, a System must call Get whenever it
/// needs access to a cached value; it must not hold the returned pointer.
///
/// Cache is not thread-safe. It is copyable, assignable, and movable.
class Cache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Cache)

  Cache();
  ~Cache();

  /// Creates a new cache entry of type `EntryType` and returns the new ticket
  /// to it, marking the entry itself as **invalid**. This entry will
  /// be invalidated whenever any of the @p prerequisites are invalidated.
  /// As an example of usage consider the code below:
  ///
  /// @code
  ///   Cache<double> cache;
  ///   CacheTicket foos_ticket =
  ///       cache.MakeCacheEntry<Foo<double>>(
  ///           {ticket1, ticket2}, /* Entry prerequisites. */
  ///           "name", 3.14);      /* Foo<double>'s constructor parameters. */
  /// @endcode
  ///
  /// @param[in] prerequisites A list of cache tickets corresponding to the
  ///                          cache entries the newly created entry depends on.
  /// @param[in] args The list of arguments to EntryType's constructor.
  ///
  /// @tparam EntryType The type of the cache entry to be created. It must be
  ///                   copy-constructible and assignable.
  template<class EntryType, typename... Args>
  CacheTicket MakeCacheEntry(const std::set<CacheTicket>& prerequisites,
                             Args&&... args) {
    CacheTicket ticket = MakeCacheTicket(prerequisites);
    store_[ticket].set_value(
        std::make_unique<Value<EntryType>>(std::forward<Args>(args)...));
    store_[ticket].set_is_valid(false);
    return ticket;
  }

  /// Creates a new cache ticket, which will be invalidated whenever any of
  /// the @p prerequisites are invalidated. The new cache entry is marked as
  /// **invalid**.
  CacheTicket MakeCacheTicket(const std::set<CacheTicket>& prerequisites);

  /// Invalidates the value for @p ticket, and all entries that depend on it.
  void Invalidate(CacheTicket ticket);

  /// Validates the cache entry corresponding to the provided @p ticket.
  /// In order to make use of the automatic validation capability of cache
  /// entries provided by %Cache, users should use Set() whenever copies of the
  /// particular entry type are cheap to perform since Set() validates the entry
  /// being modified and invalidates dependents automatically. However, in many
  /// cases cache entries are large complex data structures and it might be more
  /// convenient to first retrieve a mutable entry with GetMutable() (which
  /// automatically invalidates the requested entry and its dependents), make
  /// the necessary updates to the entry and finally, validate it with a call to
  /// this method.
  ///
  /// @warning Only advanced, careful users should call this method since
  /// validating cache entries by hand can be error prone. Use with care.
  void validate(CacheTicket ticket);

  /// Returns `true` if the cache entry referenced to by @p ticket is valid.
  /// Returns `false` otherwise.
  bool is_entry_valid(CacheTicket ticket) {
    return store_[ticket].is_valid();
  }

  /// Takes ownership of a cached item, and returns a bare pointer to the item.
  /// Marks the entry itself as valid, and invalidates all entries that depend
  /// on it.
  ///
  /// The bare pointer may be used to modify the entry immediately, but should
  /// not be held, because it may become invalid if the ticket's prerequisites
  /// are modified.  It will only actually be deleted on subsequent calls to
  /// Init for this ticket, or on the deletion of the Cache itself;
  /// however, only advanced, careful users should rely on that behavior.
  AbstractValue* Init(CacheTicket ticket,
                      std::unique_ptr<AbstractValue> value);

  /// Sets a cache entry to the given @p value. Aborts if the value has not
  /// already been initialized. May throw std::bad_cast if the value has been
  /// initialized with a different type. Marks the entry itself as valid, and
  /// invalidates all entries that depend on it.
  ///
  /// @tparam T The type of the value.
  template <typename T>
  void Set(CacheTicket ticket, const T& value) {
    DRAKE_DEMAND(ticket >= 0 && ticket < static_cast<int>(store_.size()));

    AbstractValue* entry = store_[ticket].value();
    DRAKE_DEMAND(entry != nullptr);
    entry->SetValue<T>(value);
    store_[ticket].set_is_valid(true);
    InvalidateRecursively(store_[ticket].dependents());
  }

  /// Returns the cached item for the given @p ticket, or nullptr if the item
  /// has been invalidated.
  ///
  /// @warning The bare pointer should not be held, because the data may become
  /// invalid if the ticket's prerequisites are modified.
  const AbstractValue* Get(CacheTicket ticket) const;

  /// Returns the mutable cached item for the entry referenced to by @p ticket,
  /// invalidating this entry itself and recursively invalidating all of its
  /// dependents.
  ///
  /// @warning The bare pointer should not be held, because the data may become
  /// invalid if the ticket's prerequisites are modified.
  ///
  /// @warning It is responsability of the caller to call validate() on this
  /// entry once done with the necessary updates. Failure to do so will have no
  /// effect on the final result of the computation, but it might have an impact
  /// on efficiency since the entry will be recomputed every time is accessed.
  AbstractValue* GetMutable(CacheTicket ticket);

 private:
  // Invalidates all tickets that depend on the tickets in @p to_invalidate.
  void InvalidateRecursively(const std::set<CacheTicket>& to_invalidate);

  // For each cache ticket, the stored value, and its validity bit.
  std::vector<internal::CacheEntry> store_;
};

}  // namespace systems
}  // namespace drake
