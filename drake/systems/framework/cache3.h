#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "drake/systems/framework/value.h"
#include "drake/systems/framework/value_listener3.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/** Each cache entry contains:
 - An AbstractValue containing the actual result.
 - An Evaluator function that knows how to compute the result when given a
   System and a compatible Context.
 - A bool indicating whether the result is up to date with respect to
   its prerequisites in the same Context.
 - A list of dependents ("listeners") that need to be notified when the cache
   entry is marked invalid.

A cache entry is both a prerequisite for its downstream value listeners, and
a value listener registered with its upstream prerequisites. **/

// TODO(sherm1) Consider how to define "up to date" for a sampled quantity;
//              seems like it should still be up to date if its next sample
//              time hasn't arrived yet, even if prereqs have changed.
class CacheEntry : public ValueListenerInterface {
 public:
  /** This is the type of a function that unconditionally computes this cache
  entry's value with respect to the contents of a given Context. **/
  using Calculator =
      std::function<void(const class AbstractSystem3&,
                         const class AbstractContext3&, AbstractValue*)>;

  /** Create an empty cache entry that is not yet committed to a particular
  kind of abstract value. **/
  CacheEntry() {}

  /** Create a cache entry initialized to a copy of the given `value`. **/
  explicit CacheEntry(const AbstractValue& value) : value_(value.Clone()) {}

  /** Create a cache entry that takes over ownership of the given `value`. **/
  explicit CacheEntry(std::unique_ptr<AbstractValue> value) noexcept
      : value_(std::move(value)) {}

  /** Copy constructor makes a new cache entry that has a copy of the value
  from `source`, but is marked "not current" regardless of whether `source`
  is current. **/
  CacheEntry(const CacheEntry& source)
      : CacheEntry(source.get_abstract_value()) {}

  /** Move constructor moves the value and preserves the `is_current` status
  of the source in the destination. The source is left empty and not current.
  **/
  CacheEntry(CacheEntry&& source) noexcept
      : value_(std::move(source.value_)), is_current_(source.is_current_) {
    source.set_is_current(false);
  }

  /** Set the function to be used when we need to recompute this cache entry's
  value. **/
  void set_calculator(const Calculator& calculator) {
    calculate_ = calculator;
  }

  /** Set the function to be used when we need to recompute this cache entry's
  value. **/
  const Calculator& get_calculator() const { return calculate_; }

  /** Set the AbstractValue type stored in this %CacheEntry by replacing the
  existing one (if any) with the supplied one. The %CacheEntry takes ownership
  of the AbstractValue object. **/
  void ResetAbstractValue(std::unique_ptr<AbstractValue> value) {
    value_ = std::move(value);
    set_is_current(false);
  }

  /** Set the AbstractValue type stored in this %CacheEntry by replacing the
  existing one (if any) with a copy of the supplied one. **/
  void ResetAbstractValue(const AbstractValue& value) {
    ResetAbstractValue(value.Clone());
  }

  void RealizeCacheEntry(const AbstractSystem3& system,
                         const AbstractContext3& context) {
    // TODO(sherm1) remove when invalidation logic is working
    set_is_current(false);
    //////////////////////

    if (is_current()) return;
    get_calculator()(system, context, get_mutable_abstract_value());
    set_is_current(true);
  }

  /** Return the value of the `is_current` flag for this cache entry. **/
  bool is_current() const { return is_current_; }

  /** Mark this entry as current or not; up to caller to do this right. **/
  void set_is_current(bool is_current) { is_current_ = is_current; }

  /** Return a const reference to the AbstractValue contained in this
  cache entry, *regardless* of whether it is current. **/
  const AbstractValue& get_abstract_value() const { return *value_; }

  /** Return a mutable pointer to the AbstractValue contained in this
  cache entry, *regardless* of whether it is current. The value is returned 
  *regardless* of whether it is current.**/
  AbstractValue* get_mutable_abstract_value() {
    return value_.get();
  }

  /** Return a const reference to the value contained in this cache entry,
  which must be of type `Type`. The value is returned *regardless* of whether it
  is current.
  @throws std::logic_error The value did not have type `Type`. **/
  template <class Type>
  const Type& get_value() const {
    const AbstractValue& abstract_value = get_abstract_value();
    return abstract_value.GetValue<Type>();
  }

  /** Return a mutable pointer to the value contained in this cache entry,
  which must be of type `Type`. The value is returned *regardless* of whether it
  is current. No invalidation or notification occurs.
  @throws std::logic_error The value did not have type `Type`. **/
  template <class Type>
  Type* get_mutable_value() {
    AbstractValue* abstract_value = get_mutable_abstract_value();
    return abstract_value->GetMutableValue<Type>();
  }

  template <typename T>
  const VectorInterface<T>& get_vector_value() const {
    const VectorObject<T>& object = get_value<VectorObject<T>>();
    return object.get_vector();
  }

  template <typename T>
  const VectorInterface<T>* get_mutable_vector_value() const {
    VectorObject<T>* object = get_mutable_value<VectorObject<T>>();
    return object.get_mutable_vector();
  }

  // Implement ValueListenerInterface

  /** Invalidate this cache entry and notify any downstream listeners. The
  value version number is incremented. **/
  void Invalidate() override { 
    set_is_current(false);
    ++version_;
    listeners_.NotifyListeners();
  }

 private:
  std::unique_ptr<AbstractValue> value_;

  Calculator calculate_;

  // TODO(sherm1) The rest of these members should be reset_on_copy<T> to allow
  // default constructors.

  // Whether this entry is up to date with respect to its prerequisites.
  bool is_current_{false};

  // Who needs to be notified when this entry's value changes?
  ValueListenerList listeners_;

  // A counter that is bumped every time this entry's value changes.
  long long version_{0};
};
//
///** This is a mutable object that resides in a Context and serves as memory
//for computed results so that they do not have to be recomputed if already
//valid. It contains a set of CacheEntry objects, each representing a single
//value. The values may be of any type, however if they are numerical values
//then they must use scalar type `T`.
//
//@tparam T The type of numerical values in this %Cache. Must be a
//          valid Eigen scalar. **/
//template <typename T>
//class Cache {
// public:
//  /** Construct a `Cache` containing no cache entries. **/
//  Cache() {}
//  ~Cache() = default;
//
//  /** Return the number of entries `n` currently in this %Cache. Entries are
//  numbered in the order added from 0 to `n-1`. **/
//  int get_num_entries() const { return (int)entries_.size(); }
//
//  /** Add a cache entry by moving the contents of the given `entry` into the
//  Cache. The source entry is left empty.
//  @retval entry_num The assigned entry number for retrieval. **/
//  int add_entry(CacheEntry&& entry) {
//    entries_.push_back(std::move(entry));
//    return (int)entries_.size() - 1;
//  }
//
//  /** Add a cache entry that is a copy of the given `entry`.
//  @retval entry_num The assigned entry number for retrieval. **/
//  int add_entry(const CacheEntry& entry) {
//    return add_entry(CacheEntry(entry));  // use move signature
//  }
//
//  /** Retrieve a const reference to a cache entry using the `entry_num` that
//  was returned when the entry was added. **/
//  const CacheEntry& get_entry(int entry_num) const {
//    return entries_[entry_num];
//  }
//
//  /** Retrieve a mutable pointer to a cache entry using the `entry_num` that
//  was returned when the entry was added. **/
//  CacheEntry* get_mutable_entry(int entry_num) { return &entries_[entry_num]; }
//
//  // Cache objects are copyable with the default copy constructor.
//  Cache(const Cache& other) = default;
//  Cache& operator=(const Cache& other) = default;
//
// private:
//  // Cache objects are not moveable.
//  Cache(Cache&& other) = delete;
//  Cache& operator=(Cache&& other) = delete;
//
//  std::vector<CacheEntry> entries_;
//};

}  // namespace systems
}  // namesapce drake
