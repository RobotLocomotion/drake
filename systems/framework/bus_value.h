#pragma once

#include <cstddef>
#include <string_view>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/value.h"

namespace drake {
namespace systems {

/** %BusValue is a value type used on input ports and output ports to group
labeled signals into a single port. Each signal is referred to by a unique
name and stored using an AbstractValue.
@see BusCreator, BusSelector */
class BusValue final {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(BusValue);

  /** Constructs an empty %BusValue. */
  BusValue();

  ~BusValue();

  /** @name Provides a forward_iterator over BusValue signals. */
  /** @{ */
  class Iterator;
  Iterator begin() const;
  Iterator end() const;
  using const_iterator = Iterator;
  using value_type = std::pair<const std::string_view, const AbstractValue&>;
  /** @} */

  /** Gets one signal value. Returns nullptr if not found.
  Does not invalidate any iterators, but the return value is invalidated by a
  call to any non-const method on this. */
  const AbstractValue* Find(std::string_view name) const;

  /** Removes all signals from this. Invalidates all iterators. */
  void Clear();

  /** Sets one signal value. Invalidates all iterators. The `name` can be any
  string without restriction, although we encourage valid UTF-8. */
  void Set(std::string_view name, const AbstractValue& value);

 private:
  class Impl;

  // N.B. This is allowed to be nullptr (when the bus is empty).
  // TODO(#13591) For efficiency this should probably be copy-on-write?
  copyable_unique_ptr<Impl> impl_;
};

/** Provides a forward_iterator over BusValue signals. */
class BusValue::Iterator {
 public:
  using difference_type = std::ptrdiff_t;
  using value_type = typename BusValue::value_type;

  Iterator() = default;
  value_type operator*() const;
  Iterator& operator++();
  Iterator operator++(int);
  bool operator==(const Iterator& other) const {
    return index_ == other.index_;
  }

 private:
  friend class BusValue;

  // These default values denote the end() sentinel value.
  size_t index_{~size_t{0}};
  const void* impl_{nullptr};
};

inline BusValue::Iterator BusValue::end() const {
  return BusValue::Iterator{};
}

}  // namespace systems
}  // namespace drake
