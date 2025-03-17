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

In some cases the signal names are used only for human-readable logging or
debugging, so can be anything. In other cases, the systems using the signals
will require the signals to use specific names per some convention (e.g.,
for a BusSelector the signal names must match the output port names).

@see BusCreator, BusSelector */
class BusValue final {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(BusValue);

  /** Constructs an empty %BusValue. */
  BusValue();

  ~BusValue();

  /** @name Iterators
  The iteration order is deterministic but unspecified. */
  /** @{ */
#ifndef DRAKE_DOXYGEN_CXX
  class Iterator;
#endif
  Iterator begin() const;
  Iterator end() const;
  /** @} */

  /** @name Container-like type aliases */
  /** @{ */
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
  string without restriction, although we encourage valid UTF-8.

  @warning Within a group of BusValue objects that are expected to inter-operate
  (i.e., to be copied or assigned to each other), the type of the `value` for a
  given `name` is expected to be consistent (i.e., homogenous) across the entire
  group of objects. After setting a `name` to some value the first time, every
  subsequent call to Set on the same BusValue object for that same `name` must
  provide a value of the same type, even if the object has since been cleared or
  copied onto another object. The only way to reset the hysteresis for the
  "presumed type" of a name is to construct a new BusValue object. Failure to
  keep the types consistent may result in an exception at runtime. However, we
  might relax this restriction in the future, so don't count on it for error
  handling. */
  void Set(std::string_view name, const AbstractValue& value);

 private:
  class Impl;

  // N.B. This is allowed to be nullptr (when the bus is empty).
  // TODO(#13591) For efficiency this should probably be copy-on-write?
  copyable_unique_ptr<Impl> impl_;
};

/** Provides a forward_iterator over BusValue signals.
The iteration order is deterministic but unspecified. */
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

// We define this function the header file for performance, but we can't do it
// inside the BusValue class definition because Iterator is a nested class, so
// we need to place the definition down here after both class definitions.
inline BusValue::Iterator BusValue::end() const {
  return BusValue::Iterator{};
}

}  // namespace systems
}  // namespace drake
