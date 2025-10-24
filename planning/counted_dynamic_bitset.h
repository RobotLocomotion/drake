#pragma once

#include <memory>
#include <span>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/reset_after_move.h"

namespace drake {
namespace planning {
namespace internal {

/* CountedDynamicBitset represents something like a `std::array<bool, N>` but
with a few customizations:
- the container size is determined at construction time instead of compile time;
- the count of values set to `true` is pre-computed and stored;
- convenient set-logic operations (complement, union, intersect, subtract). */
class CountedDynamicBitset {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(CountedDynamicBitset);

  /* Default constructor. Creates a bitset with size() = count() = 0. */
  CountedDynamicBitset() = default;

  /* Homogenous constructor. Creates a bitset with the given `size` where all
  values are set to the given `value`.
  @throws std::exception unless size >= 0 */
  CountedDynamicBitset(int size, bool value);

  /* Span constructor. Creates a bitset from a span of bools. */
  explicit CountedDynamicBitset(std::span<const bool> values);

  /* Returns this bitset's size. */
  int size() const { return size_; }

  /* Returns this bitset's count of values set to `true`. */
  int count() const { return count_; }

  /* Compares two bitsets for equality. Note that `other.size()` may differ from
  `this->size()`; such comparisons will report as not equal. */
  bool operator==(const CountedDynamicBitset& other) const;

  /* Returns the value at the given `index`.
  @pre `index` is in the range [0, size()). */
  bool operator[](int index) const {
    DRAKE_ASSERT(index >= 0 && index < size_);
    return buffer_[index];
  }

  /* Returns a new bitset with `true` values replaced with `false` and vice
  versa. The size is unchanged. */
  [[nodiscard]] CountedDynamicBitset Complement() const;

  /* Returns a new bitset with the union of `this` and `other`. The i'th bit is
  `true` iff that bit in either `this` or `other` is `true`.
  @pre `size() == other.size()`. */
  [[nodiscard]] CountedDynamicBitset Union(
      const CountedDynamicBitset& other) const;

  /* Returns a new bitset with the intersection of `this` and `other`. The i'th
  bit is `true` iff that bit in both `this` and `other` are `true`.
  @pre `size() == other.size()`. */
  [[nodiscard]] CountedDynamicBitset Intersect(
      const CountedDynamicBitset& other) const;

  /* Returns a new bitset with the set difference of `this` and `other`. The
  i'th bit is `true` iff that bit is `true` in `this` and `false` in `other`.
  @pre `size() == other.size()`. */
  [[nodiscard]] CountedDynamicBitset Subtract(
      const CountedDynamicBitset& other) const;

 private:
  CountedDynamicBitset(int size, int count) : size_(size), count_(count) {}

  /* Returns a view of `buffer_`.
  @pre buffer != nullptr */
  std::span<const bool> as_span() const {
    return std::span<const bool>(buffer_.get(), size_);
  }

  void CheckInvariants() {
    DRAKE_ASSERT(size_ >= 0);
    DRAKE_ASSERT(count_ <= size_);
    DRAKE_ASSERT((buffer_.get() == nullptr) == (size_ == 0));
  }

  reset_after_move<int> size_{0};
  reset_after_move<int> count_{0};
  std::unique_ptr<bool[]> buffer_{nullptr};
};

// The move operations are cheap so are defined inline. The copy operations are
// potentially expensive for larger bitsets, so are defined in the cc file.

inline CountedDynamicBitset::CountedDynamicBitset(
    CountedDynamicBitset&& other) = default;

inline CountedDynamicBitset& CountedDynamicBitset::operator=(
    CountedDynamicBitset&& other) = default;

}  // namespace internal
}  // namespace planning
}  // namespace drake
