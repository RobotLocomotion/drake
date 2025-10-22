#include "drake/planning/counted_dynamic_bitset.h"

#include <algorithm>
#include <cstring>
#include <utility>

namespace drake {
namespace planning {
namespace internal {

namespace {

// Copies a span of bools into a new unique_ptr<bool[]>.
std::unique_ptr<bool[]> MakeBoolArray(std::span<const bool> values) {
  // TODO(jwnimmer-tri) Use make_unique_for_overwrite once we have >= C++23.
  std::unique_ptr<bool[]> result(new bool[values.size()]);
  std::copy(values.begin(), values.end(), result.get());
  return result;
}

// Broadcasts a single bool into a new unique_ptr<bool[]>.
// @pre size >= 0
std::unique_ptr<bool[]> MakeBoolArray(int size, bool value = false) {
  DRAKE_ASSERT(size >= 0);
  // TODO(jwnimmer-tri) Use make_unique_for_overwrite once we have >= C++23.
  std::unique_ptr<bool[]> result(new bool[size]);
  std::fill_n(result.get(), size, value);
  return result;
}

// Copies the bool array at `other` into its complement (negation).
// @pre size >= 0
std::unique_ptr<bool[]> MakeComplement(std::span<const bool> other) {
  // TODO(jwnimmer-tri) Use make_unique_for_overwrite once we have >= C++23.
  std::unique_ptr<bool[]> result(new bool[other.size()]);
  for (size_t i = 0; i < other.size(); ++i) {
    result[i] = !other[i];
  }
  return result;
}

// The template argument for Merge(), immediately below.
enum BinaryOperator {
  kUnion,
  kIntersect,
  kSubtract,
};

// Returns a new bool array computed as a binary operator over the equally-
// sized `a` and `b` input arrays.
// @pre size >= 0
template <BinaryOperator operation>
__attribute__((noinline)) std::pair<std::unique_ptr<bool[]>, int /* count */>
Merge(int size, const bool* a, const bool* b) {
  DRAKE_ASSERT(size >= 0);
  if (!(size >= 0)) __builtin_unreachable();
  if (size == 0) {
    return std::make_pair(/* buffer = */ std::unique_ptr<bool[]>{},
                          /* count = */ 0);
  }
  // TODO(jwnimmer-tri) Use make_unique_for_overwrite once we have >= C++23.
  std::unique_ptr<bool[]> buffer(new bool[size]);
  int count = 0;
  for (int i = 0; i < size; ++i) {
    int bit;
    if constexpr (operation == kUnion) {
      bit = int{a[i]} | int{b[i]};
    } else if constexpr (operation == kIntersect) {
      bit = int{a[i]} & int{b[i]};
    } else if constexpr (operation == kSubtract) {
      bit = int{a[i]} & ~int{b[i]};
    } else {
      DRAKE_UNREACHABLE();
    }
    buffer[i] = (bit != 0);
    count += bit;
  }
  return std::make_pair(std::move(buffer), count);
}

}  // namespace

CountedDynamicBitset::CountedDynamicBitset(int size, bool value) {
  DRAKE_THROW_UNLESS(size >= 0);
  size_ = size;
  count_ = value ? size : 0;
  if (size_ > 0) {
    buffer_ = MakeBoolArray(size, value);
  }
  CheckInvariants();
}

CountedDynamicBitset::CountedDynamicBitset(std::span<const bool> values) {
  size_ = ssize(values);
  if (size_ > 0) {
    buffer_ = MakeBoolArray(values);
    count_ = std::count(values.begin(), values.end(), true);
  }
  CheckInvariants();
}

// We can't use the defaulted implementation because of our unique_ptr member.
// However, for simplicity we can just delegate to the copy-assignment operator.
CountedDynamicBitset::CountedDynamicBitset(const CountedDynamicBitset& other) {
  *this = other;
  CheckInvariants();
}

// We can't use the defaulted implementation because of our unique_ptr member.
CountedDynamicBitset& CountedDynamicBitset::operator=(
    const CountedDynamicBitset& other) {
  if (this == &other) [[unlikely]] {
    return *this;
  }
  size_ = other.size();
  count_ = other.count();
  if (size_ > 0) {
    buffer_ = MakeBoolArray(other.as_span());
  } else {
    buffer_.reset();
  }
  CheckInvariants();
  return *this;
}

CountedDynamicBitset CountedDynamicBitset::Complement() const {
  CountedDynamicBitset result(
      /* size = */ size(),
      /* count = */ size() - count());
  if (size() > 0) {
    result.buffer_ = MakeComplement(this->as_span());
  }
  result.CheckInvariants();
  return result;
}

CountedDynamicBitset CountedDynamicBitset::Union(
    const CountedDynamicBitset& other) const {
  DRAKE_ASSERT(this->size() == other.size());
  CountedDynamicBitset result(size(), /* count = */ 0);
  if (size() > 0) {
    std::tie(result.buffer_, result.count_) =
        Merge<kUnion>(size(), buffer_.get(), other.buffer_.get());
  }
  result.CheckInvariants();
  return result;
}

CountedDynamicBitset CountedDynamicBitset::Intersect(
    const CountedDynamicBitset& other) const {
  DRAKE_ASSERT(this->size() == other.size());
  CountedDynamicBitset result(size(), /* count = */ 0);
  std::tie(result.buffer_, result.count_) =
      Merge<kIntersect>(size(), buffer_.get(), other.buffer_.get());
  result.CheckInvariants();
  return result;
}

CountedDynamicBitset CountedDynamicBitset::Subtract(
    const CountedDynamicBitset& other) const {
  DRAKE_ASSERT(this->size() == other.size());
  CountedDynamicBitset result(size(), /* count = */ 0);
  std::tie(result.buffer_, result.count_) =
      Merge<kSubtract>(size(), buffer_.get(), other.buffer_.get());
  result.CheckInvariants();
  return result;
}

bool CountedDynamicBitset::operator==(const CountedDynamicBitset& other) const {
  if (this->size() != other.size()) {
    return false;
  }
  bool result = (std::memcmp(buffer_.get(), other.buffer_.get(), size()) == 0);
  DRAKE_ASSERT((result == false) || (this->count() == other.count()));
  return result;
}

}  // namespace internal
}  // namespace planning
}  // namespace drake
