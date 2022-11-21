/* clang-format off to disable clang-format-includes */
// NOLINTNEXTLINE(build/include): Our header file is included by all.h.
#include "drake/common/symbolic/expression/all.h"
/* clang-format on */

#define DRAKE_COMMON_SYMBOLIC_EXPRESSION_DETAIL_HEADER
#include "drake/common/symbolic/expression/expression_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_EXPRESSION_DETAIL_HEADER

namespace drake {
namespace symbolic {
namespace internal {

void BoxedCell::ConstructCopy(const BoxedCell& other) {
  DRAKE_ASSERT(!other.is_constant());
  SetSharedCell(&other.cell());
}

void BoxedCell::AssignCopy(const BoxedCell& other) {
  if (this == &other) {
    return;
  }
  // Decrement the use_count of our currently-managed cell.
  Release();
  if (other.is_constant()) {
    value_ = other.value_;
  } else {
    SetSharedCell(&other.cell());
  }
}

void BoxedCell::SetSharedCell(const ExpressionCell* cell) noexcept {
  DRAKE_ASSERT(cell != nullptr);
  DRAKE_ASSERT(is_constant());

  // Convert the `cell` pointer to a bit pattern.
  // https://en.cppreference.com/w/cpp/language/reinterpret_cast (case 2)
  uintptr_t bit_buffer = reinterpret_cast<uintptr_t>(cell);

  // Compute (and check) the tag.
  const uint16_t tag = static_cast<uint16_t>(cell->get_kind());
  // It must have the NaN bits set.
  DRAKE_ASSERT((tag & 0x7FF0) == 0x7FF0);
  // It must NOT have a zero in the low nibble. Otherwise, we could confuse
  // the tag with +Infinity (0x7FF0...) or -Infinity (0x8FF0...).
  DRAKE_ASSERT((tag & 0x000F) != 0);

  // Set the upper 16 bits of the point to the tag (i.e., the kind). They
  // must have already been zero (due the limit of 48-bit addressable physical
  // address space on our supported platforms).
  DRAKE_ASSERT((bit_buffer & (0xFFFFull << 48)) == 0);
  bit_buffer |= static_cast<uintptr_t>(tag) << 48;

  // Bit-cast it into a double.
  static_assert(sizeof(value_) == sizeof(bit_buffer));
  std::memcpy(&value_, &bit_buffer, sizeof(value_));
  DRAKE_ASSERT(std::isnan(value_));

  // Claim a use of the cell.
  ++(cell->use_count());
}

void BoxedCell::Release() noexcept {
  if (!is_constant()) {
    const auto& owned = cell();
    const int new_use_count = --owned.use_count();
    if (new_use_count == 0) {
      delete &owned;
    }
  }
  value_ = 0.0;
}

}  // namespace internal
}  // namespace symbolic
}  // namespace drake
