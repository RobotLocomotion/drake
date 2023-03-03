#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_EXPRESSION_ALL
#error Do not include this file. Use "drake/common/symbolic/expression.h".
#endif

#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

#include "drake/common/drake_assert.h"

namespace drake {
namespace symbolic {

class ExpressionCell;

namespace internal {

/* BoxedCell is an internal class that provides an extremely efficient
storage format for the data inside an Expression.

Conceptually, it stores three pieces of information:

(1) `ExpressionKind kind` -- the operation (e.g., add, mul, cos, max, ...).

(2) `double value` -- the constant value iff `kind == ExpressionKind::Constant`.

(3) `shared_ptr<ExpressionCell> cell` -- a copy-on-write, heap-allocated object
that stores the details of the expression operation (e.g., the terms to sum).
This is only used for expressions with `kind != ExpressionKind::Constant`; for
constants, the cell is conceptually nullptr.

In a naive implementation, those three fields would consume 2 + 8 + 16 == 26
bytes (which the compiler will often round up to 32). The crux of this class is
to represent that information using only 8 bytes (as a `double value_;`), with a
trick called "nanboxing".

Refer to https://piotrduperas.com/posts/nan-boxing for good overview of the
technique. Definitely read the background material before proceeding through the
rest of this overview!

In the following sections, we'll summarize the reasons we're using nanboxing,
recap the highlights of the technique, and discuss the specific implementation
choices used within BoxedCell.

===== Speed-ups =====

(1) Information density.

We use nanboxing in part for the typical payoff -- packing a lot of information
into a tiny space and avoiding pointer-chasing for literal values.

(2) Speculative arithmetic.

For floating-point arithmetic in particular, representing pointers as NaNs
allows us to use _speculative_ arithmetic for even more throughput.

Consider the following math: x = a*b + c*d + e*f + g*h. If all of the values are
constants (i.e., have no Variables), then ideally we'd like to compute `x` using
just 7 flops (and at most 7 machine instructions, or perhaps fewer with SIMD).
Naively, we could first check all 8 inputs (a..h) for whether they are constant,
and if yes then do the flops, otherwise fall back to the slower, fully-symbolic
evaluation. (Or worse, check before every binary operator in which case we'd
need 14 checks.) Those 8 or 14 branches (>1 per flop) are not cheap, though!
Ideally we would like to see very few branches per flop on average, for best
throughput.

Since non-constants (cell pointers) are always NaN when interpreted as a double,
another way we can implement that math is to first compute the result `x` using
7 flops, and then _afterwards_ check only `x` for NaN instead of all of the the
inputs. (Per IEEE754, any operation on NaN input always produces a NaN output.)
If `x` ends up non-NaN, that means none of the inputs could have been NaN, which
means none of the inputs could have been variables, and we're done! One branch
per 7 flops. In case `x` did end up as NaN, then we throw away the speculative
result and branch to the fully-symbolic evaluation.

Note that with how Expression is implemented today in terms of binary operations
that run one by one, this technique is only used to partial effect. We still end
up checking for some intermediate NaNs, just not as many. To take full effect,
we will need to specialize Eigen's matrix-operation functors.

Note that `x` could end up NaN even when a..h were all constants (e.g., when
subtracting infinity from itself, or dividing by zero). This is a rare case,
so we're satisfied to use the slower symbolic codepath to handle it.

(3) Fast pattern matching.

The implementation also provides some clever functions such as constant_or_nan()
or is_kind<ExpressionKind>() that allow for efficient machine code for the most
frequent operations performed on an Expression.

In many cases (e.g., std::map), the Expression::Less and Expression::EqualTo
are called extremely many times, so it's critically important that they bail
out early in case the cell pointer or constant is the same, or the kind is
different.

===== The BoxedCell implementation and its representation choices =====

When the BoxedCell kind is a Constant, then its `value_` directly holds the
constant value. As bits, an IEEE754 64-bit double looks like
 SEEEEEEE EEEEMMMM MMMMMMMM MMMMMMMM MMMMMMMM MMMMMMMM MMMMMMMM MMMMMMMM
where S is the sign bit, E are the 11 exponent bits, and M are the 52 mantissa
bits.

When the E bits are all 1's, then the floating-point value is non-finite:
- If M are all zero, then the value is ±infinity (based on the sign bit).
- Otherwise, the value is NaN (no matter the sign bit).

Because Expression does not allow NaN values as a Constant (instead, there is a
cell type ExpressionNaN), we have the opportunity to re-purpose the meaning of
the NaN bit pattern as:
 x1111111 1111xxxx PPPPPPPP PPPPPPPP PPPPPPPP PPPPPPPP PPPPPPPP PPPPPPPP
where P is a 48-bit pointer value. Because the pointer cannot be null, we know
for sure that the value will be not mistaken for an infinity.

Note that x86_64 pointers only have 48 significant bits; see
 https://en.wikipedia.org/wiki/X86-64#Architectural_features
or
 https://www.amd.com/system/files/TechDocs/24593.pdf
for details. The same is true of the nominal AArch64 virtual addresses used on
macOS (running `sysctl machdep.virtual_address_size` reports 47 on Apple M1),
and we don't expect to ever compile with "pointer authentication" (i.e., "PAC")
enabled. (PAC lays claim to the upper bits of the pointer).

With more tricks, we can also store the 16-bit ExpressionKind inline:
 KKKKKKKK KKKKKKKK PPPPPPPP PPPPPPPP PPPPPPPP PPPPPPPP PPPPPPPP PPPPPPPP
as long as the definition of ExpressionKind has the NaN bits set:
 x1111111 1111xxxx
Indeed, we've set up ExpressionKind in just such a way. The upper 16 bits are
called the "pointer tag" or just "tag".

We have one final trick up our sleeve. Recall what an infinity looks like:
 S1111111 11110000 00000000 00000000 00000000 00000000 00000000 00000000
When Expression holds infinity, we need to have e.get_kind() == Constant.
We _could_ check all 64 bits for this pattern, but the x86_64 instruction for
that requires copying it into an xmm register, whereas Expressions are usually
manipulated within integer registers (the SystemV ABI passes them as pointers,
not in xmm registers). Instead, we'll make an invariant that no ExpressionKind
tag value is ever
 x1111111 11110000
which means that e.get_kind() can be fully determined by looking only at the 16
tag bits, not all 64 bits.

Specifically, if the exponent has a zero bit anywhere ...
 -------- --------
    tag_bits()
 -------- --------
 x0xxxxxx xxxxxxxx
 xx0xxxxx xxxxxxxx
 xxx0xxxx xxxxxxxx
 xxxx0xxx xxxxxxxx
 xxxxx0xx xxxxxxxx
 xxxxxx0x xxxxxxxx
 xxxxxxx0 xxxxxxxx
 xxxxxxxx 0xxxxxxx
 xxxxxxxx x0xxxxxx
 xxxxxxxx xx0xxxxx
 xxxxxxxx xxx0xxxx
... then the get_kind() is Constant (and finite).

Otherwise, if the lower four bits of the tag are all zero ...
 xxxxxxxx xxxx0000
... then the get_kind() is Constant (and infinite). Note that in this case
the exponent will necessarily have been all 1s ...
 x1111111 11110000
... but we don't need to check for that in the matching predicate, the zero
nibble is sufficient on its own. (This is actually a meaningful speed-up!)

Otherwise, the get_kind() is exactly the tag bits (and isn't Constant) ...
 KKKKKKKK KKKKKKKK
... keeping in mind that the middle bits are necessarily all 1s ...
 K1111111 1111KKKK
... but we don't need to check for that in the matching predicate.

Note that with this bit layout, we have used ~27 of the 31 available kinds
that fit within this design. In case we ever need more than 31, note that
the lower 3 bits of the pointer are always zero (because heap allocations
always happen on >= 8 bytes boundaries), so we could store more tag bits in
the lower 48-bit "pointer" value, either in the low-order bits directly, or
by right-shifting the pointer to make more room at the top.

A final note: the high-order bit of a NaN's mantissa is the "quiet" bit.
When set to 0, the value is a "signaling NaN" and in theory might raise a
floating-point trap when operated on. In practice, our supported platforms
ignore the signaling vs quiet distinction, so we haven't bothered trying
to keep it set to 1. */
class BoxedCell {
 public:
  /* Default constructor constructs zero. */
  BoxedCell() : BoxedCell(0.0) {}

  /* Constructs a constant. */
  explicit BoxedCell(double constant) : value_(constant) {
    DRAKE_ASSERT(!std::isnan(constant));
  }

  /* Copyable-constructible. */
  BoxedCell(const BoxedCell& other) {
    if (other.is_constant()) {
      value_ = other.value_;
    } else {
      ConstructCopy(other);
    }
  }

  /* Copyable-assignable. */
  BoxedCell& operator=(const BoxedCell& other) {
    if (is_constant() && other.is_constant()) {
      value_ = other.value_;
    } else {
      AssignCopy(other);
    }
    return *this;
  }

  /* Move-constructible. After the move, `other` will be zero. */
  BoxedCell(BoxedCell&& other) noexcept {
    // When other is not a Constant, transfers ownership of other.cell() to
    // this with no change to the cell's use_count.
    value_ = other.value_;
    other.value_ = 0;
  }

  /* Move-assignable. After the move, `other` will be zero. */
  BoxedCell& operator=(BoxedCell&& other) noexcept {
    if (!is_constant()) {
      // Decrement the use_count of our currently-managed cell.
      Release();
    }
    // When other is not a Constant, transfers ownership of other.cell() to
    // this with no change to the cell's use_count.
    value_ = other.value_;
    other.value_ = 0;
    return *this;
  }

  ~BoxedCell() {
    // This is required in order to decrement the use_count of our cell.
    Release();
  }

  friend void swap(BoxedCell& a, BoxedCell& b) {
    std::swap(a.value_, b.value_);
  }

  /* Returns the contained ExpressionKind. */
  [[nodiscard]] ExpressionKind get_kind() const {
    const std::uint16_t tag = tag_bits();
    // When the floating-point exponent is all 1's, then the floating-point
    // value is non-finite: either ±infinity, or NaN (which in our case is a
    // tagged pointer). When the exponent is something _other than_ all 1's,
    // then the floating-point value is finite, i.e., a Constant.
    constexpr std::uint16_t non_finite_mask = 0x7FF0;
    if ((tag & non_finite_mask) != non_finite_mask) {
      return ExpressionKind::Constant;
    }
    // To distinguish ±infinity, it's sufficient to check the lower four bits.
    // The ExpressionKind tagging constants were carefully assigned to never
    // use a zero there.
    constexpr std::uint16_t infinity_mask = 0x000F;
    if ((tag & infinity_mask) == 0) {
      return ExpressionKind::Constant;
    }
    // We're a NaN (i.e., a tagged pointer). Our tag is the ExpressionKind.
    return static_cast<ExpressionKind>(tag);
  }

  /* Returns true iff get_kind() == ExpressionKind::Constant. */
  [[nodiscard]] bool is_constant() const { return !std::isnan(value_); }

  /* Returns true iff get_kind() == kind.
  @pre kind is not ExpressionKind::Constant (use is_constant() instead) */
  template <ExpressionKind kind>
  [[nodiscard]] bool is_kind() const {
    static_assert(kind != ExpressionKind::Constant);
    return tag_bits() == static_cast<std::uint16_t>(kind);
  }

  /* Returns the Constant value.
  @pre This expression is a Constant. */
  [[nodiscard]] double constant() const {
    DRAKE_ASSERT(is_constant());
    return value_;
  }

  /* If this is a Constant, then returns the value; otherwise, returns NaN. */
  [[nodiscard]] double constant_or_nan() const { return value_; }

  /* Returns true iff this and other are "trivially" equal, i.e., they are
  either the both same Constant value, or else both point to the same cell. */
  [[nodiscard]] bool trivially_equals(const BoxedCell& other) const {
    return value_bits() == other.value_bits();
  }

  /* Sets this to a new Constant value.
  @pre This expression is already a Constant.
  @pre The new_value is not NaN. */
  void update_constant(double new_value) {
    DRAKE_ASSERT(is_constant());
    DRAKE_ASSERT(!std::isnan(new_value));
    value_ = new_value;
  }

  /* Returns a const reference to the owned cell.
  @pre This expression is not a Constant. */
  [[nodiscard]] const ExpressionCell& cell() const {
    DRAKE_ASSERT(!is_constant());
    // Convert the bit pattern back to the `owned` pointer.
    // https://en.cppreference.com/w/cpp/language/reinterpret_cast (case 3)
    return *reinterpret_cast<ExpressionCell*>(pointer_bits());
  }

  /* Sets this to point at the given cell_to_share, incrementing its use_count.
  @pre This is a Constant (i.e., does currently not own any cell).
  @pre cell_to_share is not null. */
  void SetSharedCell(const ExpressionCell* cell_to_share) noexcept;

 private:
  explicit BoxedCell(std::unique_ptr<ExpressionCell> cell);

  /* Implements copy construction when other is not a constant. */
  void ConstructCopy(const BoxedCell& other);

  /* Implements copy-assignment when either this or other are not constants. */
  void AssignCopy(const BoxedCell& other);

  /* Assigns the Constant value 0.0 to this cell. If our current value was not
  already a Constant, then decrements the use_count of our currently-owned cell
  beforehand and also deletes the cell in case the use_count reached zero. */
  void Release() noexcept;

  /* Returns our value_ as an unsigned integer. If this is not a Constant, then
  the upper 16 bits are the kind and the lower 48 bits are the pointer to the
  shared ExpressionCell. Refer to the BoxedCell class overview comment for
  additional details. */
  std::uintptr_t value_bits() const {
    std::uintptr_t result;
    std::memcpy(&result, &value_, sizeof(result));
    return result;
  }

  /* Returns the tag portion of our value_. If this is not a Constant, then the
  result will be our ExpressionKind. Refer to the BoxedCell class overview
  comment for additional details. */
  std::uint16_t tag_bits() const { return value_bits() >> 48; }

  /* Returns the pointer portion of our value_. If this is not a Constant, then
  the result will be the pointer to the shared ExpressionCell.  Refer to the
  BoxedCell class overview for additional details. */
  std::uintptr_t pointer_bits() const {
    return value_bits() & ~(0xFFFFull << 48);
  }

#ifdef DRAKE_ASSERT_IS_ARMED
  double value_{};
#else
  // N.B. We are careful that every constructor always sets this.
  // Leaving it non-redundantly initialized here is slightly faster.
  double value_;
#endif
};

}  // namespace internal
}  // namespace symbolic
}  // namespace drake
