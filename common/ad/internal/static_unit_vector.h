#pragma once

namespace drake {
namespace ad {
namespace internal {

/* The limit (inclusive) on the size supported by GetStaticUnitVector(). */
constexpr int kMaxStaticVectorSize = 1023;

/* Returns a pointer to never-destroyed readonly memory containing a unit
vector using the given vector `offset` as the sole non-zero (1.0) element.
The implied array size is kMaxStaticVectorSize. For example, `offset == 0`
returns [1, 0, 0, 0, ...] and `offset == 2` returns `[0, 0, 1, 0, ...]`.
@pre 0 <= offset < kMaxStaticVectorSize */
const double* GetStaticUnitVector(int offset);

}  // namespace internal
}  // namespace ad
}  // namespace drake
