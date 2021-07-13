#include "drake/common/bit_cast.h"

#include <cstdint>

#include <gtest/gtest.h>

namespace drake {
namespace internal {
namespace {

using std::uint64_t;

// Mostly, this just checks for compilation failures.
GTEST_TEST(BitCast, BasicTest) {
  const uint64_t foo = bit_cast<int64_t>(1.0);
  EXPECT_EQ(foo, 0x3ff0000000000000ull);
  const double bar = bit_cast<double>(foo);
  EXPECT_EQ(bar, 1.0);
}

}  // namespace
}  // namespace internal
}  // namespace drake
