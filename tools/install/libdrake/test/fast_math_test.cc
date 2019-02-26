/// @file
/// Ensures that `-ffast--math` is not enabled in any dependencies upstream of
/// Drake.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace drake {
namespace {

GTEST_TEST(FastMathTest, SubnormalNumbers) {
  // See discussion in #10253 for more details.
  const double zero = 0.;
  const double slightly_less = std::nexttoward(
      0., -std::numeric_limits<double>::infinity());
  ASSERT_TRUE(slightly_less < zero);
}

}  // namespace
}  // namespace drake
