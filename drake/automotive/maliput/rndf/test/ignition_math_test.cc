#include "gtest/gtest.h"
#include "ignition/math/Angle.hh"

namespace drake {
namespace maliput {
namespace rndf {

// Exercise the Bazel infrastructure and ensure that Ignition Math can be
// built and linked.
GTEST_TEST(IgnitionMathTest, Test) {
  ignition::math::Angle angle;
  EXPECT_TRUE(ignition::math::equal(0.0, angle.Radian()));
};


}  // namespace rndf
}  // namespace maliput
}  // namespace drake
