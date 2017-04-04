#include "gtest/gtest.h"
#include "ignition/math/Angle.hh"

namespace drake {
namespace maliput {
namespace monolane {

GTEST_TEST(IgnitionMathTest, Test) {
  ignition::math::Angle angle;
  EXPECT_TRUE(ignition::math::equal(0.0, angle.Radian()));
};


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
