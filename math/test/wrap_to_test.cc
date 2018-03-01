#include "drake/math/wrap_to.h"

#include <gtest/gtest.h>

namespace drake {
namespace math {
namespace {

GTEST_TEST(WrapToTest, BasicTest) {
  EXPECT_NEAR(wrap_to(.1, 0., 1.), .1, 1e-12);
  EXPECT_NEAR(wrap_to(1., 0., 1.), 0., 1e-12);
  EXPECT_NEAR(wrap_to(-.1, 0., 1.), .9, 1e-12);
  EXPECT_NEAR(wrap_to(2.1, 0., 1.), .1, 1e-12);
  EXPECT_NEAR(wrap_to(-1.1, 0., 1.), .9, 1e-12);
  EXPECT_NEAR(wrap_to(6., 4., 8.), 6., 1e-12);
  EXPECT_NEAR(wrap_to(2., 4., 8.), 6., 1e-12);
  EXPECT_NEAR(wrap_to(15.5, 0., 1.), 0.5, 1e-12);
}

}  // namespace
}  // namespace math
}  // namespace drake
