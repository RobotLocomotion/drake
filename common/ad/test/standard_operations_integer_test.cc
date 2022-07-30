#include <sstream>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(StandardOperationsTest, Ceil) {
  const AutoDiffDut x{0.5, 3, 0};
  EXPECT_EQ(ceil(x), 1.0);
}

TEST_F(StandardOperationsTest, Floor) {
  const AutoDiffDut x{0.5, 3, 0};
  EXPECT_EQ(floor(x), 0.0);
}

TEST_F(StandardOperationsTest, Round) {
  const AutoDiffDut x{0.5, 3, 0};
  EXPECT_EQ(round(x), 1.0);
}

TEST_F(StandardOperationsTest, NextToward) {
  const AutoDiffDut x{0.5, 3, 0};
  EXPECT_EQ(nexttoward(x, 1.0), 0.5000000000000001);
}

TEST_F(StandardOperationsTest, Classify) {
  const AutoDiffDut a{0.25, 3, 0};
  const AutoDiffDut b{std::numeric_limits<double>::infinity(), 3, 0};
  const AutoDiffDut c{std::numeric_limits<double>::quiet_NaN(), 3, 0};
  for (const auto& x : {a, b, c}) {
    EXPECT_EQ(isfinite(x), std::isfinite(x.value())) << x;
    EXPECT_EQ(isinf(x), std::isinf(x.value())) << x;
    EXPECT_EQ(isnan(x), std::isnan(x.value())) << x;
  }
}

}  // namespace
}  // namespace test
}  // namespace drake
