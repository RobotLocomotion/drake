#include <sstream>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(StandardOperationsTest, Ceil) {
  const AutoDiffDut x{0.5, 3, 0};
  const double y = ceil(x);
  EXPECT_EQ(y, std::ceil(x.value()));
}

TEST_F(StandardOperationsTest, Floor) {
  const AutoDiffDut x{0.5, 3, 0};
  const double y = floor(x);
  EXPECT_EQ(y, std::floor(x.value()));
}

TEST_F(StandardOperationsTest, Round) {
  const AutoDiffDut x{0.5, 3, 0};
  const double y = round(x);
  EXPECT_EQ(y, std::round(x.value()));
}

TEST_F(StandardOperationsTest, NextToward) {
  const AutoDiffDut x{0.5, 3, 0};
  const double y = nexttoward(x, 1.0);
  EXPECT_EQ(y, std::nexttoward(x.value(), 1.0));
}

TEST_F(StandardOperationsTest, CopySign) {
  const AutoDiffDut x{0.5, 3, 0};
  const AutoDiffDut x_pos = copysign(x, 22.2);
  const AutoDiffDut x_neg = copysign(x, -22.2);
  EXPECT_EQ(x_pos, x);
  EXPECT_EQ(x_neg, -x);
}

TEST_F(StandardOperationsTest, Classify) {
  const AutoDiffDut a{0.5, 3, 0};
  const AutoDiffDut b{std::numeric_limits<double>::infinity(), 3, 0};
  const AutoDiffDut c{std::numeric_limits<double>::quiet_NaN(), 3, 0};
  for (const auto& x : {a, b, c}) {
    EXPECT_EQ(isfinite(x), std::isfinite(x.value())) << x.value();
    EXPECT_EQ(isinf(x), std::isinf(x.value())) << x.value();
    EXPECT_EQ(isnan(x), std::isnan(x.value())) << x.value();
  }
}

}  // namespace
}  // namespace test
}  // namespace drake
