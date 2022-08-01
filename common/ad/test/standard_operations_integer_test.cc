#include <sstream>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(StandardOperationsTest, Ceil) {
  const AutoDiffDut x{0.5, 3, 0};
  const AutoDiffDut y = ceil(x);
  EXPECT_EQ(y.value(), std::ceil(x.value()));
  EXPECT_EQ(y.derivatives(), Eigen::Vector3d::Zero());
}

TEST_F(StandardOperationsTest, Floor) {
  const AutoDiffDut x{0.5, 3, 0};
  const AutoDiffDut y = floor(x);
  EXPECT_EQ(y.value(), std::floor(x.value()));
  EXPECT_EQ(y.derivatives(), Eigen::Vector3d::Zero());
}

TEST_F(StandardOperationsTest, Round) {
  const AutoDiffDut x{0.5, 3, 0};
  const AutoDiffDut y = round(x);
  EXPECT_EQ(y.value(), std::round(x.value()));
  EXPECT_EQ(y.derivatives(), Eigen::Vector3d::Zero());
}

TEST_F(StandardOperationsTest, NextToward) {
  const AutoDiffDut x{0.5, 3, 0};
  const AutoDiffDut y = nexttoward(x, 1.0);
  EXPECT_EQ(y.value(), std::nexttoward(x.value(), 1.0));
  EXPECT_EQ(y.derivatives(), Eigen::Vector3d::Zero());
}

TEST_F(StandardOperationsTest, Classify) {
  const AutoDiffDut a{0.5, 3, 0};
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
