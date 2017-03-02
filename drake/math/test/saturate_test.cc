#include "drake/math/saturate.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

#include "gtest/gtest.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_expression_cell.h"

namespace drake {

using symbolic::Expression;

namespace math {
namespace {

// Tests that saturate works with double types.
GTEST_TEST(SaturateTest, DoubleTest) {
  const double kLow{5};
  const double kHigh{10};

  const double kTooLow{kLow - 1e-10};
  const double kMiddle{kLow + kHigh / 2.0};
  const double kTooHigh{kHigh + 1e-10};
  EXPECT_EQ(saturate(kTooLow, kLow, kHigh), kLow);
  EXPECT_EQ(saturate(kMiddle, kLow, kHigh), kMiddle);
  EXPECT_EQ(saturate(kTooHigh, kLow, kHigh), kHigh);
}

// Tests that saturate works with AutoDiff types.
GTEST_TEST(SaturateTest, AutoDiffXdTest) {
  const AutoDiffXd lowerbound{5.0, Vector1d(1.5)};
  const AutoDiffXd upperbound{10.0, Vector1d(-1.5)};
  const AutoDiffXd middleValue{7.5, Vector1d(0)};

  EXPECT_DOUBLE_EQ(
      saturate(AutoDiffXd{1.0, Vector1d(10.2)}, lowerbound, upperbound).value(),
      lowerbound.value());
  EXPECT_DOUBLE_EQ(
      saturate(AutoDiffXd{7.5, Vector1d(6.38)}, lowerbound, upperbound).value(),
      7.5);
  EXPECT_DOUBLE_EQ(
      saturate(AutoDiffXd{100.3, Vector1d(-9)}, lowerbound, upperbound).value(),
      upperbound.value());
}

// Tests that saturate() works with symbolic::Expression types.
GTEST_TEST(SaturateTest, SymbolicExpressionTest) {
  auto no_saturation = saturate(
      Expression{1.5}, Expression::One(), Expression::Pi());
  EXPECT_EQ(no_saturation.to_string(), "1.5");
  auto saturate_low = saturate(
      Expression::Zero(), Expression::One(), Expression::Pi());
  EXPECT_EQ(saturate_low.to_string(), "1");
  auto saturate_high = saturate(
      Expression{5.6}, Expression::One(), Expression::Pi());
  EXPECT_EQ(saturate_high.to_string(), "3.1415926535897931");
}

}  // namespace
}  // namespace math
}  // namespace drake
