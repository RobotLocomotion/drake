#include "drake/math/saturate.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

#include "gtest/gtest.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_expression_cell.h"
#include "drake/common/symbolic_variable.h"

namespace drake {

using symbolic::Environment;
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
  const AutoDiffXd withinBoundsValue{7.5, Vector1d(6.38)};

  EXPECT_EQ(
      saturate(AutoDiffXd{1.0, Vector1d(10.2)}, lowerbound, upperbound),
      lowerbound);
  EXPECT_EQ(
      saturate(withinBoundsValue, lowerbound, upperbound),
      withinBoundsValue);
  EXPECT_EQ(
      saturate(AutoDiffXd{100.3, Vector1d(-9)}, lowerbound, upperbound),
      upperbound);

  // Tests a mixed-type scenario. Adding or subtracting 1 from an AutoDiffXd
  // does not result in an AutoDiffXd, but rather some intermediate type that
  // implicitly converts to AutoDiffXd.
  EXPECT_EQ(
      saturate(AutoDiffXd{1.0, Vector1d(10.2)}, lowerbound - 1, upperbound + 1),
      lowerbound - 1);
}

// Tests that saturate() works with symbolic::Expression types.
GTEST_TEST(SaturateTest, SymbolicExpressionTest) {
  Expression result;
  result = saturate(Expression{1.5}, Expression::One(), Expression::Pi());
  EXPECT_EQ(result.to_string(), "1.5");
  result = saturate(Expression::Zero(), Expression::One(), Expression::Pi());
  EXPECT_EQ(result.to_string(), "1");
  result = saturate(Expression{5.6}, Expression::One(), Expression::Pi());
  const std::string kPi{"3.14"};
  EXPECT_EQ(result.to_string().compare(0, kPi.length(), kPi), 0);

  symbolic::Variable x{"x"};
  symbolic::Variable lo{"lo"};
  symbolic::Variable hi{"hi"};
  auto saturate_expression = saturate(
      Expression{x}, Expression{lo}, Expression{hi});
  EXPECT_EQ(saturate_expression.Evaluate(
      Environment{{x, 0}, {lo, 3}, {hi, 10}}), 3);
  EXPECT_EQ(saturate_expression.Evaluate(
      Environment{{x, 5}, {lo, 3}, {hi, 10}}), 5);
  EXPECT_EQ(saturate_expression.Evaluate(
      Environment{{x, 12.334}, {lo, 3}, {hi, 10}}), 10);
}

}  // namespace
}  // namespace math
}  // namespace drake
