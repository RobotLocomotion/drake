#include <limits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/eigen_types.h"

// This file contains only the very most basic unit tests for AutoDiffXd, e.g.,
// construction and assignment. Tests for the derivative math are elsewhere.

namespace drake {
namespace ad {
namespace {

using Eigen::VectorXd;

GTEST_TEST(AutodiffBasicTest, DefaultCtor) {
  const AutoDiff dut;
  EXPECT_EQ(dut.value(), 0.0);
  EXPECT_EQ(dut.derivatives().size(), 0);
}

GTEST_TEST(AutodiffBasicTest, ConvertingCtor) {
  const AutoDiff dut{1.0};
  EXPECT_EQ(dut.value(), 1.0);
  EXPECT_EQ(dut.derivatives().size(), 0);
}

GTEST_TEST(AutodiffBasicTest, UnitCtor) {
  const AutoDiff dut{1.0, 4, 2};
  EXPECT_EQ(dut.value(), 1.0);
  EXPECT_EQ(dut.derivatives().size(), 4);
  EXPECT_EQ(dut.derivatives()[2], 1.0);
}

GTEST_TEST(AutodiffBasicTest, VectorCtor) {
  const Eigen::VectorXd derivs = Eigen::VectorXd::LinSpaced(3, 1.0, 3.0);
  const AutoDiff dut{1.0, derivs};
  EXPECT_EQ(dut.value(), 1.0);
  EXPECT_EQ(dut.derivatives().size(), 3);
  EXPECT_EQ(dut.derivatives(), derivs);
}

GTEST_TEST(AutodiffBasicTest, AssignConstant) {
  AutoDiff dut{1.0, 4, 2};
  EXPECT_EQ(dut.value(), 1.0);
  EXPECT_EQ(dut.derivatives().size(), 4);
  dut = -1.0;
  EXPECT_EQ(dut.value(), -1.0);
  EXPECT_EQ(dut.derivatives().size(), 4);
  EXPECT_TRUE(dut.derivatives().isZero(0.0));
}

constexpr double kInf = std::numeric_limits<double>::infinity();

GTEST_TEST(AutodiffBasicTest, StdTraits) {
  // Our AutoDiff implementation of std::numeric_limits is a one-liner, so it's
  // sufficient to just spot-check one value. Constants don't have gradients, so
  // check that the return type is double.
  EXPECT_THAT(std::numeric_limits<AutoDiff>::infinity(),
              testing::TypedEq<double>(kInf));
}

GTEST_TEST(AutodiffBasicTest, EigenTraits) {
  using NumTraits = Eigen::NumTraits<AutoDiff>;
  EXPECT_TRUE(NumTraits::RequireInitialization);
  EXPECT_TRUE((std::is_same_v<NumTraits::Real, AutoDiff>));
  EXPECT_TRUE((std::is_same_v<NumTraits::NonInteger, AutoDiff>));
  EXPECT_TRUE((std::is_same_v<NumTraits::Nested, AutoDiff>));
  EXPECT_TRUE((std::is_same_v<NumTraits::Literal, double>));
  // Constants don't have gradients, so check that the return type is double.
  EXPECT_THAT(NumTraits::infinity(), testing::TypedEq<double>(kInf));
}

}  // namespace
}  // namespace ad
}  // namespace drake
