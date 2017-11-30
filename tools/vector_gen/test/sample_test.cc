#include "drake/tools/vector_gen/test/gen/sample.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace tools {
namespace test {
namespace {

// We don't really expect the Sample<T> implementation to fail, but we'd like
// to touch every method as an API sanity check, so that we are reminded in
// case we change the generated API without realizing it.
GTEST_TEST(SampleTest, SimpleCoverage) {
  EXPECT_EQ(SampleIndices::kNumCoordinates, 3);
  EXPECT_EQ(SampleIndices::kX, 0);
  EXPECT_EQ(SampleIndices::kTwoWord, 1);
  EXPECT_EQ(SampleIndices::kAbsone, 2);

  // The device under test.
  Sample<double> dut;

  // Size.
  EXPECT_EQ(dut.size(), SampleIndices::kNumCoordinates);

  // Default values.
  EXPECT_EQ(dut.x(), 42.0);
  EXPECT_EQ(dut.two_word(), 0.0);

  // Accessors.
  dut.set_x(11.0);
  dut.set_two_word(22.0);
  EXPECT_EQ(dut.x(), 11.0);
  EXPECT_EQ(dut.two_word(), 22.0);

  // Clone.
  auto cloned = dut.Clone();
  ASSERT_NE(cloned.get(), nullptr);
  EXPECT_NE(dynamic_cast<Sample<double>*>(cloned.get()), nullptr);
  EXPECT_EQ(cloned->GetAtIndex(0), 11.0);
  EXPECT_EQ(cloned->GetAtIndex(1), 22.0);

  // Coordinate names.
  const std::vector<std::string>& coordinate_names = dut.GetCoordinateNames();
  const std::vector<std::string> expected_names = {"x", "two_word", "absone"};
  ASSERT_EQ(coordinate_names.size(), expected_names.size());
  for (int i = 0; i < dut.size(); ++i) {
    EXPECT_EQ(coordinate_names.at(i), expected_names.at(i));
  }
}

// Cover Simple<double>::IsValid.
GTEST_TEST(SampleTest, IsValid) {
  Sample<double> dummy1;
  EXPECT_TRUE(dummy1.IsValid());
  dummy1.set_x(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(dummy1.IsValid());

  Sample<double> dummy2;
  EXPECT_TRUE(dummy2.IsValid());
  dummy2.set_two_word(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(dummy2.IsValid());
}

// Cover Simple<AutoDiffXd>::IsValid.
GTEST_TEST(SampleTest, AutoDiffXdIsValid) {
  // A NaN in the AutoDiffScalar::value() makes us invalid.
  Sample<AutoDiffXd> dut;
  EXPECT_TRUE(dut.IsValid());
  dut.set_x(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(dut.IsValid());

  // A NaN in the AutoDiffScalar::derivatives() is still valid.
  AutoDiffXd zero_with_nan_derivatives{0};
  zero_with_nan_derivatives.derivatives() =
      Vector1d(std::numeric_limits<double>::quiet_NaN());
  ASSERT_EQ(zero_with_nan_derivatives.derivatives().size(), 1);
  EXPECT_TRUE(std::isnan(zero_with_nan_derivatives.derivatives()(0)));
  dut.set_x(zero_with_nan_derivatives);
  EXPECT_TRUE(dut.IsValid());
}

// Cover Simple<Expression>::IsValid.
GTEST_TEST(SampleTest, SymbolicIsValid) {
  Sample<symbolic::Expression> dut;
  const symbolic::Variable x{"x"};
  const symbolic::Variable two_word{"two_word"};
  const symbolic::Variable absone{"absone"};
  dut.set_x(x);
  dut.set_two_word(two_word);
  dut.set_absone(absone);
  const symbolic::Formula expected_is_valid =
      !isnan(x) &&
      !isnan(two_word) &&
      !isnan(absone) &&
      (x >= 0.0) &&
      (absone >= -1.0) &&
      (absone <= 1.0);
  EXPECT_TRUE(dut.IsValid().EqualTo(expected_is_valid));
}

}  // namespace
}  // namespace test
}  // namespace tools
}  // namespace drake
