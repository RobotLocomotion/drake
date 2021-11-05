#include "drake/tools/vector_gen/test/gen/sample.h"

#include <cmath>
#include <stdexcept>
#include <type_traits>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace tools {
namespace test {
namespace {
template <typename T>
void CheckSampleBounds(const systems::VectorBase<T>& sample) {
  const double kInf = std::numeric_limits<double>::infinity();
  Eigen::VectorXd lower, upper;
  sample.GetElementBounds(&lower, &upper);
  EXPECT_TRUE(CompareMatrices(lower, Eigen::Vector4d(0, -kInf, -1, -kInf)));
  EXPECT_TRUE(CompareMatrices(upper, Eigen::Vector4d(kInf, 2, 1, kInf)));
}

// We don't really expect the Sample<T> implementation to fail, but we'd like
// to touch every method as an API sanity check, so that we are reminded in
// case we change the generated API without realizing it.
GTEST_TEST(SampleTest, SimpleCoverage) {
  EXPECT_EQ(SampleIndices::kNumCoordinates, 4);
  EXPECT_EQ(SampleIndices::kX, 0);
  EXPECT_EQ(SampleIndices::kTwoWord, 1);
  EXPECT_EQ(SampleIndices::kAbsone, 2);
  EXPECT_EQ(SampleIndices::kUnset, 3);

  // The device under test.
  Sample<double> dut;

  // Size.
  EXPECT_EQ(dut.size(), SampleIndices::kNumCoordinates);

  // Bounds.
  CheckSampleBounds(dut);

  // Default values.
  EXPECT_EQ(dut.x(), 42.0);
  EXPECT_EQ(dut.two_word(), 0.0);

  // Accessors.
  dut.set_x(11.0);
  dut.set_two_word(22.0);
  EXPECT_EQ(dut.x(), 11.0);
  EXPECT_EQ(dut.two_word(), 22.0);

  // Chained construction from a prvalue.
  const auto& chained = Sample<double>{}.with_x(33.0).with_two_word(44.0);
  EXPECT_EQ(chained.x(), 33.0);
  EXPECT_EQ(chained.two_word(), 44.0);
  CheckSampleBounds(chained);

  // Chained copying from an lvalue.
  const auto& tweaked_dut = dut.with_x(55.0);
  EXPECT_EQ(dut.x(), 11.0);
  EXPECT_EQ(tweaked_dut.x(), 55.0);
  CheckSampleBounds(tweaked_dut);

  // Clone.
  auto cloned = dut.Clone();
  ASSERT_NE(cloned.get(), nullptr);
  EXPECT_NE(dynamic_cast<Sample<double>*>(cloned.get()), nullptr);
  EXPECT_EQ(cloned->GetAtIndex(0), 11.0);
  EXPECT_EQ(cloned->GetAtIndex(1), 22.0);
  CheckSampleBounds(*cloned);

  // Coordinate names.
  const std::vector<std::string>& coordinate_names = dut.GetCoordinateNames();
  const std::vector<std::string> expected_names = {
    "x", "two_word", "absone", "unset",
  };
  ASSERT_EQ(coordinate_names.size(), expected_names.size());
  for (int i = 0; i < dut.size(); ++i) {
    EXPECT_EQ(coordinate_names.at(i), expected_names.at(i));
  }
}

// When inheritance is in use, we should only permit public copy & move
// operations to exist on classes that are marked as final.
static_assert(
    std::is_final_v<Sample<double>>,
    "Sample<T> should have been final");

// Confirm that copy semantics work.
GTEST_TEST(SampleTest, Copy) {
  Sample<double> first;
  first.set_x(1.0);
  const int nominal_size = SampleIndices::kNumCoordinates;

  // Copy construction.
  Sample<double> second(first);
  EXPECT_EQ(first.size(), nominal_size);
  EXPECT_EQ(second.size(), nominal_size);
  EXPECT_EQ(first.x(), 1.0);
  EXPECT_EQ(second.x(), 1.0);
  CheckSampleBounds(first);
  CheckSampleBounds(second);

  // Copy assignment.
  Sample<double> third;
  DRAKE_DEMAND(third.x() != 1.0);  // Prove that assignment will change this.
  third = second;
  EXPECT_EQ(second.size(), nominal_size);
  EXPECT_EQ(third.size(), nominal_size);
  EXPECT_EQ(second.x(), 1.0);
  EXPECT_EQ(third.x(), 1.0);
  CheckSampleBounds(second);
  CheckSampleBounds(third);
}

// Confirm that move semantics are efficient (no copying).
GTEST_TEST(SampleTest, Move) {
  Sample<double> first;
  first.set_x(1.0);
  const int nominal_size = SampleIndices::kNumCoordinates;
  const double* const original_storage = &first.x();

  // Note that when we move-construct or move-assign from a prvalue, the size of
  // the donor object goes to zero, even though it is still a Sample object.
  // That means that methods such as get_x etc. will throw if we call them.

  // Move construction.  The heap storage of `first` is stolen.
  Sample<double> second(std::move(first));
  EXPECT_EQ(first.size(), 0);
  DRAKE_EXPECT_THROWS_MESSAGE(first.x(), std::out_of_range, ".*moved-from.*");
  EXPECT_EQ(second.size(), nominal_size);
  EXPECT_EQ(second.x(), 1.0);
  EXPECT_EQ(&second.x(), original_storage);
  CheckSampleBounds(second);

  // Move assignment.  The heap storage of `second` is stolen.
  Sample<double> third;
  DRAKE_DEMAND(third.x() != 1.0);  // Prove that assignment will change this.
  third = std::move(second);
  EXPECT_EQ(second.size(), 0);
  DRAKE_EXPECT_THROWS_MESSAGE(second.x(), std::out_of_range, ".*moved-from.*");
  EXPECT_EQ(third.size(), nominal_size);
  EXPECT_EQ(third.x(), 1.0);
  EXPECT_EQ(&third.x(), original_storage);
  CheckSampleBounds(third);
}

// Cover Simple<double>::IsValid.
GTEST_TEST(SampleTest, IsValid) {
  // N.B. Sample<T>.unset is an invalid value by default.
  Sample<double> dummy1;
  EXPECT_FALSE(dummy1.IsValid());
  dummy1.set_unset(0.0);
  EXPECT_TRUE(dummy1.IsValid());
  dummy1.set_x(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(dummy1.IsValid());

  Sample<double> dummy2;
  dummy2.set_unset(0.0);
  EXPECT_TRUE(dummy2.IsValid());
  dummy2.set_two_word(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(dummy2.IsValid());
}

// Cover Simple<AutoDiffXd>::IsValid.
GTEST_TEST(SampleTest, AutoDiffXdIsValid) {
  // A NaN in the AutoDiffScalar::value() makes us invalid.
  Sample<AutoDiffXd> dut;
  dut.set_unset(0.0);  // N.B. Sample<T>.unset is an invalid value by default.
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

GTEST_TEST(SampleTest, SetToNamedVariablesTest) {
  Sample<symbolic::Expression> dut;
  dut.SetToNamedVariables();
  EXPECT_EQ(dut.x().to_string(), "x");
  EXPECT_EQ(dut.two_word().to_string(), "two_word");
  EXPECT_EQ(dut.absone().to_string(), "absone");
}

// Cover Simple<Expression>::IsValid.
GTEST_TEST(SampleTest, SymbolicIsValid) {
  Sample<symbolic::Expression> dut;
  dut.SetToNamedVariables();
  const symbolic::Formula expected_is_valid =
      !isnan(dut.x()) &&
      !isnan(dut.two_word()) &&
      !isnan(dut.absone()) &&
      !isnan(dut.unset()) &&
      (dut.x() >= 0.0) &&
      (dut.two_word() <= 2.0) &&
      (dut.absone() >= -1.0) &&
      (dut.absone() <= 1.0);
  EXPECT_TRUE(dut.IsValid().EqualTo(expected_is_valid));
}

// Cover Serialize and its relationship to YAML.
GTEST_TEST(SampleTest, YamlTest) {
  const std::string yaml_text = R"""(
x: 0
two_word: -1
absone: 0.25
unset: 99e9
)""";
  const auto dut = drake::yaml::LoadYamlString<Sample<double>>(yaml_text);
  EXPECT_EQ(dut.x(), 0.0);
  EXPECT_EQ(dut.two_word(), -1.0);
  EXPECT_EQ(dut.absone(), 0.25);
  EXPECT_EQ(dut.unset(), 99e9);
}

}  // namespace
}  // namespace test
}  // namespace tools
}  // namespace drake
