#include "drake/systems/framework/basic_vector.h"

#include <cmath>
#include <sstream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

namespace drake {
namespace systems {
namespace {

// Tests the initializer_list functionality.
GTEST_TEST(BasicVectorTest, InitializerList) {
  const BasicVector<double> empty1;  // Default constructor.
  const BasicVector<double> empty2{};  // Initializer list.
  EXPECT_EQ(0, empty1.size());
  EXPECT_EQ(0, empty2.size());

  const BasicVector<double> pair{1.0, 2.0};
  ASSERT_EQ(2, pair.size());
  EXPECT_EQ(1.0, pair[0]);
  EXPECT_EQ(2.0, pair[1]);

  const BasicVector<double> from_floats{3.0f, 4.0f, 5.0f};
  ASSERT_EQ(3, from_floats.size());
  EXPECT_EQ(3.0, from_floats[0]);
  EXPECT_EQ(4.0, from_floats[1]);
  EXPECT_EQ(5.0, from_floats[2]);

  const BasicVector<AutoDiffXd> autodiff{22.0};
  ASSERT_EQ(1, autodiff.size());
  EXPECT_EQ(22.0, autodiff[0].value());
}

GTEST_TEST(BasicVectorTest, Access) {
  const BasicVector<double> pair{1.0, 2.0};
  EXPECT_EQ(1.0, pair.GetAtIndex(0));
  EXPECT_EQ(2.0, pair.GetAtIndex(1));

  EXPECT_EQ(Eigen::Vector2d(1.0, 2.0), pair.value());

  // To be deprecated: get_value() wraps a VectorBlock around the value.
  EXPECT_EQ(pair.value(), pair.get_value());
}

GTEST_TEST(BasicVectorTest, OutOfRange) {
  BasicVector<double> pair{1.0, 2.0};
  EXPECT_THROW(pair.GetAtIndex(-1), std::exception);
  EXPECT_THROW(pair.GetAtIndex(10), std::exception);
  EXPECT_THROW(pair.SetAtIndex(-1, 0.0), std::exception);
  EXPECT_THROW(pair.SetAtIndex(10, 0.0), std::exception);
}

// Tests SetZero functionality.
GTEST_TEST(BasicVectorTest, SetZero) {
  BasicVector<double> vec{1.0, 2.0, 3.0};
  EXPECT_EQ(Eigen::Vector3d(1.0, 2.0, 3.0), vec.value());
  vec.SetZero();
  EXPECT_EQ(Eigen::Vector3d(0, 0, 0), vec.value());
}

// Tests that the BasicVector<double> is initialized to NaN.
GTEST_TEST(BasicVectorTest, DoubleInitiallyNaN) {
  BasicVector<double> vec(3);
  Eigen::Vector3d expected(NAN, NAN, NAN);
  EXPECT_TRUE(CompareMatrices(expected, vec.value(),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Tests that the BasicVector<AutoDiffXd> is initialized to NaN.
GTEST_TEST(BasicVectorTest, AutodiffInitiallyNaN) {
  BasicVector<AutoDiffXd> vec(3);
  EXPECT_TRUE(std::isnan(vec[0].value()));
  EXPECT_TRUE(std::isnan(vec[1].value()));
  EXPECT_TRUE(std::isnan(vec[2].value()));
}

// Tests that the BasicVector<symbolic::Expression> is initialized to NaN.
GTEST_TEST(BasicVectorTest, SymbolicInitiallyNaN) {
  BasicVector<symbolic::Expression> vec(1);
  EXPECT_TRUE(symbolic::is_nan(vec.value()[0]));
}

// Tests BasicVector<T>::Make.
GTEST_TEST(BasicVectorTest, Make) {
  auto dut1 = BasicVector<double>::Make(1.0, 2.0);
  auto dut2 = BasicVector<double>::Make({3.0, 4.0});
  EXPECT_TRUE(CompareMatrices(dut1->value(), Eigen::Vector2d(1.0, 2.0)));
  EXPECT_TRUE(CompareMatrices(dut2->value(), Eigen::Vector2d(3.0, 4.0)));
}

// Tests that BasicVector<symbolic::Expression>::Make does what it says on
// the tin.
GTEST_TEST(BasicVectorTest, MakeSymbolic) {
  auto vec = BasicVector<symbolic::Expression>::Make(
      symbolic::Variable("x"),
      2.0,
      symbolic::Variable("y") + 2.0);
  EXPECT_EQ("x", vec->GetAtIndex(0).to_string());
  EXPECT_EQ("2", vec->GetAtIndex(1).to_string());
  EXPECT_EQ("(2 + y)", vec->GetAtIndex(2).to_string());
}

// Tests that the BasicVector has a size as soon as it is constructed.
GTEST_TEST(BasicVectorTest, Size) {
  BasicVector<double> vec(5);
  EXPECT_EQ(5, vec.size());
}

// Tests that the BasicVector can be mutated in-place.
GTEST_TEST(BasicVectorTest, Mutate) {
  BasicVector<double> vec(2);
  vec.get_mutable_value() << 1, 2;
  Eigen::Vector2d expected;
  expected << 1, 2;
  EXPECT_EQ(expected, vec.value());
}

// Tests that the BasicVector can be addressed as an array.
GTEST_TEST(BasicVectorTest, ArrayOperator) {
  BasicVector<double> vec(2);
  vec[0] = 76;
  vec[1] = 42;

  Eigen::Vector2d expected;
  expected << 76, 42;
  EXPECT_EQ(expected, vec.value());

  const auto& const_vec = vec;
  EXPECT_EQ(76, const_vec[0]);
  EXPECT_EQ(42, const_vec[1]);
}

// Tests that the BasicVector can be set from another vector.
GTEST_TEST(BasicVectorTest, SetWholeVector) {
  BasicVector<double> vec(2);
  vec.get_mutable_value() << 1, 2;

  Eigen::Vector2d next_value;
  next_value << 3, 4;
  vec.set_value(next_value);
  EXPECT_EQ(next_value, vec.value());

  Eigen::Vector2d another_value;
  another_value << 5, 6;
  vec.SetFromVector(another_value);
  EXPECT_EQ(another_value, vec.value());

  EXPECT_THROW(vec.SetFromVector(Eigen::Vector3d::Zero()), std::exception);
}

// Tests that the BasicVector can be set from another vector base.
GTEST_TEST(BasicVectorTest, SetFrom) {
  BasicVector<double> vec(2);
  vec.get_mutable_value() << 1, 2;
  BasicVector<double> next_vec(2);
  next_vec.get_mutable_value() << 3, 4;
  vec.SetFrom(next_vec);
  EXPECT_EQ(next_vec.value(), vec.value());
  EXPECT_THROW(vec.SetFrom(BasicVector<double>(3)), std::exception);
}

// Tests that when BasicVector is cloned, its data is preserved.
GTEST_TEST(BasicVectorTest, Clone) {
  BasicVector<double> vec(2);
  vec.get_mutable_value() << 1, 2;

  std::unique_ptr<BasicVector<double>> clone = vec.Clone();

  Eigen::Vector2d expected;
  expected << 1, 2;
  EXPECT_EQ(expected, clone->value());
}

// Tests that an error is thrown when the BasicVector is set from a vector
// of a different size.
GTEST_TEST(BasicVectorTest, ReinitializeInvalid) {
  BasicVector<double> vec(2);
  Eigen::Vector3d next_value;
  next_value << 3, 4, 5;
  EXPECT_THROW(vec.set_value(next_value), std::exception);
}

// Tests all += * operations for BasicVector.
GTEST_TEST(BasicVectorTest, PlusEqScaled) {
  BasicVector<double> ogvec(2), vec1(2), vec2(2), vec3(2), vec4(2), vec5(2);
  Eigen::Vector2d ans1, ans2, ans3, ans4, ans5;
  ogvec.SetZero();
  vec1.get_mutable_value() << 1, 2;
  vec2.get_mutable_value() << 3, 5;
  vec3.get_mutable_value() << 7, 11;
  vec4.get_mutable_value() << 13, 17;
  vec5.get_mutable_value() << 19, 23;
  VectorBase<double>& v1 = vec1;
  VectorBase<double>& v2 = vec2;
  VectorBase<double>& v3 = vec3;
  VectorBase<double>& v4 = vec4;
  VectorBase<double>& v5 = vec5;
  ogvec.PlusEqScaled(2, v1);
  ans1 << 2, 4;
  EXPECT_EQ(ans1, ogvec.value());

  ogvec.SetZero();
  ogvec.PlusEqScaled({{2, v1}, {3, v2}});
  ans2 << 11, 19;
  EXPECT_EQ(ans2, ogvec.value());

  ogvec.SetZero();
  ogvec.PlusEqScaled({{2, v1}, {3, v2}, {5, v3}});
  ans3 << 46, 74;
  EXPECT_EQ(ans3, ogvec.value());

  ogvec.SetZero();
  ogvec.PlusEqScaled({{2, v1}, {3, v2}, {5, v3}, {7, v4}});
  ans4 << 137, 193;
  EXPECT_EQ(ans4, ogvec.value());

  ogvec.SetZero();
  ogvec.PlusEqScaled({{2, v1}, {3, v2}, {5, v3}, {7, v4}, {11, v5}});
  ans5 << 346, 446;
  EXPECT_EQ(ans5, ogvec.value());
}

template <typename T>
class TypedBasicVectorTest : public ::testing::Test {};

using DefaultScalars =
    ::testing::Types<double, AutoDiffXd, symbolic::Expression>;
TYPED_TEST_SUITE(TypedBasicVectorTest, DefaultScalars);

// Tests ability to stream a BasicVector into a string.
TYPED_TEST(TypedBasicVectorTest, StringStream) {
  using T = TypeParam;
  BasicVector<T> vec(3);
  vec.get_mutable_value() << 1.0, 2.2, 3.3;
  std::stringstream s;
  s << "hello " << vec << " world";
  std::stringstream s_expected;
  s_expected << "hello " << vec.value().transpose() << " world";
  EXPECT_EQ(s.str(), s_expected.str());
}

// Tests ability to stream a BasicVector of size zero into a string.
TYPED_TEST(TypedBasicVectorTest, ZeroLengthStringStream) {
  using T = TypeParam;
  BasicVector<T> vec(0);
  std::stringstream s;
  s << "foo [" << vec << "] bar";
  std::stringstream s_expected;
  s_expected << "foo [" << vec.value().transpose() << "] bar";
  EXPECT_EQ(s.str(), s_expected.str());
}

// Tests the default set of bounds (empty).
GTEST_TEST(BasicVectorTest, DefaultCalcInequalityConstraint) {
  VectorX<double> value = VectorX<double>::Ones(22);
  BasicVector<double> vec(1);
  Eigen::VectorXd lower, upper;
  // Deliberately set lower/upper to size 2, to check if GetElementBounds will
  // resize the bounds to empty size.
  lower.resize(2);
  upper.resize(2);
  vec.GetElementBounds(&lower, &upper);
  EXPECT_EQ(lower.size(), 0);
  EXPECT_EQ(upper.size(), 0);
}

// Tests the protected `::values()` methods.
GTEST_TEST(BasicVectorTest, ValuesAccess) {
  MyVector2d dut;
  dut[0] = 11.0;
  dut[1] = 22.0;

  // Values are as expected.
  ASSERT_EQ(dut.values().size(), 2);
  EXPECT_EQ(dut.values()[0], 11.0);
  EXPECT_EQ(dut.values()[1], 22.0);
  dut.values()[0] = 33.0;

  // The const overload is the same.
  const auto& const_dut = dut;
  EXPECT_EQ(&dut.values(), &const_dut.values());
  EXPECT_EQ(const_dut.values()[0], 33.0);
}

// Tests for reasonable exception message text; because the formatting is
// reused from VectorBase, this effectively tests the formatting for all
// subclasses of VectorBase.
GTEST_TEST(BasicVectorTest, ExceptionMessages) {
  BasicVector<double> pair{1.0, 2.0};
  DRAKE_EXPECT_THROWS_MESSAGE(
      pair.GetAtIndex(-1),
      "Index -1 is not within \\[0, 2\\) while accessing .*BasicVector.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      pair.GetAtIndex(10),
      "Index 10 is not within \\[0, 2\\) while accessing .*BasicVector.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      pair.SetFromVector(Eigen::Vector3d::Zero()),
      "Operand vector size 3 does not match this .*BasicVector.* size 2");
}

}  // namespace
}  // namespace systems
}  // namespace drake
