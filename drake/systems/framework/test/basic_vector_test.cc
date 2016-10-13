#include "drake/systems/framework/basic_vector.h"

#include <cmath>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/functional_form.h"
#include "drake/common/polynomial.h"

namespace drake {
namespace systems {
namespace {

// Tests SetZero functionality.
GTEST_TEST(BasicVectorTest, SetZero) {
  BasicVector<double> vec(3);
  vec.get_mutable_value() << 1.0, 2.0, 3.0;
  vec.SetZero();
  Eigen::Vector3d expected;
  expected << 0.0, 0.0, 0.0;
  EXPECT_TRUE(CompareMatrices(expected, vec.get_value(),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  vec.get_mutable_value() << 1.0, 2.0, 3.0;
}

// Tests that the BasicVector<double> is initialized to NaN.
GTEST_TEST(BasicVectorTest, DoubleInitiallyNaN) {
  BasicVector<double> vec(3);
  Eigen::Vector3d expected;
  expected << NAN, NAN, NAN;
  EXPECT_TRUE(CompareMatrices(expected, vec.get_value(),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Tests that the BasicVector<int> is initialized to zero.
GTEST_TEST(BasicVectorTest, IntInitiallyZero) {
  BasicVector<int> vec(3);
  Eigen::Vector3i expected;
  expected << 0, 0, 0;
  EXPECT_EQ(expected, vec.get_value());
}

// Tests that the BasicVector<Polynomiald> is initialized to zero.
GTEST_TEST(BasicVectorTest, PolynomialInitiallyZero) {
  BasicVector<Polynomiald> vec(1);
  EXPECT_TRUE(vec.get_value()[0].IsApprox(Polynomiald(0.0),
                                          Eigen::NumTraits<double>::epsilon()));
}

// Tests that the BasicVector<FunctionalForm> is initialized to undefined.
GTEST_TEST(BasicVectorTest, FunctionalFormInitiallyUndefined) {
  BasicVector<FunctionalForm> vec(1);
  EXPECT_TRUE(vec.get_value()[0].IsUndefined());
}

// Tests that the BasicVector has a size as soon as it is constructed.
GTEST_TEST(BasicVectorTest, Size) {
  BasicVector<int> vec(5);
  EXPECT_EQ(5, vec.size());
}

// Tests that the BasicVector can be mutated in-place.
GTEST_TEST(BasicVectorTest, Mutate) {
  BasicVector<int> vec(2);
  vec.get_mutable_value() << 1, 2;
  Eigen::Vector2i expected;
  expected << 1, 2;
  EXPECT_EQ(expected, vec.get_value());
}

// Tests that the BasicVector can be set from another vector.
GTEST_TEST(BasicVectorTest, SetWholeVector) {
  BasicVector<int> vec(2);
  vec.get_mutable_value() << 1, 2;
  Eigen::Vector2i next_value;
  next_value << 3, 4;
  vec.set_value(next_value);
  EXPECT_EQ(next_value, vec.get_value());
}

// Tests that when BasicVector is cloned, its data is preserved.
GTEST_TEST(BasicVectorTest, Clone) {
  BasicVector<int> vec(2);
  vec.get_mutable_value() << 1, 2;

  std::unique_ptr<BasicVector<int>> clone = vec.Clone();

  Eigen::Vector2i expected;
  expected << 1, 2;
  EXPECT_EQ(expected, clone->get_value());
}

// Tests that an error is thrown when the BasicVector is set from a vector
// of a different size.
GTEST_TEST(BasicVectorTest, ReinitializeInvalid) {
  BasicVector<int> vec(2);
  Eigen::Vector3i next_value;
  next_value << 3, 4, 5;
  EXPECT_THROW(vec.set_value(next_value), std::out_of_range);
}

// Tests all += * operations for BasicVector.
GTEST_TEST(BasicVectorTest, PlusEqScaled) {
  BasicVector<double> ogvec(2), vec1(2), vec2(2), vec3(2), vec4(2), vec5(2);
  Eigen::Vector2d ans1, ans2, ans3, ans4, ans5;
  ogvec.SetZero();
  vec1.get_mutable_value() << 1.0, 2.0;
  vec2.get_mutable_value() << 3.0, 5.0;
  vec3.get_mutable_value() << 7.0, 11.0;
  vec4.get_mutable_value() << 13.0, 17.0;
  vec5.get_mutable_value() << 19.0, 23.0;
  VectorBase<double>& v1 = vec1;
  VectorBase<double>& v2 = vec2;
  VectorBase<double>& v3 = vec3;
  VectorBase<double>& v4 = vec4;
  VectorBase<double>& v5 = vec5;
  ogvec.PlusEqScaled(2.0, v1);
  ans1 << 2.0, 4.0;
  EXPECT_TRUE(CompareMatrices(ans1, ogvec.get_value(),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  ogvec.SetZero();
  ogvec.PlusEqScaled({{2.0, v1}, {3.0, v2}});
  ans2 << 11.0, 19.0;
  EXPECT_TRUE(CompareMatrices(ans2, ogvec.get_value(),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  ogvec.SetZero();
  ogvec.PlusEqScaled({{2.0, v1}, {3.0, v2}, {5.0, v3}});
  ans3 << 46.0, 74.0;
  EXPECT_TRUE(CompareMatrices(ans3, ogvec.get_value(),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  ogvec.SetZero();
  ogvec.PlusEqScaled({{2.0, v1}, {3.0, v2}, {5.0, v3}, {7.0, v4}});
  ans4 << 137.0, 193.0;
  EXPECT_TRUE(CompareMatrices(ans4, ogvec.get_value(),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  ogvec.SetZero();
  ogvec.PlusEqScaled({{2.0, v1}, {3.0, v2}, {5.0, v3}, {7.0, v4}, {11.0, v5}});
  ans5 << 346.0, 446.0;
  EXPECT_TRUE(CompareMatrices(ans5, ogvec.get_value(),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace systems
}  // namespace drake
