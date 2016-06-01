#include "drake/systems/framework/basic_vector.h"

#include <cmath>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/core/functional_form.h"
#include "drake/util/Polynomial.h"
#include "drake/util/eigen_matrix_compare.h"

using drake::util::CompareMatrices;
using drake::util::MatrixCompareType;

namespace drake {
namespace systems {
namespace {

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
  EXPECT_TRUE(vec.get_value()[0].isApprox(Polynomiald(0.0),
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

// Tests that an error is thrown when the BasicVector is set from a vector
// of a different size.
GTEST_TEST(BasicVectorTest, ReinitializeInvalid) {
  BasicVector<int> vec(2);
  Eigen::Vector3i next_value;
  next_value << 3, 4, 5;
  EXPECT_THROW(vec.set_value(next_value), std::out_of_range);
}

}  // namespace
}  // namespace systems
}  // namespace drake
