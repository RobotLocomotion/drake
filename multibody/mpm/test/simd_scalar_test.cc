#include "drake/multibody/mpm/simd_scalar.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

using Eigen::Vector3d;

GTEST_TEST(SimdScalar, Constructor) {
  {
    SimdScalar<double> x_double;
    SimdScalar<float> x_float;

    EXPECT_EQ(x_double.lanes(), 4);
    EXPECT_EQ(x_float.lanes(), 8);
  }
  {
    SimdScalar<double> x_double(1.2);
    SimdScalar<float> x_float(3.4);
    const std::vector<double> expected_double = {1.2, 1.2, 1.2, 1.2};
    const std::vector<float> expected_float = {3.4, 3.4, 3.4, 3.4,
                                               3.4, 3.4, 3.4, 3.4};

    std::vector<double> x_double_array(4);
    std::vector<float> x_float_array(8);
    x_double.Write(&x_double_array.front());
    x_float.Write(&x_float_array.front());
    EXPECT_EQ(x_double_array, expected_double);
    EXPECT_EQ(x_float_array, expected_float);
  }

  {
    const std::vector<double> expected_double = {1.0, 2.0, 3.0, 4.0};
    const std::vector<float> expected_float = {1.0, 2.0, 3.0, 4.0,
                                               5.0, 6.0, 7.0, 8.0};
    SimdScalar<double> x_double(&expected_double.front());
    SimdScalar<float> x_float(&expected_float.front());
    std::vector<double> x_double_array(4);
    std::vector<float> x_float_array(8);
    x_double.Write(&x_double_array.front());
    x_float.Write(&x_float_array.front());
    EXPECT_EQ(x_double_array, expected_double);
    EXPECT_EQ(x_float_array, expected_float);
  }
}

GTEST_TEST(SimdScalar, Add) {
  SimdScalar<double> x(1.2);
  SimdScalar<double> y(3.4);
  SimdScalar<double> sum(4.6);
  EXPECT_EQ(x + y, sum);
  x += y;
  EXPECT_EQ(x, sum);
}

GTEST_TEST(SimdScalar, Sub) {
  SimdScalar<double> x(1.2);
  SimdScalar<double> y(3.4);
  SimdScalar<double> diff(-2.2);
  EXPECT_EQ(x - y, diff);
  x -= y;
  EXPECT_EQ(x, diff);
}

GTEST_TEST(SimdScalar, Mul) {
  SimdScalar<double> x(1.2);
  SimdScalar<double> y(3.4);
  SimdScalar<double> prod(1.2 * 3.4);
  EXPECT_EQ(x * y, prod);
  x *= y;
  EXPECT_EQ(x, prod);
}

GTEST_TEST(SimdScalar, Div) {
  SimdScalar<double> x(1.2);
  SimdScalar<double> y(3.4);
  SimdScalar<double> quot(1.2 / 3.4);
  EXPECT_EQ(x / y, quot);
  x /= y;
  EXPECT_EQ(x, quot);
}

GTEST_TEST(SimdScalar, Eigen) {
  Matrix3<SimdScalar<double>> m;
  // clang-format off
  m << SimdScalar<double>(1.0), SimdScalar<double>(2.0), SimdScalar<double>(3.0),
       SimdScalar<double>(4.0), SimdScalar<double>(5.0), SimdScalar<double>(6.0),
       SimdScalar<double>(7.0), SimdScalar<double>(8.0), SimdScalar<double>(9.0);
  // clang-format on
  m *= 2.0;

  Matrix3<SimdScalar<double>> expected;
  // clang-format off
  expected << SimdScalar<double>(2.0), SimdScalar<double>(4.0), SimdScalar<double>(6.0),
              SimdScalar<double>(8.0), SimdScalar<double>(10.0), SimdScalar<double>(12.0),
              SimdScalar<double>(14.0), SimdScalar<double>(16.0), SimdScalar<double>(18.0);
  // clang-format on

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_EQ(m(i, j), expected(i, j));
    }
  }
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
