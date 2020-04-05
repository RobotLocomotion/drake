#include "drake/common/extract_double.h"

#include <stdexcept>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

// A non-numeric ScalarType for testing.
namespace { struct NonNumericScalar { }; }

namespace drake {
namespace {

GTEST_TEST(ExtractDoubleTest, BasicTest) {
  // A double works.
  double x = 1.0;
  EXPECT_EQ(ExtractDoubleOrThrow(x), 1.0);

  // A const double works.
  const double y = 2.0;
  EXPECT_EQ(ExtractDoubleOrThrow(y), 2.0);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // A non-numeric throws.
  NonNumericScalar non_numeric;
  EXPECT_THROW(ExtractDoubleOrThrow(non_numeric), std::exception);
#pragma GCC diagnostic pop
}

struct ConditionalScalar {
  ConditionalScalar() = default;
  explicit ConditionalScalar(double x) : value(x) {}
  double value{};
};

double ExtractDoubleOrThrow(const ConditionalScalar& scalar) {
  if (scalar.value < 0.0) {
    throw std::runtime_error(
        "Negative ConditionalScalar cannot be converted to a double");
  }
  return scalar.value;
}

GTEST_TEST(ExtractDoubleTest, MatrixTest) {
  using drake::ExtractDoubleOrThrow;

  // Fixed size matrices work.
  Eigen::Matrix2d x;
  x << 1.0, 2.0, 3.0, 4.0;
  EXPECT_TRUE(CompareMatrices(ExtractDoubleOrThrow(x), x));

  // Dynamically-sized matrices work.
  Eigen::MatrixXd y(1, 5);
  y << 1, 2, 3, 4, 5;
  EXPECT_TRUE(CompareMatrices(ExtractDoubleOrThrow(y), y));

  // A type with conditional extraction can work.
  Vector2<ConditionalScalar> c(1.0, 2.0);
  EXPECT_TRUE(CompareMatrices(ExtractDoubleOrThrow(c),
                              Eigen::Vector2d(1.0, 2.0)));

  // A type with conditional extraction can throw based on one bad elemnent.
  c(1) = ConditionalScalar(-1.0);
  EXPECT_THROW(ExtractDoubleOrThrow(c), std::exception);
}

}  // namespace
}  // namespace drake
