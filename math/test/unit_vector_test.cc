#include "drake/math/unit_vector.h"

#include <gtest/gtest.h>
#include <unsupported/Eigen/AutoDiff>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace math {
namespace internal {
namespace {

GTEST_TEST(UnitVectorTest, ThrowOrWarnIfNotUnitVector) {
  double vector_mag_squared;

  // Verify that no exception is thrown for a valid unit vector.
  Vector3<double> unit_vector(1.0, 0.0, 0.0);
  DRAKE_EXPECT_NO_THROW(vector_mag_squared = ThrowIfNotUnitVector(
                            unit_vector, "UnusedFunctionName"));
  EXPECT_EQ(vector_mag_squared, unit_vector.squaredNorm());

  // Verify WarnIfNotUnitVector() returns exactly 1.0 for a perfect unit vector.
  vector_mag_squared = WarnIfNotUnitVector(unit_vector, "UnusedFunctionName");
  EXPECT_EQ(vector_mag_squared, 1.0);

  // Verify that no exception is thrown for a valid or near valid unit vector.
  unit_vector = Vector3<double>(4.321, M_PI, 97531.2468).normalized();
  DRAKE_EXPECT_NO_THROW(vector_mag_squared = ThrowIfNotUnitVector(
                            unit_vector, "UnusedFunctionName"));
  EXPECT_EQ(vector_mag_squared, unit_vector.squaredNorm());

  // Verify that no exception is thrown when |unit_vector| is nearly 1.0.
  constexpr double kepsilon = std::numeric_limits<double>::epsilon();
  unit_vector = Vector3<double>(1 + kepsilon, 0, 0);
  DRAKE_EXPECT_NO_THROW(vector_mag_squared = ThrowIfNotUnitVector(
                            unit_vector, "UnusedFunctionName"));
  EXPECT_EQ(vector_mag_squared, unit_vector.squaredNorm());
  EXPECT_NE(vector_mag_squared, 1.0);

  // Verify that no exception is thrown when unit_vector is symbolic.
  const Vector3<symbolic::Expression> unit_vector_symbolic(1, 2, 3);
  symbolic::Expression vector_mag_squared_symbolic;
  DRAKE_EXPECT_NO_THROW(vector_mag_squared_symbolic = ThrowIfNotUnitVector(
                            unit_vector_symbolic, "TestSymbolicFunctionName"));
  EXPECT_EQ(vector_mag_squared_symbolic, 1.0);

  // Verify an exception is thrown for an invalid unit vector.
  Vector3<double> not_unit_vector(1.0, 2.0, 3.0);
  std::string expected_message =
      "SomeFunctionName\\(\\): The unit_vector argument 1 2 3 is"
      " not a unit vector.\n"
      "\\|unit_vector\\| = 3.74165738677\\d+\n"
      "\\|\\|unit_vector\\| - 1\\| = 2.74165738677\\d+ is greater than .*.";
  DRAKE_EXPECT_THROWS_MESSAGE(
      ThrowIfNotUnitVector(not_unit_vector, "SomeFunctionName"),
      expected_message);

  // Verify WarnIfNotUnitVector() returns ≈ 1 for near valid unit vector.
  vector_mag_squared = WarnIfNotUnitVector(unit_vector, "TestFunctionName");
  EXPECT_EQ(vector_mag_squared, unit_vector.squaredNorm());
  EXPECT_NE(vector_mag_squared, 1.0);  // For unit_vector = [1 + kepsilon, 0, 0]
  EXPECT_NEAR(vector_mag_squared, 1.0, 2.2 * kepsilon);

  // Verify return value from WarnIfNotUnitVector() ≠ 1 for invalid unit vector.
  // Not checked: A message should have be written to the log file.
  vector_mag_squared = WarnIfNotUnitVector(not_unit_vector, "SomeFunctionName");
  EXPECT_EQ(vector_mag_squared, not_unit_vector.squaredNorm());
  EXPECT_NE(vector_mag_squared, 1.0);

  // Verify WarnIfNotUnitVector() returns exactly 1.0 for symbolic unit vector.
  vector_mag_squared_symbolic =
      WarnIfNotUnitVector(unit_vector_symbolic, "TestSymbolicFunctionName");
  EXPECT_EQ(vector_mag_squared_symbolic, 1.0);

  // Verify an exception is thrown for a unit vector with NAN elements.
  not_unit_vector = Vector3<double>(NAN, NAN, NAN);
  expected_message =
      "SomeFunctionName\\(\\): The unit_vector argument nan nan nan is"
      " not a unit vector.\n"
      "\\|unit_vector\\| = nan\n"
      "\\|\\|unit_vector\\| - 1\\| = nan is greater than .*.";
  DRAKE_EXPECT_THROWS_MESSAGE(
      ThrowIfNotUnitVector(not_unit_vector, "SomeFunctionName"),
      expected_message);

  // Verify an exception is thrown for a unit vector with infinity elements.
  constexpr double kInfinity = std::numeric_limits<double>::infinity();
  not_unit_vector = Vector3<double>(kInfinity, kInfinity, kInfinity);
  expected_message =
      "SomeFunctionName\\(\\): The unit_vector argument inf inf inf is"
      " not a unit vector.\n"
      "\\|unit_vector\\| = inf\n"
      "\\|\\|unit_vector\\| - 1\\| = inf is greater than .*.";
  DRAKE_EXPECT_THROWS_MESSAGE(
      ThrowIfNotUnitVector(not_unit_vector, "SomeFunctionName"),
      expected_message);
}

GTEST_TEST(UnitVectorTest, AutoDiffTestForUnitVector) {
  // Values that make a perfect unit vector, i.e., ‖unit_vector‖ = 1.
  const double value0 = 1.0 / 3.0;
  const double value1 = 2.0 / 3.0;
  const double value2 = 2.0 / 3.0;
  const Vector3<double> values(value0, value1, value2);

  // Create an array of three variables v0, v1, v2.
  typedef Eigen::AutoDiffScalar<Eigen::VectorXd> Scalar;
  Vector3<Scalar> variables;  // Array of variables.

  // Initialize values and partial derivatives in the array of variables, i.e.,
  // variables[i].value() = valuei;         (i = 0, 1, 2).
  // variables[i].derivatives().resize(3);  (i = 0, 1, 2).
  // variables[i].derivatives()(i) = 1.0;   (i = 0, 1, 2).
  // variables[i].derivatives()(j) = 0.0;   (i, j = 0, 1, 2, with i ≠ j).
  InitializeAutoDiff(values, {}, {}, &variables);

  // Form a vector vec whose three elements are vi = variables[i] (i = 1, 2 3).
  Vector3<AutoDiffXd> vec(variables[0], variables[1], variables[2]);

  // Form ‖vec‖² which is a scalar function, herein denoted y = v0² + v1² + v2².
  // Also form partial derivatives of y = v0² + v1² + v2² with respect to vi.
  const Eigen::AutoDiffScalar y =
      vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
  const Eigen::MatrixXd y_partials = y.derivatives();  // Eigen's calculation.
  Eigen::MatrixXd partials_expected(3, 1);
  partials_expected(0, 0) = 2 * value0;  // ∂y/∂v0 = 2 * v0  (by-hand).
  partials_expected(1, 0) = 2 * value1;  // ∂y/∂v1 = 2 * v1  (by-hand).
  partials_expected(2, 0) = 2 * value2;  // ∂y/∂v2 = 2 * v2  (by-hand).

  // Compare calculations from Eigen's AutoDiff with by-hand calculations.
  constexpr double kTolerance =  4 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(y_partials, partials_expected, kTolerance,
                              MatrixCompareType::absolute));

  // Compare calculations from ThrowIfNotUnitVector() with by-hand calculations.
  const Eigen::AutoDiffScalar mag_squared =
      math::internal::ThrowIfNotUnitVector(v, "AutoDiffTestForUnitVector");
  EXPECT_NEAR(ExtractDoubleOrThrow(y),
              ExtractDoubleOrThrow(mag_squared), kTolerance);
  const Eigen::MatrixXd mag_squared_partials = mag_squared.derivatives();
  EXPECT_TRUE(CompareMatrices(mag_squared_partials, partials_expected,
                              kTolerance, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace internal
}  // namespace math
}  // namespace drake
