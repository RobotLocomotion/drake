#include "drake/math/unit_vector.h"

#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff.h"

namespace drake {
namespace math {
namespace internal {
namespace {

using Eigen::Vector3d;

GTEST_TEST(UnitVectorTest, ThrowIfNotUnitVector) {
  // Verify that no exception is thrown for a valid unit vector.
  Vector3<double> unit_vector(1.0, 0.0, 0.0);
  DRAKE_EXPECT_NO_THROW(
      ThrowIfNotUnitVector(unit_vector, "UnusedFunctionName"));

  // Verify that no exception is thrown for a valid or near valid unit vector.
  unit_vector = Vector3<double>(4.321, M_PI, 97531.2468).normalized();
  DRAKE_EXPECT_NO_THROW(
      ThrowIfNotUnitVector(unit_vector, "UnusedFunctionName"));

  // Verify that no exception is thrown when ‖unit_vector‖ is nearly 1.0.
  constexpr double epsilon = std::numeric_limits<double>::epsilon();
  unit_vector = Vector3<double>(1 + epsilon, 0, 0);
  DRAKE_EXPECT_NO_THROW(
      ThrowIfNotUnitVector(unit_vector, "UnusedFunctionName"));

  // Verify that no exception is thrown when unit_vector is symbolic.
  const Vector3<symbolic::Expression> unit_vector_symbolic(1, 2, 3);
  DRAKE_EXPECT_NO_THROW(
      ThrowIfNotUnitVector(unit_vector_symbolic, "TestSymbolicFunctionName"));

  // Verify an exception is thrown for an invalid unit vector.
  Vector3<double> not_unit_vector(1.0, 2.0, 3.0);
  std::string expected_message =
      "SomeFunctionName\\(\\): The unit_vector argument \\[1, 2, 3\\]ᵀ is"
      " not a unit vector.\n"
      "\\|unit_vector\\| = 3.74165738677\\d+\n"
      "\\|\\|unit_vector\\| - 1\\| = 2.74165738677\\d+ is greater than .*.";
  DRAKE_EXPECT_THROWS_MESSAGE(
      ThrowIfNotUnitVector(not_unit_vector, "SomeFunctionName"),
      expected_message);

  // Verify an exception is thrown for a unit vector with NAN elements.
  not_unit_vector = Vector3<double>(NAN, NAN, NAN);
  expected_message =
      "SomeFunctionName\\(\\): The unit_vector argument \\[nan, nan, nan\\]ᵀ is"
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
      "SomeFunctionName\\(\\): The unit_vector argument \\[inf, inf, inf\\]ᵀ is"
      " not a unit vector.\n"
      "\\|unit_vector\\| = inf\n"
      "\\|\\|unit_vector\\| - 1\\| = inf is greater than .*.";
  DRAKE_EXPECT_THROWS_MESSAGE(
      ThrowIfNotUnitVector(not_unit_vector, "SomeFunctionName"),
      expected_message);
}

GTEST_TEST(UnitVectorTest, ThrowIfNotUnitVectorAutoDiffXd) {
  // AutoDiffXd unit vectors should accept near-unit values without throwing.
  const Vector3<AutoDiffXd> unit =
      InitializeAutoDiff(Vector3d(1.0, 0.0, 0.0));
  DRAKE_EXPECT_NO_THROW(ThrowIfNotUnitVector(unit, "AutoDiffUnit"));

  // Non-unit AutoDiffXd vectors still throw (based on discarded gradient mag).
  const Vector3<AutoDiffXd> not_unit = InitializeAutoDiff(Vector3d(1, 2, 3));
  DRAKE_EXPECT_THROWS_MESSAGE(ThrowIfNotUnitVector(not_unit, "AutoDiffNotUnit"),
                              "AutoDiffNotUnit\\(\\): The unit_vector argument "
                              ".* is not a unit vector.*");
}

GTEST_TEST(UnitVectorTest, NormalizeOrThrowDouble) {
  const Vector3d v(3.0, 0.0, 4.0);
  const Vector3d u = NormalizeOrThrow(v, "NormalizeOrThrow");
  EXPECT_TRUE(CompareMatrices(u, Vector3d(0.6, 0.0, 0.8), 1e-14));

  // Symbolic is a no-op that does not throw even for a zero vector.
  const Vector3<symbolic::Expression> v_sym(0, 0, 0);
  DRAKE_EXPECT_NO_THROW(NormalizeOrThrow(v_sym, "SymbolicNormalize"));

  constexpr double kInfinity = std::numeric_limits<double>::infinity();
  DRAKE_EXPECT_THROWS_MESSAGE(
      NormalizeOrThrow(Vector3d(0, 0, 0), "ZeroVec"),
      "ZeroVec\\(\\) cannot normalize the given vector v\\..*"
      "magnitude of at least .*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      NormalizeOrThrow(Vector3d(NAN, 1, 0), "NanVec"),
      "NanVec\\(\\) cannot normalize the given vector v\\..*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      NormalizeOrThrow(Vector3d(kInfinity, 0, 0), "InfVec"),
      "InfVec\\(\\) cannot normalize the given vector v\\..*");

  // Slightly below the 1e-10 magnitude threshold must throw.
  constexpr double kTolerance = 2 * std::numeric_limits<double>::epsilon();
  DRAKE_EXPECT_THROWS_MESSAGE(
      NormalizeOrThrow(Vector3d(1.0E-10 - kTolerance, 0, 0), "TinyVec"),
      "TinyVec\\(\\) cannot normalize the given vector v\\..*");
}

// Confirms NormalizeOrThrow preserves AutoDiffXd derivatives. This is the
// regression coverage requested by #20405 after the fix in #20406/#20423.
GTEST_TEST(UnitVectorTest, NormalizeOrThrowAutoDiffXd) {
  const Vector3d v_double(3.0, 0.0, 4.0);
  Eigen::Matrix3d v_grad = Eigen::Matrix3d::Identity();
  const Vector3<AutoDiffXd> v = InitializeAutoDiff(v_double, v_grad);

  const Vector3<AutoDiffXd> u = NormalizeOrThrow(v, "NormalizeOrThrow");

  // Value matches ordinary normalize.
  EXPECT_TRUE(CompareMatrices(ExtractValue(u), Vector3d(0.6, 0.0, 0.8), 1e-14));

  // Dividing by an AutoDiffXd norm (not ExtractDoubleOrThrow) keeps gradients.
  // Analytical Jacobian of v/||v|| is (I - u u^T) / ||v||.
  const Vector3d u_double = v_double.normalized();
  const double norm = v_double.norm();
  const Eigen::Matrix3d J =
      (Eigen::Matrix3d::Identity() - u_double * u_double.transpose()) / norm;
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(CompareMatrices(u[i].derivatives(), J.row(i).transpose(), 1e-12))
        << "row " << i;
  }

  // Cross-check against a direct AutoDiffXd normalize (same math as the fix).
  const Vector3<AutoDiffXd> u_ref = v / v.norm();
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(u[i].value(), u_ref[i].value(), 1e-14);
    EXPECT_TRUE(CompareMatrices(u[i].derivatives(), u_ref[i].derivatives(),
                                1e-14));
  }

  // Non-finite / tiny AutoDiffXd inputs still throw.
  DRAKE_EXPECT_THROWS_MESSAGE(
      NormalizeOrThrow(InitializeAutoDiff(Vector3d::Zero()), "ADZero"),
      "ADZero\\(\\) cannot normalize the given vector v\\..*");
}

}  // namespace
}  // namespace internal
}  // namespace math
}  // namespace drake
