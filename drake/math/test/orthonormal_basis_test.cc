#include "drake/math/orthonormal_basis.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

using Eigen::Vector3d;

namespace drake {
namespace math {
namespace {

// Checks that the axes are set correctly.
void CheckBasisOrthonomality(const Matrix3<double>& R) {
  const double tol = 10 * std::numeric_limits<double>::epsilon();
  const Vector3d a = R.col(0);
  const Vector3d b = R.col(1);
  const Vector3d c = R.col(2);
  EXPECT_NEAR(a.norm(), 1.0, tol);
  EXPECT_NEAR(b.norm(), 1.0, tol);
  EXPECT_NEAR(c.norm(), 1.0, tol);
  EXPECT_LT(a.dot(b), tol);
  EXPECT_LT(a.dot(c), tol);
  EXPECT_LT(b.dot(c), tol);
  EXPECT_TRUE(CompareMatrices(a.cross(b), c, tol, MatrixCompareType::absolute));
}

// Checks that the axes are set correctly *and* the appropriate axis of the
// 3x3 matrix is set.
void CheckBasis(int axis_index, const Vector3<double>& axis) {
  Matrix3<double> R = ComputeBasisFromAxis(axis_index, axis);
  CheckBasisOrthonomality(R);

  // Verify that the vector ends up in the correct column.
  const double eps = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(axis.dot(R.col(axis_index))/axis.norm(), 1.0, eps);
}

// Tests ComputeBasisFromAxis() produces a right-handed orthogonal matrix.
GTEST_TEST(ComputeBasisFromAxisTest, RightHandOrthogonal) {
  // Iterate over all three axes.
  for (int i = 0; i < 3; ++i) {
    // Check a non-unit vector.
    Vector3d v(1, 1, 1);
    CheckBasisOrthonomality(ComputeBasisFromAxis(i, v));

    // Check a zero vector.
    v.setZero();
    EXPECT_THROW(ComputeBasisFromAxis(i, v), std::logic_error);

    // Check the x-, y- and z-axes.
    const Vector3d x_axis = Vector3d::UnitX();
    const Vector3d y_axis = Vector3d::UnitY();
    const Vector3d z_axis = Vector3d::UnitZ();
    CheckBasis(i, x_axis);
    CheckBasis(i, y_axis);
    CheckBasis(i, z_axis);
  }
}

}  // namespace
}  // namespace math
}  // namespace drake
