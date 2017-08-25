#include "drake/math/orthonormal_basis.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

using Vec3d = drake::Vector3<double>;

namespace drake {
namespace math {
namespace {

// Checks that the axes are set correctly.
void CheckBasis(const Matrix3<double>& R) {
  const double tol = 1e-12;
  const Vec3d a = R.col(0);
  const Vec3d b = R.col(1);
  const Vec3d c = R.col(2);
  EXPECT_NEAR(a.norm(), 1.0, tol);
  EXPECT_NEAR(b.norm(), 1.0, tol);
  EXPECT_NEAR(c.norm(), 1.0, tol);
  EXPECT_LT(a.dot(b), tol);
  EXPECT_LT(a.dot(c), tol);
  EXPECT_LT(b.dot(c), tol);
  EXPECT_TRUE(CompareMatrices(a.cross(b), c, tol, MatrixCompareType::absolute));
}

// Tests ComputeBasisFromX() produces a right-handed orthogonal matrix.
GTEST_TEST(ComputeBasisFromXTest, RightHandOrthogonal) {
  // Check a non-unit vector.
  Vec3d v(1, 1, 1);
  EXPECT_THROW(ComputeBasisFromX(v), std::logic_error);

  // Check a unit vector.
  v.normalize();
  CheckBasis(ComputeBasisFromX(v));

  // Check the x-, y- and z-axes.
  const Vec3d x_axis = Vec3d::UnitX();
  const Vec3d y_axis = Vec3d::UnitY();
  const Vec3d z_axis = Vec3d::UnitZ();
  CheckBasis(ComputeBasisFromX(x_axis));
  CheckBasis(ComputeBasisFromX(y_axis));
  CheckBasis(ComputeBasisFromX(z_axis));
}

// Tests ComputeBasisFromY() produces a right-handed orthogonal matrix.
GTEST_TEST(ComputeBasisFromYTest, RightHandOrthogonal) {
  // Check a non-unit vector.
  Vec3d v(1, 1, 1);
  EXPECT_THROW(ComputeBasisFromY(v), std::logic_error);

  // Check a unit vector.
  v.normalize();
  CheckBasis(ComputeBasisFromY(v));

  // Check the x-, y- and z-axes.
  const Vec3d x_axis = Vec3d::UnitX();
  const Vec3d y_axis = Vec3d::UnitY();
  const Vec3d z_axis = Vec3d::UnitZ();
  CheckBasis(ComputeBasisFromY(x_axis));
  CheckBasis(ComputeBasisFromY(y_axis));
  CheckBasis(ComputeBasisFromY(z_axis));
}

// Tests ComputeBasisFromZ() produces a right-handed orthogonal matrix.
GTEST_TEST(ComputeBasisFromZTest, RightHandOrthogonal) {
  // Check a non-unit vector.
  Vec3d v(1, 1, 1);
  EXPECT_THROW(ComputeBasisFromZ(v), std::logic_error);

  // Check a unit vector.
  v.normalize();
  CheckBasis(ComputeBasisFromZ(v));

  // Check the x-, y- and z-axes.
  const Vec3d x_axis = Vec3d::UnitX();
  const Vec3d y_axis = Vec3d::UnitY();
  const Vec3d z_axis = Vec3d::UnitZ();
  CheckBasis(ComputeBasisFromZ(x_axis));
  CheckBasis(ComputeBasisFromZ(y_axis));
  CheckBasis(ComputeBasisFromZ(z_axis));
}

}  // namespace
}  // namespace math
}  // namespace drake
