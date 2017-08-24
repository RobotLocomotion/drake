#include "drake/math/orthonormal_basis.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

using Vec3d = drake::Vector3<double>;

namespace drake {
namespace math {
namespace {

// Checks that the axes are set correctly.
void CheckAxes(const Vec3d& a, const Vec3d& b, const Vec3d& c) {
  const double tol = 1e-12;
  EXPECT_NEAR(a.norm(), 1.0, tol);
  EXPECT_NEAR(b.norm(), 1.0, tol);
  EXPECT_NEAR(c.norm(), 1.0, tol);
  EXPECT_LT(a.dot(b), tol);
  EXPECT_LT(a.dot(c), tol);
  EXPECT_LT(b.dot(c), tol);
  EXPECT_TRUE(CompareMatrices(a.cross(b), c, tol, MatrixCompareType::absolute));
}

// Tests that CalcOrthonormalBasis() computes the correct result for input
// of the x-axis.
GTEST_TEST(OrthnormalBasisTest, XAxis) {
  Vec3d xaxis = Vec3d::UnitX();
  Vec3d v1, v2;
  CalcOrthonormalBasis(&xaxis, &v1, &v2);

  // Verify that x-axis has not changed.
  CheckAxes(xaxis, v1, v2);

  // Scale the x-axis, and make sure that the x-axis becomes normalized and
  // the v1 and v2 directions are still correct.
  xaxis *= 10;
  CalcOrthonormalBasis(&xaxis, &v1, &v2);
  CheckAxes(xaxis, v1, v2);
}

// Tests that CalcOrthonormalBasis() computes the correct result for input
// of the y-axis.
GTEST_TEST(OrthnormalBasisTest, YAxis) {
  Vec3d yaxis = Vec3d::UnitY();
  Vec3d v1, v2;
  CalcOrthonormalBasis(&yaxis, &v1, &v2);
}

// Tests that CalcOrthonormalBasis() computes the correct result for input
// of the z-axis.
GTEST_TEST(OrthnormalBasisTest, ZAxis) {
  Vec3d zaxis = Vec3d::UnitZ();
  Vec3d v1, v2;
  CalcOrthonormalBasis(&zaxis, &v1, &v2);
}

// Tests that CalcOrthonormalBasis() computes the correct result for input
// of a vector with equal components in all directions.
GTEST_TEST(OrthnormalBasisTest, EqualComponents) {
  Vec3d v(1, 1, 1);
  Vec3d v1, v2;
  CalcOrthonormalBasis(&v, &v1, &v2);
}

// Tests CalcOrthonormalBasis() exception handling.
GTEST_TEST(OrthnormalBasisTest, Exceptions) {
  Vec3d zero(0, 0, 0);
  Vec3d xaxis = Vec3d::UnitX();
  Vec3d v1, v2;
  Vec3d* null = nullptr;
  EXPECT_THROW(CalcOrthonormalBasis(&zero, &v1, &v2), std::logic_error);
  EXPECT_THROW(CalcOrthonormalBasis(&xaxis, null, &v2), std::logic_error);
  EXPECT_THROW(CalcOrthonormalBasis(&xaxis, &v1, null), std::logic_error);
}

// Tests ComputeBasisFromX() produces a right-handed orthogonal matrix.
GTEST_TEST(ComputeBasisFromXTest, RightHandOrthogonal) {
  Vec3d v(1, 1, 1);
  EXPECT_THROW(ComputeBasisFromX(v), std::logic_error);
  v.normalize();
  Matrix3<double> R = ComputeBasisFromX(v);
  EXPECT_NEAR(R.determinant(), 1.0, std::numeric_limits<double>::epsilon());
}

// Tests ComputeBasisFromY() produces a right-handed orthogonal matrix.
GTEST_TEST(ComputeBasisFromYTest, RightHandOrthogonal) {
  Vec3d v(1, 1, 1);
  EXPECT_THROW(ComputeBasisFromY(v), std::logic_error);
  v.normalize();
  Matrix3<double> R = ComputeBasisFromY(v);
  EXPECT_NEAR(R.determinant(), 1.0, std::numeric_limits<double>::epsilon());
}

// Tests ComputeBasisFromZ() produces a right-handed orthogonal matrix.
GTEST_TEST(ComputeBasisFromZTest, RightHandOrthogonal) {
  Vec3d v(1, 1, 1);
  EXPECT_THROW(ComputeBasisFromY(v), std::logic_error);
  v.normalize();
  Matrix3<double> R = ComputeBasisFromZ(v);
  EXPECT_NEAR(R.determinant(), 1.0, std::numeric_limits<double>::epsilon());
}

}  // namespace
}  // namespace math
}  // namespace drake
