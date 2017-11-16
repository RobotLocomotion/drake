#include "drake/multibody/multibody_tree/math/rotation_matrix.h"

#include <iomanip>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::NumTraits;
using Eigen::Vector3d;

#ifdef DRAKE_ASSERT_IS_DISARMED
// With assertion disarmed, expect no exception.
#define EXPECT_THROW_IF_ARMED(expression, exception) \
do {\
  EXPECT_NO_THROW(expression); \
} while (0)

#else

#define EXPECT_THROW_IF_ARMED(expression, exception) \
do { \
  EXPECT_THROW(expression, exception); \
} while (0)
#endif

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Test default constructor - should be identity matrix.
GTEST_TEST(RotationMatrix, DefaultRotationMatrixIsIdentity) {
  RotationMatrix<double> R;
  Matrix3d zero_matrix = R.get_as_Matrix3() - Matrix3d::Identity();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test making a RotationMatrix from a Matrix3 (unchecked).
GTEST_TEST(RotationMatrix, MakeMatrix3Unchecked) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;

  RotationMatrix<double> R = RotationMatrix<double>::MakeUnchecked(m);
  Matrix3d zero_matrix = m - R.get_as_Matrix3();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test making a RotationMatrix from a Matrix3 (checked).
GTEST_TEST(RotationMatrix, MakeMatrix3Checked) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;

  RotationMatrix<double> R = RotationMatrix<double>::MakeChecked(m, 5*kEpsilon);
  Matrix3d zero_matrix = m - R.get_as_Matrix3();
  EXPECT_TRUE((zero_matrix.array() == 0).all());

  // TODO(Mitiguy) Implement test to check throwing assertion.
  // m << 1, 10*kEpsilon, 10*kEpsilon,
  //      0, cos_theta, sin_theta,
  //      0, -sin_theta, cos_theta;
  // EXPECT_THROW_IF_ARMED(RotationMatrix<double>::MakeChecked(m, 5*kEpsilon),
  //                       std::logic_error);
}


// Test transpose.
GTEST_TEST(RotationMatrix, Transpose) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;

  RotationMatrix<double> R = RotationMatrix<double>::MakeChecked(m, 5*kEpsilon);
  RotationMatrix<double> R_transpose = R.transpose();
  Matrix3d zero_matrix = m.transpose() - R_transpose.get_as_Matrix3();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test inverse.
GTEST_TEST(RotationMatrix, Inverse) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;

  RotationMatrix<double> R = RotationMatrix<double>::MakeChecked(m, 5*kEpsilon);
  RotationMatrix<double> R_inverse = R.inverse();
  RotationMatrix<double> should_be_identity = R * R_inverse;
  RotationMatrix<double> identity_matrix;
  EXPECT_TRUE(should_be_identity.IsNearlyEqualTo(identity_matrix, 5*kEpsilon));
}

// Test access by (i, j) indexes.
GTEST_TEST(RotationMatrix, AccessByIndexes) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;

  RotationMatrix<double> R = RotationMatrix<double>::MakeChecked(m, 5*kEpsilon);
  EXPECT_EQ(R(0, 0), m(0, 0));
  EXPECT_EQ(R(0, 1), m(0, 1));
  EXPECT_EQ(R(0, 2), m(0, 2));
  EXPECT_EQ(R(1, 0), m(1, 0));
  EXPECT_EQ(R(1, 1), m(1, 1));
  EXPECT_EQ(R(1, 2), m(1, 2));
  EXPECT_EQ(R(2, 0), m(2, 0));
  EXPECT_EQ(R(2, 1), m(2, 1));
  EXPECT_EQ(R(2, 2), m(2, 2));
}

// Test matrix multiply.
GTEST_TEST(RotationMatrix, MultiplyRotationMatrices) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m1, m2;
  m1 << 1, 0, 0,
        0, cos_theta, sin_theta,
        0, -sin_theta, cos_theta;
  m2 << cos_theta, sin_theta, 0,
       -sin_theta, cos_theta, 0,
        0, 0, 1;
  Matrix3d m_mult = m1 * m2;

  RotationMatrix<double> R1, R2;
  R1 = RotationMatrix<double>::MakeChecked(m1, 5*kEpsilon);
  R2 = RotationMatrix<double>::MakeChecked(m2, 5*kEpsilon);
  Matrix3d zero_matrix = m_mult - (R1 * R2).get_as_Matrix3();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test IsValidRotationMatrix.
GTEST_TEST(RotationMatrix, IsValidRotationMatrix) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;

  RotationMatrix<double> R = RotationMatrix<double>::MakeChecked(m, 5*kEpsilon);
  EXPECT_TRUE(R.IsValidRotationMatrix(5*kEpsilon));

  m << 1, 10*kEpsilon, 10*kEpsilon,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  R = RotationMatrix<double>::MakeUnchecked(m);
  EXPECT_FALSE(R.IsValidRotationMatrix(5*kEpsilon));
}


}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
