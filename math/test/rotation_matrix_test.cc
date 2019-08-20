#include "drake/math/rotation_matrix.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace math {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();


// Test default constructor - should be identity matrix.
GTEST_TEST(RotationMatrix, DefaultRotationMatrixIsIdentity) {
  RotationMatrix<double> R;
  Matrix3d zero_matrix = R.matrix() - Matrix3d::Identity();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test constructing a RotationMatrix from a Matrix3.
GTEST_TEST(RotationMatrix, RotationMatrixConstructor) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;

  RotationMatrix<double> R1(m);
  Matrix3d zero_matrix = m - R1.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());

  if (kDrakeAssertIsArmed) {
    // Really poor non-orthogonal matrix should throw an exception.
    m << 1, 2,  3,
         4, 5,  6,
         7, 8, -10;
    DRAKE_EXPECT_THROWS_MESSAGE(
        RotationMatrix<double>{m}, std::logic_error,
        "Error: Rotation matrix is not orthonormal[\\s\\S]*");

    // Barely non-orthogonal matrix should throw an exception.
    m << 1, 9000*kEpsilon, 9000*kEpsilon,
         0, cos_theta, sin_theta,
         0, -sin_theta, cos_theta;
    DRAKE_EXPECT_THROWS_MESSAGE(
        RotationMatrix<double>{m}, std::logic_error,
        "Error: Rotation matrix is not orthonormal[\\s\\S]*");

    // Orthogonal matrix with determinant = -1 should throw an exception.
    m << 1, 0, 0,
         0, 1, 0,
         0, 0, -1;
    DRAKE_EXPECT_THROWS_MESSAGE(
        RotationMatrix<double>{m}, std::logic_error,
        "Error: Rotation matrix determinant is negative.*");

    // Matrix with a NaN should throw an exception.
    m << 1, 0, 0,
         0, 1, 0,
         0, 0, std::numeric_limits<double>::quiet_NaN();
    DRAKE_EXPECT_THROWS_MESSAGE(RotationMatrix<double>{m}, std::logic_error,
        "Error: Rotation matrix contains an element that is infinity or NaN.*");

    // Matrix with an infinity should throw an exception.
    m << 1, 0, 0,
         0, 1, 0,
         0, 0, std::numeric_limits<double>::infinity();
    DRAKE_EXPECT_THROWS_MESSAGE(RotationMatrix<double>{m}, std::logic_error,
        "Error: Rotation matrix contains an element that is infinity or NaN.*");
  }
}

// Test making a RotationMatrix from three right-handed orthogonal unit vectors.
GTEST_TEST(RotationMatrix, MakeFromOrthonormalRowsOrColumns) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);

  // Construct a matrix whose rows are right-handed orthogonal unit vectors.
  const Vector3d Ax(1, 0, 0);
  const Vector3d Ay(0, cos_theta, -sin_theta);
  const Vector3d Az(0, sin_theta,  cos_theta);
  Matrix3d m_row;
  m_row.row(0) = Ax;
  m_row.row(1) = Ay;
  m_row.row(2) = Az;

  // Make the rows of a RotationMatrix R from the unit vectors Ax, Ay, Az.
  // Ensure m_row is identical to the 3x3 matrix underlying R.
  RotationMatrix<double> R =
      RotationMatrix<double>::MakeFromOrthonormalRows(Ax, Ay, Az);
  const Matrix3d zero_row_matrix = m_row - R.matrix();
  EXPECT_TRUE((zero_row_matrix.array() == 0).all());

  // Construct a matrix whose columns are right-handed orthogonal unit vectors.
  Matrix3d m_column;
  m_column.col(0) = Ax;
  m_column.col(1) = Ay;
  m_column.col(2) = Az;

  // Make the columns of a RotationMatrix R2 from the unit vectors Ax, Ay, Az.
  // Ensure m_column is identical to the 3x3 matrix underlying R2.
  RotationMatrix<double> R2 =
      RotationMatrix<double>::MakeFromOrthonormalColumns(Ax, Ay, Az);
  const Matrix3d zero_column_matrix = m_column - R2.matrix();
  EXPECT_TRUE((zero_column_matrix.array() == 0).all());

  // Test that RotationMatrix R2 is the inverse (transpose) of R2.
  EXPECT_TRUE(R.IsExactlyEqualTo(R2.inverse()));
  EXPECT_TRUE(R.IsExactlyEqualTo(R2.transpose()));

  // The next test intentionally creates an invalid RotationMatrix that deviates
  // from a valid RotationMatrix by a factor of 8.  The factor of 8 times the
  // internal orthonormality tolerance provides a tolerance to the loss of up to
  // three bits of precision (2^3 = 8) and allows for possible imprecision
  // issues associated with variations in compilers, operating systems, etc.
  const double delta =
      8 * RotationMatrix<double>::get_internal_tolerance_for_orthonormality();
  const Vector3d Fx(1, 0, delta);
  const Vector3d Fy(0, cos_theta, -sin_theta);
  const Vector3d Fz(0, sin_theta,  cos_theta);

  // Non-orthogonal matrix should throw an exception (at least in debug builds).
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      R = RotationMatrix<double>::MakeFromOrthonormalRows(Fx, Fy, Fz),
      std::logic_error, "Error: Rotation matrix is not orthonormal[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      R = RotationMatrix<double>::MakeFromOrthonormalColumns(Fx, Fy, Fz),
      std::logic_error, "Error: Rotation matrix is not orthonormal[\\s\\S]*");

  // Non-right handed matrix with determinant < 0 should throw an exception.
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      R = RotationMatrix<double>::MakeFromOrthonormalRows(
          Vector3d(-1, 0, 0), Fy, Fz), std::logic_error,
      "Error: Rotation matrix determinant is negative.*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      R = RotationMatrix<double>::MakeFromOrthonormalColumns(
          Vector3d(-1, 0, 0), Fy, Fz), std::logic_error,
      "Error: Rotation matrix determinant is negative.*");

  // Matrix with a NaN should throw an exception.
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      R = RotationMatrix<double>::MakeFromOrthonormalRows(
          Vector3d(std::numeric_limits<double>::quiet_NaN(), 0, 0), Fy, Fz),
      std::logic_error,
      "Error: Rotation matrix contains an element that is infinity or NaN.*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      R = RotationMatrix<double>::MakeFromOrthonormalColumns(
          Vector3d(std::numeric_limits<double>::quiet_NaN(), 0, 0), Fy, Fz),
          std::logic_error,
        "Error: Rotation matrix contains an element that is infinity or NaN.*");

  // Matrix with an infinity should throw an exception.
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      R = RotationMatrix<double>::MakeFromOrthonormalRows(
          Vector3d(std::numeric_limits<double>::infinity(), 0, 0), Fy, Fz),
      std::logic_error,
      "Error: Rotation matrix contains an element that is infinity or NaN.*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      R = RotationMatrix<double>::MakeFromOrthonormalColumns(
          Vector3d(std::numeric_limits<double>::infinity(), 0, 0), Fy, Fz),
          std::logic_error,
        "Error: Rotation matrix contains an element that is infinity or NaN.*");

  if (kDrakeAssertIsDisarmed) {
    // In release builds, check for invalid matrix.
    EXPECT_NO_THROW(
        R = RotationMatrix<double>::MakeFromOrthonormalRows(Fx, Fy, Fz));
    EXPECT_FALSE(R.IsValid());
    EXPECT_NO_THROW(
        R = RotationMatrix<double>::MakeFromOrthonormalColumns(Fx, Fy, Fz));
    EXPECT_FALSE(R.IsValid());
  }
}

// Test setting a RotationMatrix from a Matrix3.
GTEST_TEST(RotationMatrix, SetRotationMatrix) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;

  RotationMatrix<double> R;
  R.set(m);
  Matrix3d zero_matrix = m - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());

  // Bad matrix should throw exception.
  m << 1, 9000*kEpsilon, 9000*kEpsilon,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  if (kDrakeAssertIsArmed) {
    EXPECT_THROW(R.set(m), std::logic_error);
  } else {
    EXPECT_NO_THROW(R.set(m));
  }
}

// Test getting rows or columns from a RotationMatrix.
GTEST_TEST(RotationMatrix, GetRowsOrColumnsFromRotationmatrix) {
  const double q = 1.2345;  // Angle in radians.
  RotationMatrix<double> R_AB = RotationMatrix<double>::MakeZRotation(q);
  const Vector3d Ax_B = R_AB.row(0);
  const Vector3d Ay_B = R_AB.row(1);
  const Vector3d Az_B = R_AB.row(2);
  const Vector3d Bx_A = R_AB.col(0);
  const Vector3d By_A = R_AB.col(1);
  const Vector3d Bz_A = R_AB.col(2);

  constexpr double tolerance = 32 * kEpsilon;
  const double cos_q = std::cos(q);
  const double sin_q = std::sin(q);
  EXPECT_TRUE(CompareMatrices(Ax_B, Vector3d(cos_q, -sin_q, 0), tolerance));
  EXPECT_TRUE(CompareMatrices(Ay_B, Vector3d(sin_q, cos_q, 0), tolerance));
  EXPECT_TRUE(CompareMatrices(Az_B, Vector3d(0, 0, 1), tolerance));
  EXPECT_TRUE(CompareMatrices(Bx_A, Vector3d(cos_q, sin_q, 0), tolerance));
  EXPECT_TRUE(CompareMatrices(By_A, Vector3d(-sin_q, cos_q, 0), tolerance));
  EXPECT_TRUE(CompareMatrices(Bz_A, Vector3d(0, 0, 1), tolerance));
}

// Test setting a RotationMatrix to an identity matrix.
GTEST_TEST(RotationMatrix, MakeIdentityMatrix) {
  const RotationMatrix<double>& R = RotationMatrix<double>::Identity();
  Matrix3d zero_matrix = Matrix3<double>::Identity() - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test making a rotation matrix associated with a X, Y, or Z-rotation.
GTEST_TEST(RotationMatrix, MakeXRotationMakeYRotationMakeZRotation) {
  const Vector3d i = Eigen::Vector3d::UnitX();
  const Vector3d j = Eigen::Vector3d::UnitY();
  const Vector3d k = Eigen::Vector3d::UnitZ();
  constexpr double tolerance = 32 * kEpsilon;

  // Test making a rotation matrix associated with X-rotation.
  double theta = 0.3;
  Matrix3d m = Eigen::AngleAxisd(theta, Vector3d::UnitX()).matrix();
  RotationMatrix<double> R = RotationMatrix<double>::MakeXRotation(theta);
  Matrix3d zero_matrix = m - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
  EXPECT_TRUE(CompareMatrices(RotationMatrixd::MakeXRotation(M_PI_4) * i,
                              i, tolerance));
  EXPECT_TRUE(CompareMatrices(RotationMatrixd::MakeXRotation(M_PI_4) * j,
                              Vector3d(0, M_SQRT1_2, M_SQRT1_2), tolerance));
  EXPECT_TRUE(CompareMatrices(RotationMatrixd::MakeXRotation(M_PI_4) * k,
                              Vector3d(0, -M_SQRT1_2, M_SQRT1_2), tolerance));

  // Test making a rotation matrix associated with Y-rotation.
  theta = 0.4;
  m = Eigen::AngleAxisd(theta, Vector3d::UnitY()).matrix();
  R = RotationMatrix<double>::MakeYRotation(theta);
  zero_matrix = m - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
  EXPECT_TRUE(CompareMatrices(RotationMatrixd::MakeYRotation(M_PI_4) * i,
                              Vector3d(M_SQRT1_2, 0, -M_SQRT1_2), tolerance));
  EXPECT_TRUE(CompareMatrices(RotationMatrixd::MakeYRotation(M_PI_4) * j,
                              j, tolerance));
  EXPECT_TRUE(CompareMatrices(RotationMatrixd::MakeYRotation(M_PI_4) * k,
                              Vector3d(M_SQRT1_2, 0, M_SQRT1_2), tolerance));

  // Test making a rotation matrix associated with Z-rotation.
  theta = 0.5;
  m = Eigen::AngleAxisd(theta, Vector3d::UnitZ()).matrix();
  R = RotationMatrix<double>::MakeZRotation(theta);
  zero_matrix = m - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
  EXPECT_TRUE(CompareMatrices(RotationMatrixd::MakeZRotation(M_PI_4) * i,
                              Vector3d(M_SQRT1_2, M_SQRT1_2, 0), tolerance));
  EXPECT_TRUE(CompareMatrices(RotationMatrixd::MakeZRotation(M_PI_4) * j,
                              Vector3d(-M_SQRT1_2, M_SQRT1_2, 0), tolerance));
  EXPECT_TRUE(CompareMatrices(RotationMatrixd::MakeZRotation(M_PI_4) * k,
                              k, tolerance));

  // Test that rotation by Pi + theta does not change the rotation axis and
  // flips the signs on the other two axes.
  RotationMatrixd RA, RB;
  RA = RotationMatrixd::MakeXRotation(M_PI + theta);
  RB = RotationMatrixd(Eigen::DiagonalMatrix<double, 3>(1, -1, -1)) *
       RotationMatrixd::MakeXRotation(theta);
  EXPECT_TRUE(RA.IsNearlyEqualTo(RB, tolerance));

  RA = RotationMatrixd::MakeYRotation(M_PI + theta);
  RB = RotationMatrixd(Eigen::DiagonalMatrix<double, 3>(-1, 1, -1)) *
       RotationMatrixd::MakeYRotation(theta);
  EXPECT_TRUE(RA.IsNearlyEqualTo(RB, tolerance));

  RA = RotationMatrixd::MakeZRotation(M_PI + theta);
  RB = RotationMatrixd(Eigen::DiagonalMatrix<double, 3>(-1, -1, 1)) *
       RotationMatrixd::MakeZRotation(theta);
  EXPECT_TRUE(RA.IsNearlyEqualTo(RB, tolerance));
}

// Test making a rotation matrix from a RollPitchYaw rotation sequence (which is
// equivalent to a Body-fixed Z-Y-X or a Space-fixed X-Y-Z rotation sequence).
// Also tests method IsExactlyEqualTo() and typedef (using) RotationMatrixd.
GTEST_TEST(RotationMatrix, ConstructorWithRollPitchYaw) {
  const double r(0.5), p(0.4), y(0.3);
  const RollPitchYaw<double> rpy(r, p, y);
  const Matrix3d m = (Eigen::AngleAxisd(y, Vector3d::UnitZ())
                    * Eigen::AngleAxisd(p, Vector3d::UnitY())
                    * Eigen::AngleAxisd(r, Vector3d::UnitX())).matrix();
  const RotationMatrix<double> R_eigen(m);
  const RotationMatrix<double> R_rpy(rpy);
  EXPECT_TRUE(R_rpy.IsNearlyEqualTo(R_eigen, kEpsilon));

  RotationMatrixd R1 = RotationMatrix<double>::MakeZRotation(y);
  RotationMatrixd R2 = RotationMatrix<double>::MakeYRotation(p);
  RotationMatrixd R3 = RotationMatrix<double>::MakeXRotation(r);
  RotationMatrixd R_expected = R1 * R2 * R3;
  EXPECT_TRUE(R_rpy.IsExactlyEqualTo(R_expected));
}

// Test calculating the inverse and transpose of a RotationMatrix.
GTEST_TEST(RotationMatrix, InverseAndTranspose) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
      0, cos_theta, sin_theta,
      0, -sin_theta, cos_theta;
  RotationMatrix<double> R(m);
  RotationMatrix<double> RRinverse   = R * R.inverse();
  RotationMatrix<double> RRtranspose = R * R.transpose();
  const RotationMatrix<double>& I = RotationMatrix<double>::Identity();
  EXPECT_TRUE(RRinverse.IsNearlyEqualTo(I, 8 * kEpsilon));
  EXPECT_TRUE(RRtranspose.IsNearlyEqualTo(I, 8 * kEpsilon));
}

// Test rotation matrix multiplication and IsNearlyEqualTo.
GTEST_TEST(RotationMatrix, OperatorMultiplyAndIsNearlyEqualTo) {
  const RollPitchYaw<double> rpy0(0.2, 0.3, 0.4);
  const RollPitchYaw<double> rpy1(-0.5, -0.6, 0.9);
  const RotationMatrix<double> R_BA(rpy0);
  const RotationMatrix<double> R_CB(rpy1);
  const RotationMatrix<double> R_CA = R_CB * R_BA;
  const Matrix3d m_BA = R_BA.matrix();
  const Matrix3d m_CB = R_CB.matrix();
  const Matrix3d m_CA = m_CB * m_BA;
  const RotationMatrix<double> R_CA_manual_multiply(m_CA);

  // Test operator *().
  EXPECT_TRUE(
      R_CA.IsNearlyEqualTo(R_CA_manual_multiply, 10 * kEpsilon));

  // Also test IsNearlyEqualTo.
  EXPECT_FALSE(R_CA.IsNearlyEqualTo(R_CB, 10000 * kEpsilon));

  // Also test operator*=().
  RotationMatrix<double> R_CA_times_equal_test = R_CB;
  R_CA_times_equal_test *= R_BA;
  EXPECT_TRUE(
      R_CA_times_equal_test.IsNearlyEqualTo(R_CA, 10 * kEpsilon));
  EXPECT_FALSE(
      R_CA_times_equal_test.IsNearlyEqualTo(R_CB, 10000 * kEpsilon));

  // Also test operator*() with vectors.
  const Vector3d vA(1, 2, 3);     // Vector v expressed in frame A.
  const Vector3d vC = R_CA * vA;  // Vector v expressed in frame C.
  const Vector3d vC_expected = m_CA * vA;
  EXPECT_TRUE(vC.isApprox(vC_expected));
}

// Test IsOrthonormal, IsValid.
GTEST_TEST(RotationMatrix, IsValid) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  EXPECT_GT(m.determinant(), 0);
  EXPECT_TRUE(RotationMatrix<double>::IsOrthonormal(m, 5 * kEpsilon));
  EXPECT_TRUE(RotationMatrix<double>::IsValid(m, 5 * kEpsilon));

  // Test a matrix that should fail orthonormality check.
  m << 1, 10 * kEpsilon, 10 * kEpsilon,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  EXPECT_GT(m.determinant(), 0);
  EXPECT_FALSE(RotationMatrix<double>::IsOrthonormal(m, 5 * kEpsilon));
  EXPECT_FALSE(RotationMatrix<double>::IsValid(m, 5 * kEpsilon));

  // Test a matrix that should fail determinant test.
  m << -1, 0, 0,
        0, cos_theta, sin_theta,
        0, -sin_theta, cos_theta;
  EXPECT_LT(m.determinant(), 0);
  EXPECT_TRUE(RotationMatrix<double>::IsOrthonormal(m, 5 * kEpsilon));
  EXPECT_FALSE(RotationMatrix<double>::IsValid(m, 5 * kEpsilon));
}

// Tests whether or not a RotationMatrix is an identity matrix.
GTEST_TEST(RotationMatrix, IsExactlyIdentity) {
  // Test that the default constructor creates an exact identity matrix.
  RotationMatrix<double> R;
  EXPECT_TRUE(R.IsExactlyIdentity());

  // Test that setting R to an identity matrix does not throw an exception.
  Matrix3d m;
  m << 1, 0, 0,
       0, 1, 0,
       0, 0, 1;
  R.set(m);
  EXPECT_TRUE(R.IsExactlyIdentity());

  // Test impact of absolute mininimum deviation from identity matrix.
  m(0, 2) = std::numeric_limits<double>::denorm_min();  // ≈ 4.94066e-324
  EXPECT_NO_THROW(R.set(m));
  EXPECT_FALSE(R.IsExactlyIdentity());

  // Test that setting a RotationMatrix to a 3x3 matrix that is close to a valid
  // RotationMatrix does not throw an exception, whereas setting to a 3x3 matrix
  // that is slightly too-far from a valid RotationMatrix throws an exception.
  m(0, 2) = 127 * kEpsilon;
  EXPECT_NO_THROW(R.set(m));
  m(0, 2) = 129 * kEpsilon;
  if (kDrakeAssertIsArmed) {
    EXPECT_THROW(R.set(m), std::logic_error);
  } else {
    EXPECT_NO_THROW(R.set(m));
  }

  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  R.set(m);
  EXPECT_FALSE(R.IsExactlyIdentity());
}


// Test ProjectMatrixToRotationMatrix.
GTEST_TEST(RotationMatrix, ProjectToRotationMatrix) {
  // Test an identity matrix (valid rotation matrix).
  Matrix3d m = Matrix3d::Identity();
  double quality_factor;
  RotationMatrix<double> R =
      RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  EXPECT_TRUE(R.IsNearlyEqualTo(RotationMatrix<double>(Matrix3d::Identity()),
                                10 * kEpsilon));
  EXPECT_TRUE(std::abs(quality_factor - 1.0) < 40 * kEpsilon);

  // Test another valid rotation matrix.  Ensure near-perfect quality_factor.
  const RollPitchYaw<double> rpy(0.1, 0.2, 0.3);
  m = RotationMatrix<double>(rpy).matrix();
  R = RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  EXPECT_TRUE(
      R.IsNearlyEqualTo(RotationMatrix<double>(m), 10*kEpsilon));
  EXPECT_TRUE(std::abs(quality_factor - 1.0) < 40*kEpsilon);

  // Test scaling each element of a rotation matrix by 2 (linear scaling).
  const Matrix3d m2 = 2 * m;
  R = RotationMatrix<double>::ProjectToRotationMatrix(m2, &quality_factor);
  EXPECT_TRUE(
      R.IsNearlyEqualTo(RotationMatrix<double>(m), 10*kEpsilon));
  EXPECT_TRUE(std::abs(quality_factor - 2.0) < 40*kEpsilon);

  // Test a 3x3 matrix that is far from orthonormal.
  m << 1,   0.1, 0.1,
      -0.2, 1.0, 0.1,
       0.5, 0.6, 0.8;
  EXPECT_FALSE(RotationMatrix<double>::IsValid(m, 64000 * kEpsilon));
  R = RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  EXPECT_TRUE(R.IsValid());
  // Singular values from MotionGenesis [1.405049, 1.061152, 0.4688222]
  EXPECT_TRUE(std::abs(quality_factor - 0.4688222) < 1E-5);

  // Test another 3x3 matrix that is far from orthonormal.
  m << 1, 2,  3,
       4, 5,  6,
       7, 8, -10;
  R = RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  EXPECT_TRUE(R.IsValid());
  // Singular values from MotionGenesis [14.61524, 9.498744, 0.4105846]
  EXPECT_TRUE(std::abs(quality_factor - 14.61524) < 1E-5);

  // Test another 3x3 matrix that is far from orthonormal.
  m << 1E-7, 2, 3,
          4, 5, 6,
          7, 8, -1E6;
  R = RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  EXPECT_TRUE(R.IsValid());
  // Singular values from MotionGenesis [1000000, 6.597777, 1.21254]
  EXPECT_TRUE(std::abs(quality_factor - 1000000) < 1E-1);

  // Test a 3x3 near-zero matrix whose determinant is positive (det = 1E-47).
  m << kEpsilon, 0, 0,
       0, kEpsilon, 0,
       0, 0, kEpsilon;
  EXPECT_TRUE(0 < m.determinant() &&
              m.determinant() < 64 * kEpsilon * kEpsilon * kEpsilon);
  R = RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  EXPECT_TRUE(R.IsValid());
  // Singular values from MotionGenesis [kEpsilon, kEpsilon, kEpsilon]
  EXPECT_TRUE(quality_factor > 0 &&  quality_factor < 64 * kEpsilon);
  EXPECT_TRUE(R.IsNearlyEqualTo(RotationMatrix<double>(Matrix3d::Identity()),
                                64 * kEpsilon));

  // Test a 3x3 near-zero matrix whose determinant is negative (det = -1E-47).
  m << kEpsilon, 0, 0,
      0, kEpsilon, 0,
      0, 0, -kEpsilon;
  EXPECT_TRUE(-64 * kEpsilon * kEpsilon * kEpsilon < m.determinant() &&
               m.determinant() < 0);
  EXPECT_THROW(RotationMatrix<double>::ProjectToRotationMatrix(m,
               &quality_factor), std::logic_error);

  // Test a 3x3 orthogonal matrix but whose determinant is negative (-1).
  m << 1, 0, 0,
       0, 1, 0,
       0, 0, -1;
  EXPECT_TRUE(std::abs(m.determinant() + 1) < 64 * kEpsilon);
  EXPECT_THROW(RotationMatrix<double>::ProjectToRotationMatrix(m,
               &quality_factor), std::logic_error);

  // Check that an exception is thrown if the resulting rotation matrix would
  // have been improper (the resulting rotation matrix would have a determinant
  // of -1 instead of +1).  One way to generate an improper rotation matrix is
  // to try to project a matrix m whose determinant is negative.  For the
  // example below we use a matrix m that also happens to be near-singular,
  // which means that it has a barely negative determinant [det(m) ≈ -3e-13].
  // One can see that matrix m below is near singular by noticing either:
  // row(0) + row(2) ≈ 2 * row(1)  or  col(0) + col(2) ≈ 2 * col(1).
  m << 1, 2, 3,
       4, 5, 6,
       7, 8, 9 + 400 * kEpsilon;
  EXPECT_TRUE(-1600 * kEpsilon < m.determinant() && m.determinant() < 0);
  EXPECT_THROW(RotationMatrix<double>::ProjectToRotationMatrix(m,
               &quality_factor), std::logic_error);

  // Check that no exception is thrown if the resulting rotation matrix is a
  // valid rotation matrix and is proper (meaning the resulting rotation matrix
  // has a determinant of +1, not -1).  As long as a 3x3 matrix m is full rank
  // and has a positive determinant, the test below should succeed.  For the
  // example below, we use a matrix m that happens to be near-singular, which
  // means that it has a barely positive determinant [det(m) ≈ 3e-13].
  // One can see that matrix m below is near singular by noticing either:
  // row(0) + row(2) ≈ 2 * row(1)  or  col(0) + col(2) ≈ 2 * col(1).
  m << 1, 2, 3,
       4, 5, 6,
       7, 8, 9 - 400 * kEpsilon;
  EXPECT_TRUE(0 < m.determinant() && m.determinant() < 1600 * kEpsilon);
  RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  EXPECT_TRUE(quality_factor > 0 && std::abs(quality_factor) < 1600 * kEpsilon);

  // Check that an exception is thrown if the resulting rotation matrix would
  // have been improper (the resulting rotation matrix would have a determinant
  // of -1 instead of +1).  One way to generate an improper rotation matrix is
  // to try to project a matrix m whose determinant is negative [det(m) = -6].
  m << 1, 2, 3,
       4, 5, 6,
      -7, -8, -7;
  EXPECT_LT(m.determinant(), 0);
  EXPECT_THROW(RotationMatrix<double>::ProjectToRotationMatrix(m,
               &quality_factor), std::logic_error);

  // Check that returned rotation matrix is orthonormal.  In other words, its
  // transpose should be equal to its inverse so  that R * Rᵀ = IdentityMatrix.
  m << 1, 2, 3,
       4, 5, 6,
       7, 8, 7;
  EXPECT_GT(m.determinant(), 0);
  R = RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  const RotationMatrix<double> I = R * R.inverse();
  EXPECT_TRUE(I.IsNearlyEqualTo(RotationMatrix<double>(Matrix3d::Identity()),
                                8 * kEpsilon));
  EXPECT_TRUE(R.inverse().IsNearlyEqualTo(R.transpose(), 8 * kEpsilon));
}


// Test RotationMatrix cast method from double to AutoDiffXd.
GTEST_TEST(RotationMatrix, CastFromDoubleToAutoDiffXd) {
  const RollPitchYaw<double> rpy(0.2, 0.3, 0.4);
  const RotationMatrix<double> R_double(rpy);
  const RotationMatrix<AutoDiffXd> R_autodiff = R_double.cast<AutoDiffXd>();

  // To avoid a (perhaps) tautological test, do not just use an Eigen cast() to
  // the Matrix3 that underlies the RotationMatrix class -- i.e., avoid just
  // comparing m_autodiff.cast<double>() with m_double.
  // Instead, check element-by-element equality as follows.
  const Matrix3<double>& m_double = R_double.matrix();
  const Matrix3<AutoDiffXd>& m_autodiff = R_autodiff.matrix();
  for (int i = 0;  i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      const double mij_double = m_double(i, j);
      const AutoDiffXd& mij_autodiff = m_autodiff(i, j);
      EXPECT_EQ(mij_autodiff, mij_double);
      EXPECT_EQ(mij_autodiff.derivatives().size(), 0);
    }
  }
}

// Verify RotationMatrix constructor is compatible with symbolic::Expression,
// including the ThrowIfNotValid() check.
GTEST_TEST(RotationMatrix, SymbolicConstructionTest) {
  using symbolic::Expression;

  // When the underlying scalar type is a symbolic::Expression, ensure
  // set(m_symbolic) only sets the rotation matrix, with no validity checks
  // e.g., ThrowIfNotValid() is a "no-op" (does nothing).
  Matrix3<Expression> m_symbolic;
  m_symbolic << 1, 2, 3,  // This is an obviously invalid rotation matrix.
                4, 5, 6,
                7, 8, 9;
  // Note: The function under test in the next line is ThrowIfNotValid().
  // Since this function is private, it cannot be directly tested.
  // Instead, it is tested via the set() method which calls ThrowIfNotValid()
  // when assertions are armed.
  RotationMatrix<Expression> R;
  EXPECT_NO_THROW(R.set(m_symbolic));

  // Set one of the matrix terms to a variable.  Still no throw.
  const symbolic::Variable x{"x"};
  m_symbolic(0, 0) = x;
  EXPECT_NO_THROW(R.set(m_symbolic));
}

// Verify RotationMatrix projection with symbolic::Expression behaves as
// expected.  (In prior revisions, it was specialized for Expressions.)
// If there are free variables, it will throw.
GTEST_TEST(RotationMatrix, SymbolicProjectionTest) {
  using symbolic::Expression;

  // Set up an identity matrix, but with one off-diagonal free variable.
  Matrix3<Expression> m_symbolic = Matrix3<Expression>::Identity();
  const symbolic::Variable x{"x"};
  m_symbolic(2, 0) = x;

  // Verify Eigen's SVD [which is called by ProjectToRotationMatrix()] throws
  // an exception if it is passed a symbolic matrix with an element that it
  // cannot resolve to a numerical value (e.g., the element contains a free
  // variable).
  // In the future, it would be acceptable if the implementation returned a
  // symbolic result instead of throwing, but for now we'll lock in the "must
  // throw" contract so that we'll notice if the behavior changes.
  using RotMatExpr = RotationMatrix<Expression>;
  Expression quality;
  DRAKE_EXPECT_THROWS_MESSAGE(
      RotMatExpr::ProjectToRotationMatrix(m_symbolic, &quality),
      std::runtime_error,
      ".*environment does not have an entry for the variable.*\n*");

  // Removing the free variable allows us to succeed.
  m_symbolic(2, 0) = 0;   // The input is now the identity matrix.
  RotMatExpr::ProjectToRotationMatrix(m_symbolic, &quality);

  // Sanity check that the operation succeeded.  (We don't specify a tight
  // tolerance here because the precise numerical result is tested elsewhere.)
  EXPECT_LT(abs(quality - 1.0), 1e-3);

  // Verify ProjectToRotationMatrix() (which uses Eigen's SVD) can handle
  // symbolic matrices as long as every element resolves to a numerical value
  // (no free variables).  To more fully test the code, the test matrix below
  // is not already orthonormal since an already-orthonormal matrix may produce
  // an early-return from Eigen's SVD.
  Matrix3d m;
  m << 1, 2,  3,
       4, 5,  6,
       7, 8, -10;
  m_symbolic = m.template cast<Expression>();
  RotMatExpr::ProjectToRotationMatrix(m_symbolic, &quality);
  EXPECT_GT(quality, 10.0);
}

// Utility function to help test ProjectMatToRotMatWithAxis().
// Take many samples of the rotation angle θ, make sure the rotation matrix
// R[θ] = AngleAxis(θ, axis) has larger error than the projected matrix R, so
// (R(i,j) - M(i,j))² <= (R[θ](i,j) - M(i,j))² ∀ θ: angle_lb <= θ <= angle_ub.
void CheckProjectionWithAxis(const Eigen::Matrix3d& M,
                             const Eigen::Vector3d& axis,
                             const double angle_lb,
                             const double angle_ub) {
  const double angle = ProjectMatToRotMatWithAxis(M, axis, angle_lb, angle_ub);
  const RotationMatrixd R(Eigen::AngleAxisd(angle, axis));
  const double R_error = (R.matrix() - M).squaredNorm();
  const int kNumAngles = 100;
  double theta_lb{};
  double theta_ub{};
  // Depending on the value of angle_lb and angle_ub, we choose the range for
  // the sampled theta. If angle_lb and/or angle_ub is inf, then the theta_lb
  // and/or theta_ub will be set to a finite value.
  if (!std::isinf(angle_lb) && !std::isinf(angle_ub)) {
    theta_lb = angle_lb;
    theta_ub = angle_ub;
  } else if (std::isinf(angle_lb) && std::isinf(angle_ub)) {
    theta_lb = -2 * M_PI;
    theta_ub = 2 * M_PI;
  } else if (std::isinf(angle_lb)) {
    theta_lb = angle_ub - 2 * M_PI;
    theta_ub = angle_ub;
  } else {
    theta_lb = angle_lb;
    theta_ub = angle_lb + 2 * M_PI;
  }
  const Eigen::Matrix<double, kNumAngles, 1> theta =
      Eigen::Matrix<double, kNumAngles, 1>::LinSpaced(theta_lb, theta_ub);

  for (int i = 0; i < kNumAngles; ++i) {
    const RotationMatrixd Ri(Eigen::AngleAxisd(theta(i), axis));
    const double Ri_error = (Ri.matrix() - M).squaredNorm();
    EXPECT_GE(Ri_error, R_error - 1E-10);
  }
}

GTEST_TEST(RotationMatrixTest, TestProjectionWithAxis) {
  const Eigen::Vector3d axis(1.0 / 3.0, 2.0 / 3.0, -2.0 / 3.0);
  constexpr double tolerance = 64 * kEpsilon;
  // Note: Before 7/24/2018, tolerance = 1E-6.

  // For a proper rotation matrix with the desired axis, the projected matrix
  // should be the same, if the angle falls inside the bound.
  Eigen::Matrix3d M = Eigen::AngleAxisd(0.2, axis).toRotationMatrix();
  double angle = ProjectMatToRotMatWithAxis(M, axis, 0, 1);
  EXPECT_NEAR(angle, 0.2, tolerance);

  // If the angle of `M` falls outside the angle's bounds, then the optimal
  // projection is either the lower or upper bound (for next test lower bound).
  angle = ProjectMatToRotMatWithAxis(M, axis, 0.3, 1);
  EXPECT_NEAR(angle, 0.3, tolerance);

  // If angle bounds include infinity, the maximal angle is to shift 0.2 by 2kπ.
  constexpr double infinity_dbl = std::numeric_limits<double>::infinity();
  angle = ProjectMatToRotMatWithAxis(M, axis, 0.3, infinity_dbl);
  EXPECT_NEAR(angle, 0.2 + 2 * M_PI, tolerance);

  angle = ProjectMatToRotMatWithAxis(M, axis, -infinity_dbl, 0.1);
  EXPECT_NEAR(angle, 0.2 - 2 * M_PI, tolerance);

  angle = ProjectMatToRotMatWithAxis(M, axis, -infinity_dbl, infinity_dbl);
  EXPECT_NEAR(angle, 0.2, tolerance);

  angle = ProjectMatToRotMatWithAxis(M, axis, -4, 0.1);
  EXPECT_NEAR(angle, 0.1, tolerance);

  M = 2 * Eigen::AngleAxisd(M_PI_2, axis).toRotationMatrix();
  CheckProjectionWithAxis(M, axis, 0.1, 2 * M_PI);
  CheckProjectionWithAxis(M, axis, M_PI, 2 * M_PI);
  CheckProjectionWithAxis(M, axis, -2 * M_PI, -M_PI);

  M = 0.2 * Eigen::AngleAxisd(M_PI / 3, axis).toRotationMatrix();
  CheckProjectionWithAxis(M, axis, 0.1, 2 * M_PI);
  CheckProjectionWithAxis(M, axis, M_PI, 2 * M_PI);
  CheckProjectionWithAxis(M, axis, -2 * M_PI, -M_PI);

  // A random matrix.
  M << 0.1, 0.4, 1.2,
      -0.4, 2.3, 1.5,
      1.3, -.4, -0.2;
  CheckProjectionWithAxis(M, axis, M_PI, 2 * M_PI);
  CheckProjectionWithAxis(M, axis, -2 * M_PI, 0);
  CheckProjectionWithAxis(M, axis, 0.1, 0.2);
  CheckProjectionWithAxis(M, axis, -infinity_dbl, 2 * M_PI);
  CheckProjectionWithAxis(M, axis, -M_PI, infinity_dbl);
  CheckProjectionWithAxis(M, axis, -2 * M_PI, 4 * M_PI);
}

// Tests RotationMatrix R_AB multiplied by a 3 x n matrix whose columns are
// arbitrary vectors, expressed in B.  The result is tested to be a 3 x n matrix
// whose columns are those same vectors but expressed in A.
GTEST_TEST(RotationMatrixTest, OperatorMultiplyByMatrix3X) {
  // Create a somewhat arbitrary RotationMatrix.
  const double r(0.5), p(0.4), y(0.3);
  const RollPitchYaw<double> rpy(r, p, y);
  const RotationMatrix<double> R_AB(rpy);

  // Multiply the RigidTransform R_AB by three vectors to test operator* for a
  // 3 x n matrix, where n = 3 is known before compilation.
  Eigen::Matrix3d v_B;
  const Vector3d v1_B(-12, -9, 7);   v_B.col(0) = v1_B;
  const Vector3d v2_B(-11, -8, 10);  v_B.col(1) = v2_B;
  const Vector3d v3_B(-10, -7, 12);  v_B.col(2) = v3_B;
  const auto v_A = R_AB * v_B;

  // Ensure the compiler's declared type for v_A has the proper number of
  // rows and columns before compilation.  Then verify the results.
  EXPECT_EQ(decltype(v_A)::RowsAtCompileTime, 3);
  EXPECT_EQ(decltype(v_A)::ColsAtCompileTime, 3);

  // Ensure the results for v_A match those from Eigen's matrix multiply.
  // Note: Validating v_A is important because its results are reused below.
  EXPECT_TRUE(CompareMatrices(v_A.col(0), R_AB.matrix() * v1_B, kEpsilon));
  EXPECT_TRUE(CompareMatrices(v_A.col(1), R_AB.matrix() * v2_B, kEpsilon));

  // Multiply the RotationMatrix R_AB by n = 2 vectors to test operator* for a
  // 3 x n matrix, where n is not known before compilation.
  const int number_of_vectors = 2;
  Eigen::Matrix3Xd w_B(3, number_of_vectors);
  w_B.col(0) = v1_B;
  w_B.col(1) = v2_B;
  const auto w_A = R_AB * w_B;

  // Ensure the compiler's declared type for w_A has the proper number of
  // rows before compilation (dictated by the return type of operator*) and
  // has the proper number of columns at run time.
  EXPECT_EQ(decltype(w_A)::RowsAtCompileTime, 3);
  EXPECT_EQ(w_A.cols(), number_of_vectors);
  for (int i = 0; i < number_of_vectors; ++i) {
    const Vector3d wi_A = w_A.col(i);
    const Vector3d wi_A_expected = v_A.col(i);  // Previous result.
    EXPECT_TRUE(CompareMatrices(wi_A, wi_A_expected, kEpsilon));
  }

  // Test RotationMatrix operator* can multiply an Eigen expression, namely the
  // Eigen expression arising from a 3x1 matrix multiplied by a 1x4 matrix.
  const Eigen::MatrixXd s_A = R_AB * (Eigen::Vector3d(1, 2, 3) *
      Eigen::RowVector4d(1, 2, 3, 4));
  EXPECT_EQ(s_A.rows(), 3);
  EXPECT_EQ(s_A.cols(), 4);
  Eigen::Matrix<double, 3, 4> m34_expected;
  m34_expected << 1, 2, 3, 4,
                  2, 4, 6, 8,
                  3, 6, 9, 12;
  EXPECT_TRUE(CompareMatrices(s_A, R_AB.matrix() * m34_expected, kEpsilon));

  // Test RotationMatrix operator* can multiply a different looking Eigen
  // expression that produces the same result.
  const auto z_A_expected = R_AB *
      (Eigen::MatrixXd(3, 4) << Eigen::Vector3d(1, 2, 3),
          Eigen::Vector3d(2, 4, 6),
          Eigen::Vector3d(3, 6, 9),
          Eigen::Vector3d(4, 8, 12)).finished();
  EXPECT_EQ(decltype(z_A_expected)::RowsAtCompileTime, 3);
  EXPECT_EQ(z_A_expected.cols(), 4);
  EXPECT_TRUE(CompareMatrices(s_A, z_A_expected, kEpsilon));

  // Test that operator* disallows weirdly-sized matrix multiplication.
  if (kDrakeAssertIsArmed) {
    Eigen::MatrixXd m_7x8(7, 8);
    m_7x8 = Eigen::MatrixXd::Identity(7, 8);
    Eigen::MatrixXd bad_matrix_multiply;
    EXPECT_THROW(bad_matrix_multiply = R_AB * m_7x8, std::logic_error);
    DRAKE_EXPECT_THROWS_MESSAGE(
        bad_matrix_multiply = R_AB * m_7x8, std::logic_error,
        "Error: Inner dimension for matrix multiplication is not 3.");
  }
}


class RotationMatrixConversionTests : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  void SetUp() override {
    SetupQuaternionTestCases();
  }

  void SetupQuaternionTestCases() {
    // kSweepSize is an even number so that no samples are taken at zero. This
    // test scales as O(N^4) in sweep size, so be cautious about turning it up!
    const int kSweepSize = 6;

    // Set up a variety of general tests for quaternions.
    auto qw = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto qx = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto qy = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto qz = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    for (int i = 0; i < qw.size(); ++i) {
      for (int j = 0; j < qx.size(); ++j) {
        for (int k = 0; k < qy.size(); ++k) {
          for (int l = 0; l < qz.size(); ++l) {
            Eigen::Quaterniond q(qw(i), qx(j), qy(k), qz(l));
            // As long as q.norm() is reasonably far from 0 to avoid divide-by-
            // zero, normalize q so it becomes a valid quaterion (i.e., so that
            // after normalization, q(0)^2 + q(1)^2 + q(2)^2 + q(3)^2 = 1).
            if (q.norm() > 1E-3) {
              q.normalize();
              quaternion_test_cases_.push_back(q);
            }
          }
        }
      }
    }
  }

  std::vector<Eigen::Quaterniond> quaternion_test_cases_;
};

TEST_F(RotationMatrixConversionTests, RotationMatrixToQuaternion) {
  constexpr double tol = 40 * kEpsilon;
  for (const Eigen::Quaterniond& qi : quaternion_test_cases_) {
    // Step 1: Convert the quaternion qi to a 3x3 matrix mi.
    // Step 2: Construct a RotationMatrix Ri from the 3x3 matrix.
    // Step 3: Convert rotation matrix Ri to a quaternion q_expected.
    // Step 4: Ensure qi and q_expected represent the same orientation.
    const Matrix3d mi = qi.toRotationMatrix();
    const RotationMatrix<double> Ri(mi);
    const Eigen::Quaterniond q_actual = Ri.ToQuaternion();
    ASSERT_TRUE(AreQuaternionsEqualForOrientation(qi, q_actual, tol));
  }
}

// Repeat the prior test case with T = Expression without using Variables.
TEST_F(RotationMatrixConversionTests, RotationMatrixToQuaternionSymbolic) {
  using symbolic::Expression;
  constexpr double tol = 40 * kEpsilon;
  for (const Eigen::Quaterniond& qi : quaternion_test_cases_) {
    const Matrix3<Expression> mi = qi.toRotationMatrix();
    const RotationMatrix<Expression> Ri(mi);
    const Eigen::Quaternion<Expression> q_actual_expr = Ri.ToQuaternion();
    const Eigen::Quaterniond q_actual_double(q_actual_expr.coeffs().unaryExpr(
        [](const Expression& x) { return ExtractDoubleOrThrow(x); }));
    ASSERT_TRUE(AreQuaternionsEqualForOrientation(qi, q_actual_double, tol));
  }
}

// Repeat the prior test case with T = Expression and using Variables.
TEST_F(RotationMatrixConversionTests, RotationMatrixToQuaternionVariable) {
  using symbolic::Environment;
  using symbolic::Expression;
  using symbolic::Variable;
  constexpr double tol = 40 * kEpsilon;

  // Perform a fully-symbolic ToQuaternion, where R is only Variables.
  const Matrix3<Variable> m_var =
      symbolic::MakeMatrixContinuousVariable<3, 3>("m");
  const RotationMatrix<Expression> R_expr(m_var);
  const Eigen::Quaternion<Expression> q_expr = R_expr.ToQuaternion();

  // Evaluate the Quaterionion<Expression> for each Ri.
  for (const Eigen::Quaterniond& qi : quaternion_test_cases_) {
    // Prepare the variable substitions.
    const Matrix3d mi = qi.toRotationMatrix();
    Environment env;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        env.insert(m_var(row, col), mi(row, col));
      }
    }
    // Evaluate and compare.
    const Eigen::Quaterniond q_actual_double(q_expr.coeffs().unaryExpr(
        [&env](const Expression& x) { return x.Evaluate(env); }));
    ASSERT_TRUE(AreQuaternionsEqualForOrientation(qi, q_actual_double, tol));
  }
}

TEST_F(RotationMatrixConversionTests, QuaternionToRotationMatrix) {
  for (const Eigen::Quaterniond& qi : quaternion_test_cases_) {
    // Compute the rotation matrix using Eigen's geometry module.
    // Compare that result with the corresponding RotationMatrix constructor.
    const Matrix3d m_expected = qi.toRotationMatrix();
    const RotationMatrix<double> R_expected(m_expected);
    const RotationMatrix<double> R(qi);
    ASSERT_TRUE(R.IsNearlyEqualTo(R_expected, 40 * kEpsilon));
  }

  if (kDrakeAssertIsArmed) {
    // A zero quaternion should throw an exception.
    const Eigen::Quaterniond q_zero(0, 0, 0, 0);
    EXPECT_THROW(const RotationMatrix<double> R_bad(q_zero), std::logic_error);

    // A quaternion containing a NaN throw an exception.
    double nan = std::numeric_limits<double>::quiet_NaN();
    const Eigen::Quaterniond q_nan(nan, 0, 0, 0);
    EXPECT_THROW(const RotationMatrix<double> R_nan(q_nan), std::logic_error);
  }
}

TEST_F(RotationMatrixConversionTests, AngleAxisToRotationMatrix) {
  for (const Eigen::Quaterniond& qi : quaternion_test_cases_) {
    // Compute the rotation matrix R using the quaternion argument.
    const RotationMatrix<double> R(qi);
    // Compare R with the RotationMatrix constructor that uses Eigen::AngleAxis.
    const Eigen::AngleAxisd angle_axis(qi);
    const RotationMatrix<double> R_expected(angle_axis);
    ASSERT_TRUE(R.IsNearlyEqualTo(R_expected, 200 * kEpsilon));

    // Check that inverting this operation (calculating the AngleAxis from
    // rotation matrix R_expected) corresponds to the same orientation.
    // This check is done by comparing equivalent rotation matrices.
    // Note: We do not compare the angle-axes directly. This is because the
    // angle-axis has singularities for angles near 0 and 180 degree.
    Eigen::AngleAxis<double> inverse_angle_axis;
    inverse_angle_axis.fromRotationMatrix(R_expected.matrix());
    const RotationMatrix<double> R_test(inverse_angle_axis);
    ASSERT_TRUE(R.IsNearlyEqualTo(R_test, 200 * kEpsilon));
    // Ensure the angle returned via Eigen's AngleAxis is between 0 and PI.
    const double angle = inverse_angle_axis.angle();
    ASSERT_TRUE(0 <= angle && angle <= M_PI);
  }

  if (kDrakeAssertIsArmed) {
    // An AngleAxis with a zero unit vector should throw an exception.
    const Eigen::AngleAxisd aa_zero(5, Vector3d(0, 0, 0));
    EXPECT_THROW(const RotationMatrix<double> R_zero(aa_zero),
        std::logic_error);

    // An AngleAxis containing a NaN should throw an exception.
    double nan = std::numeric_limits<double>::quiet_NaN();
    const Eigen::AngleAxisd aa_nan(nan, Vector3d(1, 0, 0));
    EXPECT_THROW(const RotationMatrix<double> R_nan(aa_nan), std::logic_error);
  }
}

}  // namespace
}  // namespace math
}  // namespace drake
