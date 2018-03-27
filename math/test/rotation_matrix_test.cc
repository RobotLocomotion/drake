#include "drake/math/rotation_matrix.h"

#include <gtest/gtest.h>

#include "drake/math/quaternion.h"

namespace drake {
namespace math {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Helper function to create a rotation matrix associated with a BodyXYZ
// rotation by angles q1 = 0.2 radians, q2 = 0.3 radians, q3 = 0.4 radians.
// Note: These matrices must remain BodyXYZ matrices with the specified angles
// q1, q2, q3, as these matrices are used in conjunction with MotionGenesis
// pre-computed solutions based on these exact matrices.
Matrix3d MakeRotationMatrixBodyXYZ() {
  const double q1 = 0.2, q2 = 0.3, q3 = 0.4;
  const double c1 = std::cos(q1), c2 = std::cos(q2), c3 = std::cos(q3);
  const double s1 = std::sin(q1), s2 = std::sin(q2), s3 = std::sin(q3);
  Matrix3d m;
  m << c2 * c3,
      s3 * c1 + s1 * s2 * c3,
      s1 * s3 - s2 * c1 * c3,
      -s3 * c2,
      c1 * c3 - s1 * s2 * s3,
      s1 * c3 + s2 * s3 * c1,
      s2,
      -s1 * c2,
      c1 * c2;
  return m;
}

// Helper function to create a rotation matrix associated with a BodyXYX
// rotation by angles r1 = 0.5 radians, r2 = 0.5 radians, r3 = 0.7 radians.
// Note: These matrices must remain BodyXYX matrices with the specified angles
// r1, r2, r3, as these matrices are used in conjunction with MotionGenesis
// pre-computed solutions based on these exact matrices.
Matrix3d MakeRotationMatrixBodyXYX() {
  const double r1 = 0.5, r2 = 0.5, r3 = 0.7;
  const double c1 = std::cos(r1), c2 = std::cos(r2), c3 = std::cos(r3);
  const double s1 = std::sin(r1), s2 = std::sin(r2), s3 = std::sin(r3);
  Matrix3d m;
  m << c2,
      s1 * s2,
      -s2 * c1,
      s2 * s3,
      c1 * c3 - s1 * s3 * c2,
      s1 * c3 + s3 * c1 * c2,
      s2 * c3,
      -s3 * c1 - s1 * c2 * c3,
      c1 * c2 * c3 - s1 * s3;
  return m;
}

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

#ifdef DRAKE_ASSERT_IS_ARMED
  // Bad matrix should throw exception.
  m << 1, 9000*kEpsilon, 9000*kEpsilon,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  EXPECT_THROW(RotationMatrix<double> R2(m), std::logic_error);
#endif
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
  R.SetOrThrowIfNotValid(m);
  Matrix3d zero_matrix = m - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());

  // Bad matrix should throw exception.
  m << 1, 9000*kEpsilon, 9000*kEpsilon,
      0, cos_theta, sin_theta,
      0, -sin_theta, cos_theta;
  EXPECT_THROW(R.SetOrThrowIfNotValid(m), std::logic_error);
}

// Test setting a RotationMatrix to an identity matrix.
GTEST_TEST(RotationMatrix, MakeIdentityMatrix) {
  const RotationMatrix<double>& R = RotationMatrix<double>::Identity();
  Matrix3d zero_matrix = Matrix3<double>::Identity() - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test making a rotation matrix associated with a X-rotation.
GTEST_TEST(RotationMatrix, RotationMatrixX) {
  const double theta = 0.3;
  const Matrix3d m = Eigen::AngleAxisd(theta, Vector3d::UnitX()).matrix();
  RotationMatrix<double> R = RotationMatrix<double>::MakeXRotation(theta);
  const Matrix3d zero_matrix = m - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test making a rotation matrix associated with a Y-rotation.
GTEST_TEST(RotationMatrix, RotationMatrixY) {
  const double theta = 0.4;
  const Matrix3d m = Eigen::AngleAxisd(theta, Vector3d::UnitY()).matrix();
  RotationMatrix<double> R = RotationMatrix<double>::MakeYRotation(theta);
  const Matrix3d zero_matrix = m - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test making a rotation matrix associated with a Z-rotation.
GTEST_TEST(RotationMatrix, RotationMatrixZ) {
  const double theta = 0.5;
  const Matrix3d m = Eigen::AngleAxisd(theta, Vector3d::UnitZ()).matrix();
  RotationMatrix<double> R = RotationMatrix<double>::MakeZRotation(theta);
  const Matrix3d zero_matrix = m - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test making a rotation matrix associated with a Body-fixed Z-Y-X rotation.
// or with a Space-fixed X-Y-Z rotation.  Also tests method IsExactlyEqualTo().
GTEST_TEST(RotationMatrix, RotationMatrixBodyZYX) {
  const Vector3d q(0.3, 0.4, 0.5);  // yaw-pitch-roll angles.
  const Matrix3d m = (Eigen::AngleAxisd(q(0), Vector3d::UnitZ())
                    * Eigen::AngleAxisd(q(1), Vector3d::UnitY())
                    * Eigen::AngleAxisd(q(2), Vector3d::UnitX())).matrix();
  const RotationMatrix<double> R_eigen(m);
  const RotationMatrix<double> R_bodyZYX =
      RotationMatrix<double>::MakeBodyZYXRotation(q);
  EXPECT_TRUE(R_bodyZYX.IsNearlyEqualTo(R_eigen, kEpsilon));

  RotationMatrix<double> R1 = RotationMatrix<double>::MakeZRotation(q(0));
  RotationMatrix<double> R2 = RotationMatrix<double>::MakeYRotation(q(1));
  RotationMatrix<double> R3 = RotationMatrix<double>::MakeXRotation(q(2));
  RotationMatrix<double> R_expected = R1 * R2 * R3;
  EXPECT_TRUE(R_bodyZYX.IsExactlyEqualTo(R_expected));

  // Compare to SpaceXYZ rotation sequence.
  const Vector3d roll_pitch_yaw(q(2), q(1), q(0));
  const RotationMatrix<double> R_spaceXYZ =
      RotationMatrix<double>::MakeSpaceXYZRotation(roll_pitch_yaw);
  EXPECT_TRUE(R_spaceXYZ.IsNearlyEqualTo(R_eigen, kEpsilon));
  EXPECT_TRUE(R_spaceXYZ.IsExactlyEqualTo(R_bodyZYX));
}

// Test calculating the inverse of a RotationMatrix.
GTEST_TEST(RotationMatrix, Inverse) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
      0, cos_theta, sin_theta,
      0, -sin_theta, cos_theta;
  RotationMatrix<double> R(m);
  RotationMatrix<double> RRinv = R * R.inverse();
  const RotationMatrix<double>& I = RotationMatrix<double>::Identity();
  EXPECT_TRUE(RRinv.IsNearlyEqualTo(I, 8 * kEpsilon));
}

// Test rotation matrix multiplication and IsNearlyEqualTo.
GTEST_TEST(RotationMatrix, OperatorMultiplyAndIsNearlyEqualTo) {
  Matrix3d m_BA = MakeRotationMatrixBodyXYZ();
  Matrix3d m_CB = MakeRotationMatrixBodyXYX();

  RotationMatrix<double> R_BA(m_BA);
  RotationMatrix<double> R_CB(m_CB);
  RotationMatrix<double> R_CA = R_CB * R_BA;

  // Expected results (from MotionGenesis).
  Matrix3d m_CA;
  m_CA << 0.5623597514496498, 0.6644746169581934, -0.4921635839512615,
          0.3778794976730916, 0.3228981562377939, 0.8677233810014371,
          0.7354988750418453, -0.6739512327435091, -0.06950640758724619;
  RotationMatrix<double> R_CA_expected(m_CA);

  // Also test IsNearlyEqualTo.
  EXPECT_TRUE(R_CA.IsNearlyEqualTo(R_CA_expected, 10 * kEpsilon));

  // Also test operator*=().
  R_CB *= R_BA;
  EXPECT_TRUE(R_CB.IsNearlyEqualTo(R_CA, 10 * kEpsilon));
  EXPECT_FALSE(R_CB.IsNearlyEqualTo(R_BA, 10000 * kEpsilon));

  // Also test operator*() with vectors.
  Vector3d vA(1, 2, 3);     // Vector v expressed in frame A.
  Vector3d vC = R_CA * vA;  // Vector v expressed in frame C.
  Vector3d vC_expected = m_CA * vA;
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
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
      0, cos_theta, sin_theta,
      0, -sin_theta, cos_theta;

  const RotationMatrix<double> R1(m);
  const RotationMatrix<double> R2;
  EXPECT_FALSE(R1.IsExactlyIdentity());
  EXPECT_TRUE(R2.IsExactlyIdentity());
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
  const Vector3d angles(0.1, 0.2, 0.3);
  m = RotationMatrix<double>::MakeSpaceXYZRotation(angles).matrix();
  R = RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  EXPECT_TRUE(R.IsNearlyEqualTo(RotationMatrix<double>(m), 10*kEpsilon));
  EXPECT_TRUE(std::abs(quality_factor - 1.0) < 40*kEpsilon);

  // Test scaling each element of a rotation matrix by 2 (linear scaling).
  const Matrix3d m2 = 2 * m;
  R = RotationMatrix<double>::ProjectToRotationMatrix(m2, &quality_factor);
  EXPECT_TRUE(R.IsNearlyEqualTo(RotationMatrix<double>(m), 10*kEpsilon));
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
                                10 * kEpsilon));
}

// Test RotationMatrix cast method from double to AutoDiffXd.
GTEST_TEST(RotationMatrix, CastFromDoubleToAutoDiffXd) {
  const Matrix3d m = MakeRotationMatrixBodyXYZ();
  const RotationMatrix<double> R_double(m);
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
      EXPECT_EQ(mij_autodiff.value(), mij_double);
      EXPECT_EQ(mij_autodiff.derivatives().size(), 0);
    }
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

TEST_F(RotationMatrixConversionTests, RotationMatrixToQuaternionViceVersa) {
  const double tolerance = 40 * kEpsilon;
  for (const Eigen::Quaterniond& qi : quaternion_test_cases_) {
    // Step 1: Convert the quaternion qi to a 3x3 matrix mi.
    // Step 2: Construct a RotationMatrix Ri from the 3x3 matrix.
    // Step 3: Convert rotation matrix Ri to a quaternion q_expected.
    // Step 4: Ensure qi and q_expected represent the same orientation.
    const Matrix3d mi = qi.toRotationMatrix();
    const RotationMatrix<double> Ri(mi);
    const Eigen::Quaterniond q_expected = Ri.ToQuaternion();
    EXPECT_TRUE(AreQuaternionsEqualForOrientation(qi, q_expected, tolerance));
  }
}

TEST_F(RotationMatrixConversionTests, QuaternionToRotationMatrix) {
  for (const Eigen::Quaterniond& qi : quaternion_test_cases_) {
    // Compute the rotation matrix using Eigen's geometry module.
    // Compare that result with the corresponding RotationMatrix constructor.
    const Matrix3d m_expected = qi.toRotationMatrix();
    const RotationMatrix<double> R_expected(m_expected);
    const RotationMatrix<double> R(qi);
    EXPECT_TRUE(R.IsNearlyEqualTo(R_expected, 40 * kEpsilon));
  }

#ifdef DRAKE_ASSERT_IS_ARMED
  // A zero quaternion should throw an exception.
  const Eigen::Quaterniond q_zero(0, 0, 0, 0);
  EXPECT_THROW(const RotationMatrix<double> R_bad(q_zero), std::logic_error);

  // A NAN quaternion should throw an exception.
  double nan = std::numeric_limits<double>::quiet_NaN();
  const Eigen::Quaterniond q_nan(nan, 0, 0, 0);
  EXPECT_THROW(const RotationMatrix<double> R_nan(q_nan), std::logic_error);
#endif
}

}  // namespace
}  // namespace math
}  // namespace drake
