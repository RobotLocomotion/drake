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

#ifdef DRAKE_ASSERT_IS_ARMED
  // Really poor non-orthogonal matrix should throw an exception.
  m << 1, 2,  3,
       4, 5,  6,
       7, 8, -10;
  DRAKE_EXPECT_THROWS_MESSAGE(RotationMatrix<double>{m}, std::logic_error,
                              "Error: Rotation matrix is not orthonormal.*")

  // Barely non-orthogonal matrix should throw an exception.
  m << 1, 9000*kEpsilon, 9000*kEpsilon,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  DRAKE_EXPECT_THROWS_MESSAGE(RotationMatrix<double>{m}, std::logic_error,
                              "Error: Rotation matrix is not orthonormal.*");

  // Orthogonal matrix with determinant = -1 should throw an exception.
  m << 1, 0, 0,
       0, 1, 0,
       0, 0, -1;
  DRAKE_EXPECT_THROWS_MESSAGE(RotationMatrix<double>{m}, std::logic_error,
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
  R.set(m);
  Matrix3d zero_matrix = m - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());

  // Bad matrix should throw exception.
  m << 1, 9000*kEpsilon, 9000*kEpsilon,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
#ifdef DRAKE_ASSERT_IS_ARMED
  EXPECT_THROW(R.set(m), std::logic_error);
#else
  EXPECT_NO_THROW(R.set(m));
#endif
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
  double tolerance = 32 * kEpsilon;

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
  EXPECT_TRUE(RA.IsNearlyEqualTo(RB, tolerance).value());

  RA = RotationMatrixd::MakeYRotation(M_PI + theta);
  RB = RotationMatrixd(Eigen::DiagonalMatrix<double, 3>(-1, 1, -1)) *
       RotationMatrixd::MakeYRotation(theta);
  EXPECT_TRUE(RA.IsNearlyEqualTo(RB, tolerance).value());

  RA = RotationMatrixd::MakeZRotation(M_PI + theta);
  RB = RotationMatrixd(Eigen::DiagonalMatrix<double, 3>(-1, -1, 1)) *
       RotationMatrixd::MakeZRotation(theta);
  EXPECT_TRUE(RA.IsNearlyEqualTo(RB, tolerance).value());
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
  EXPECT_TRUE(R_rpy.IsNearlyEqualTo(R_eigen, kEpsilon).value());

  RotationMatrixd R1 = RotationMatrix<double>::MakeZRotation(y);
  RotationMatrixd R2 = RotationMatrix<double>::MakeYRotation(p);
  RotationMatrixd R3 = RotationMatrix<double>::MakeXRotation(r);
  RotationMatrixd R_expected = R1 * R2 * R3;
  EXPECT_TRUE(R_rpy.IsExactlyEqualTo(R_expected).value());
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
  EXPECT_TRUE(RRinv.IsNearlyEqualTo(I, 8 * kEpsilon).value());
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
      R_CA.IsNearlyEqualTo(R_CA_manual_multiply, 10 * kEpsilon).value());

  // Also test IsNearlyEqualTo.
  EXPECT_FALSE(R_CA.IsNearlyEqualTo(R_CB, 10000 * kEpsilon).value());

  // Also test operator*=().
  RotationMatrix<double> R_CA_times_equal_test = R_CB;
  R_CA_times_equal_test *= R_BA;
  EXPECT_TRUE(
      R_CA_times_equal_test.IsNearlyEqualTo(R_CA, 10 * kEpsilon).value());
  EXPECT_FALSE(
      R_CA_times_equal_test.IsNearlyEqualTo(R_CB, 10000 * kEpsilon).value());

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
  EXPECT_TRUE(RotationMatrix<double>::IsOrthonormal(m, 5 * kEpsilon).value());
  EXPECT_TRUE(RotationMatrix<double>::IsValid(m, 5 * kEpsilon).value());

  // Test a matrix that should fail orthonormality check.
  m << 1, 10 * kEpsilon, 10 * kEpsilon,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  EXPECT_GT(m.determinant(), 0);
  EXPECT_FALSE(RotationMatrix<double>::IsOrthonormal(m, 5 * kEpsilon).value());
  EXPECT_FALSE(RotationMatrix<double>::IsValid(m, 5 * kEpsilon).value());

  // Test a matrix that should fail determinant test.
  m << -1, 0, 0,
        0, cos_theta, sin_theta,
        0, -sin_theta, cos_theta;
  EXPECT_LT(m.determinant(), 0);
  EXPECT_TRUE(RotationMatrix<double>::IsOrthonormal(m, 5 * kEpsilon).value());
  EXPECT_FALSE(RotationMatrix<double>::IsValid(m, 5 * kEpsilon).value());
}

// Tests whether or not a RotationMatrix is an identity matrix.
GTEST_TEST(RotationMatrix, IsExactlyIdentity) {
  // Test that the default constructor creates an exact identity matrix.
  RotationMatrix<double> R;
  EXPECT_TRUE(R.IsExactlyIdentity().value());

  // Test that setting R to an identity matrix does not throw an exception.
  Matrix3d m;
  m << 1, 0, 0,
       0, 1, 0,
       0, 0, 1;
  R.set(m);
  EXPECT_TRUE(R.IsExactlyIdentity().value());

  // Test impact of absolute mininimum deviation from identity matrix.
  m(0, 2) = std::numeric_limits<double>::denorm_min();  // ≈ 4.94066e-324
  EXPECT_NO_THROW(R.set(m));
  EXPECT_FALSE(R.IsExactlyIdentity().value());

  // Test that setting a RotationMatrix to a 3x3 matrix that is close to a valid
  // RotationMatrix does not throw an exception, whereas setting to a 3x3 matrix
  // that is slightly too-far from a valid RotationMatrix throws an exception.
  m(0, 2) = 127 * kEpsilon;
  EXPECT_NO_THROW(R.set(m));
  m(0, 2) = 129 * kEpsilon;
#ifdef DRAKE_ASSERT_IS_ARMED
  EXPECT_THROW(R.set(m), std::logic_error);
#else
  EXPECT_NO_THROW(R.set(m));
#endif

  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  R.set(m);
  EXPECT_FALSE(R.IsExactlyIdentity().value());
}


// Test ProjectMatrixToRotationMatrix.
GTEST_TEST(RotationMatrix, ProjectToRotationMatrix) {
  // Test an identity matrix (valid rotation matrix).
  Matrix3d m = Matrix3d::Identity();
  double quality_factor;
  RotationMatrix<double> R =
      RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  EXPECT_TRUE(R.IsNearlyEqualTo(RotationMatrix<double>(Matrix3d::Identity()),
                                10 * kEpsilon).value());
  EXPECT_TRUE(std::abs(quality_factor - 1.0) < 40 * kEpsilon);

  // Test another valid rotation matrix.  Ensure near-perfect quality_factor.
  const RollPitchYaw<double> rpy(0.1, 0.2, 0.3);
  m = RotationMatrix<double>(rpy).matrix();
  R = RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  EXPECT_TRUE(
      R.IsNearlyEqualTo(RotationMatrix<double>(m), 10*kEpsilon).value());
  EXPECT_TRUE(std::abs(quality_factor - 1.0) < 40*kEpsilon);

  // Test scaling each element of a rotation matrix by 2 (linear scaling).
  const Matrix3d m2 = 2 * m;
  R = RotationMatrix<double>::ProjectToRotationMatrix(m2, &quality_factor);
  EXPECT_TRUE(
      R.IsNearlyEqualTo(RotationMatrix<double>(m), 10*kEpsilon).value());
  EXPECT_TRUE(std::abs(quality_factor - 2.0) < 40*kEpsilon);

  // Test a 3x3 matrix that is far from orthonormal.
  m << 1,   0.1, 0.1,
      -0.2, 1.0, 0.1,
       0.5, 0.6, 0.8;
  EXPECT_FALSE(RotationMatrix<double>::IsValid(m, 64000 * kEpsilon).value());
  R = RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  EXPECT_TRUE(R.IsValid().value());
  // Singular values from MotionGenesis [1.405049, 1.061152, 0.4688222]
  EXPECT_TRUE(std::abs(quality_factor - 0.4688222) < 1E-5);

  // Test another 3x3 matrix that is far from orthonormal.
  m << 1, 2,  3,
       4, 5,  6,
       7, 8, -10;
  R = RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  EXPECT_TRUE(R.IsValid().value());
  // Singular values from MotionGenesis [14.61524, 9.498744, 0.4105846]
  EXPECT_TRUE(std::abs(quality_factor - 14.61524) < 1E-5);

  // Test another 3x3 matrix that is far from orthonormal.
  m << 1E-7, 2, 3,
          4, 5, 6,
          7, 8, -1E6;
  R = RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  EXPECT_TRUE(R.IsValid().value());
  // Singular values from MotionGenesis [1000000, 6.597777, 1.21254]
  EXPECT_TRUE(std::abs(quality_factor - 1000000) < 1E-1);

  // Test a 3x3 near-zero matrix whose determinant is positive (det = 1E-47).
  m << kEpsilon, 0, 0,
       0, kEpsilon, 0,
       0, 0, kEpsilon;
  EXPECT_TRUE(0 < m.determinant() &&
              m.determinant() < 64 * kEpsilon * kEpsilon * kEpsilon);
  R = RotationMatrix<double>::ProjectToRotationMatrix(m, &quality_factor);
  EXPECT_TRUE(R.IsValid().value());
  // Singular values from MotionGenesis [kEpsilon, kEpsilon, kEpsilon]
  EXPECT_TRUE(quality_factor > 0 &&  quality_factor < 64 * kEpsilon);
  EXPECT_TRUE(R.IsNearlyEqualTo(RotationMatrix<double>(Matrix3d::Identity()),
                                64 * kEpsilon).value());

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
                                10 * kEpsilon).value());
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
      EXPECT_EQ(mij_autodiff.value(), mij_double);
      EXPECT_EQ(mij_autodiff.derivatives().size(), 0);
    }
  }
}

// Verify RotationMatrix is compatible with symbolic::Expression. This includes,
// construction, and the two methods specialized for symbolic::Expression:
// ThrowIfNotValid() and ProjectToRotationMatrix().
GTEST_TEST(RotationMatrix, SymbolicRotationMatrixSimpleTests) {
  RotationMatrix<symbolic::Expression> R;

  // When the underlying scalar type is a symbolic::Expression, ensure
  // set(m_symbolic) only sets the rotation matrix, with no validity checks
  // e.g., ThrowIfNotValid() is a "no-op" (does nothing).
  Matrix3<symbolic::Expression> m_symbolic;
  m_symbolic << 1, 2, 3,  // This is an obviously invalid rotation matrix.
                4, 5, 6,
                7, 8, 9;
  // Note: The function under test in the next line is ThrowIfNotValid().
  // Since this function is private, it cannot be directly tested.
  // Instead, it is tested via the set() method which calls ThrowIfNotValid()
  // when assertions are armed.
  EXPECT_NO_THROW(R.set(m_symbolic));

  // Test ProjectToRotationMatrix() throws an exception for symbolic expression.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RotationMatrix<symbolic::Expression> R_symbolic_after_project =
      RotationMatrix<symbolic::Expression>::ProjectToRotationMatrix(m_symbolic),
      std::runtime_error,
      "This method is not supported for scalar types "
      "that are not drake::is_numeric<S>.");
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
    EXPECT_TRUE(R.IsNearlyEqualTo(R_expected, 40 * kEpsilon).value());
  }

#ifdef DRAKE_ASSERT_IS_ARMED
  // A zero quaternion should throw an exception.
  const Eigen::Quaterniond q_zero(0, 0, 0, 0);
  EXPECT_THROW(const RotationMatrix<double> R_bad(q_zero), std::logic_error);

  // A quaternion containing a NaN throw an exception.
  double nan = std::numeric_limits<double>::quiet_NaN();
  const Eigen::Quaterniond q_nan(nan, 0, 0, 0);
  EXPECT_THROW(const RotationMatrix<double> R_nan(q_nan), std::logic_error);
#endif
}

TEST_F(RotationMatrixConversionTests, AngleAxisToRotationMatrix) {
  for (const Eigen::Quaterniond& qi : quaternion_test_cases_) {
    // Compute the rotation matrix R using the quaternion argument.
    const RotationMatrix<double> R(qi);
    // Compare R with the RotationMatrix constructor that uses Eigen::AngleAxis.
    const Eigen::AngleAxisd angle_axis(qi);
    const RotationMatrix<double> R_expected(angle_axis);
    EXPECT_TRUE(R.IsNearlyEqualTo(R_expected, 200 * kEpsilon).value());

    // Check that inverting this operation (calculating the AngleAxis from
    // rotation matrix R_expected) corresponds to the same orientation.
    // This check is done by comparing equivalent rotation matrices.
    // Note: We do not compare the angle-axes directly. This is because the
    // angle-axis has singularities for angles near 0 and 180 degree.
    Eigen::AngleAxis<double> inverse_angle_axis;
    inverse_angle_axis.fromRotationMatrix(R_expected.matrix());
    const RotationMatrix<double> R_test(inverse_angle_axis);
    EXPECT_TRUE(R.IsNearlyEqualTo(R_test, 200 * kEpsilon).value());
    // Ensure the angle returned via Eigen's AngleAxis is between 0 and PI.
    const double angle = inverse_angle_axis.angle();
    EXPECT_TRUE(0 <= angle && angle <= M_PI);
  }

#ifdef DRAKE_ASSERT_IS_ARMED
  // An AngleAxis with a zero unit vector should throw an exception.
  const Eigen::AngleAxisd aa_zero(5, Vector3d(0, 0, 0));
  EXPECT_THROW(const RotationMatrix<double> R_zero(aa_zero), std::logic_error);

  // An AngleAxis containing a NaN should throw an exception.
  double nan = std::numeric_limits<double>::quiet_NaN();
  const Eigen::AngleAxisd aa_nan(nan, Vector3d(1, 0, 0));
  EXPECT_THROW(const RotationMatrix<double> R_nan(aa_nan), std::logic_error);
#endif
}

}  // namespace
}  // namespace math
}  // namespace drake
