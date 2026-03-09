#include "drake/multibody/tree/rotational_inertia.h"

#include <limits>
#include <sstream>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace multibody {
namespace math {
namespace {

using drake::math::RotationMatrixd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::NumTraits;
using Eigen::Vector3d;
using std::sort;
using symbolic::Expression;
using symbolic::Variable;

// With assertion disarmed, expect no exception.
#define EXPECT_THROW_IF_ARMED(expression, exception) \
  do {                                               \
    if (kDrakeAssertIsArmed) {                       \
      EXPECT_THROW(expression, exception);           \
    } else {                                         \
      DRAKE_EXPECT_NO_THROW(expression);             \
    }                                                \
  } while (0)

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Test default constructor - all elements should be NaN. Also test IsNaN().
GTEST_TEST(RotationalInertia, DefaultRotationalInertiaConstructorIsNaN) {
  const RotationalInertia<double> default_rotational_inertia;
  Matrix3d inertia_matrix = default_rotational_inertia.CopyToFullMatrix3();
  EXPECT_TRUE(inertia_matrix.array().isNaN().all());
  EXPECT_TRUE(default_rotational_inertia.IsNaN());
  EXPECT_FALSE(default_rotational_inertia.IsZero());
}

GTEST_TEST(RotationalInertia, TestIsZeroAndIsNanFunctions) {
  const RotationalInertia<double> zero_rotational_inertia(0, 0, 0);
  const RotationalInertia<double> non_zero_rotational_inertia(3, 4, 5);
  EXPECT_TRUE(zero_rotational_inertia.IsZero());
  EXPECT_FALSE(non_zero_rotational_inertia.IsZero());
  EXPECT_FALSE(non_zero_rotational_inertia.IsNaN());
  EXPECT_FALSE(zero_rotational_inertia.IsNaN());
}

// Test constructor for a diagonal rotational inertia with all elements equal.
GTEST_TEST(RotationalInertia, DiagonalInertiaConstructor) {
  const double I_diagonal = 3.14;
  RotationalInertia<double> I =
      RotationalInertia<double>::TriaxiallySymmetric(I_diagonal);
  Vector3d moments_expected;
  moments_expected.setConstant(I_diagonal);
  Vector3d products_expected = Vector3d::Zero();
  EXPECT_EQ(I.get_moments(), moments_expected);
  EXPECT_EQ(I.get_products(), products_expected);
}

// Test the factory method for rotational inertia factory that uses moments and
// products of inertia (similar to construction from a 3x3 symmetric matrix).
GTEST_TEST(RotationalInertia, MakeFromMomentsAndProductsOfInertia) {
  // Ensure a zero rotational inertia is valid.
  EXPECT_NO_THROW(
      RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          0, 0, 0, 0, 0, 0, /* skip_validity_check = */ false));

  // Check that an _invalid_ rotational inertia with tiny negative moments of
  // inertia does _not_ throw an exception (ensure test is not too fussy).
  constexpr double kTiny = 8 * kEpsilon;
  EXPECT_NO_THROW(
      RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          -kTiny, 0, -kTiny, 0, 0, 0, /* skip_validity_check = */ false));

  // Ensure an _invalid_ rotational inertia with very small negative moments of
  // inertia throws an exception (ensure test is fussy enough for robotics).
  constexpr double kSmall = 32 * kEpsilon;
  EXPECT_THROW(
      RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          -kSmall, 0, -kSmall, 0, 0, 0, /* skip_validity_check = */ false),
      std::exception);

  // Form an arbitrary (but valid) rotational inertia.
  // Ensure MakeFromMomentsAndProductsOfInertia() and CouldBePhysicallyValid()
  // lead to the same conclusion for at least one _valid_ rotational inertia.
  const double Ixx = 17, Iyy = 13, Izz = 10;
  const double Ixy = -3, Ixz = -3, Iyz = -6;
  RotationalInertia<double> I;
  EXPECT_NO_THROW(
      I = RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          Ixx, Iyy, Izz, Ixy, Ixz, Iyz, /* skip_validity_check = */ false));
  EXPECT_TRUE(I.CouldBePhysicallyValid());

  // Ensure MakeFromMomentsAndProductsOfInertia() and CouldBePhysicallyValid()
  // lead to the same conclusion for at least one _invalid_ rotational inertia.
  RotationalInertia<double> I_bad;
  EXPECT_NO_THROW(
      I_bad = RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          2 * Ixx, Iyy, Izz, Ixy, Ixz, Iyz, /* skip_validity_check = */ true));
  EXPECT_FALSE(I_bad.CouldBePhysicallyValid());

  // Ensure an invalid rotational inertia always throws an exception if the
  // 2nd argument of MakeFromMomentsAndProductsOfInertia() is false or missing.
  EXPECT_THROW(
      RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          2 * Ixx, Iyy, Izz, Ixy, Ixz, Iyz, /* skip_validity_check = */ false),
      std::exception);
  EXPECT_THROW(RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
                   2 * Ixx, Iyy, Izz, Ixy, Ixz, Iyz),
               std::exception);

  // Ensure an invalid rotational inertia does not throw an exception if the
  // 2nd argument of MakeFromMomentsAndProductsOfInertia() is true.
  EXPECT_NO_THROW(
      I = RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          2 * Ixx, Iyy, Izz, Ixy, Ixz, Iyz,
          /* skip_validity_check = */ true));
  EXPECT_FALSE(I.CouldBePhysicallyValid());

  // Check for a thrown exception with proper error message when creating a
  // rotational inertia with a non-finite moment or product of inertia.
  std::string expected_message =
      "[^]*Non-finite moment or product of inertia "
      "detected in RotationalInertia\\.";
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  DRAKE_EXPECT_THROWS_MESSAGE(
      RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          kNaN, Iyy, Izz, /* Ixy = */ 0, /* Ixz = */ 0, /* Iyz = */ 0,
          /* skip_validity_check = */ false),
      expected_message);

  constexpr double kInfinity = std::numeric_limits<double>::infinity();
  DRAKE_EXPECT_THROWS_MESSAGE(
      RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          kInfinity, Iyy, Izz, /* Ixy = */ 0, /* Ixz = */ 0, /* Iyz = */ 0,
          /* skip_validity_check = */ false),
      expected_message);

  // Check for a thrown exception with proper error message when creating an
  // invalid rotational inertia (a principal moment of inertia is negative).
  expected_message =
      "MakeFromMomentsAndProductsOfInertia\\(\\): The rotational inertia\n"
      "\\[ 1  -3  -3\\]\n"
      "\\[-3  13  -6\\]\n"
      "\\[-3  -6  10\\]\n"
      "did not pass the test CouldBePhysicallyValid\\(\\)\\.";
  expected_message += fmt::format(
      "\nThe associated principal moments of inertia:"
      "\n-1.583957883\\d+  7.881702629\\d+  17.702255254\\d+"
      "\nare invalid since at least one is negative.");
  // Note: The principal moments of inertia (with more significant digits)
  // are: -1.583957883490135  7.881702629192435  17.702255254297697.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          1.0, Iyy, Izz, Ixy, Ixz, Iyz, /* skip_validity_check = */ false),
      expected_message);

  // Ensure no exception is thrown for this _invalid_ rotational inertia
  // because it is close enough to valid to pass the triangle inequality test.
  constexpr double Jxx = 2, Jyy = 4, Jzz = 6;
  const double trace = Jxx + Jyy + Jzz;
  double extra = 8 * kEpsilon * trace / 2.0;
  DRAKE_EXPECT_NO_THROW(
      RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          Jxx, Jyy, Jzz + extra, /* Jxy = */ 0, /* Jxz = */ 0, /* Jyz = */ 0,
          /* skip_validity_check = */ false));

  // Check for a thrown exception with proper error message when creating a
  // simple rotational inertia that violates the triangle inequality.
  extra = 64 * kEpsilon * trace / 2.0;
  expected_message =
      "MakeFromMomentsAndProductsOfInertia\\(\\): The rotational inertia\n"
      "\\[                2                  0                  0\\]\n"
      "\\[                0                  4                  0\\]\n"
      "\\[                0                  0  6.000000000000085\\]\n"
      "did not pass the test CouldBePhysicallyValid\\(\\)\\.";
  // TODO(Mitiguy) It is unnecessary (and confusing) to append information about
  //  associated principal moments of inertia when the moments of inertia for
  //  the original matrix do not pass the triangle-inequality test.
  expected_message +=
      fmt::format("\nThe associated principal moments of inertia:[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          Jxx, Jyy, Jzz + extra, /* Jxy = */ 0, /* Jxz = */ 0, /* Jyz = */ 0,
          /* skip_validity_check = */ false),
      expected_message);

  // Check for a thrown exception with proper error message when creating a
  // rotational inertia that violates the triangle inequality.
  expected_message =
      "MakeFromMomentsAndProductsOfInertia\\(\\): The rotational inertia\n"
      "\\[34  -3  -3\\]\n"
      "\\[-3  13  -6\\]\n"
      "\\[-3  -6  10\\]\n"
      "did not pass the test CouldBePhysicallyValid\\(\\)\\.";
  expected_message += fmt::format(
      "\nThe associated principal moments of inertia:"
      "\n4.70955263953\\d+  17.66953281\\d+  34.6209145475\\d+"
      "\ndo not satisfy the triangle inequality.");
  // Note: The principal moments of inertia (with more significant digits)
  // are:  4.709552639531104  17.66953281292159  34.620914547547315.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          2 * Ixx, Iyy, Izz, Ixy, Ixz, Iyz, /* skip_validity_check = */ false),
      expected_message);
}

// Test constructor for a principal axes rotational inertia matrix (products
// of inertia are zero).
GTEST_TEST(RotationalInertia, PrincipalAxesConstructor) {
  const Vector3d moments(2.0, 2.3, 2.4);
  RotationalInertia<double> I(moments(0), moments(1), moments(2));
  EXPECT_EQ(I.get_moments(), moments);
  EXPECT_EQ(I.get_products(), Vector3d::Zero());
}

// Test constructor for a general rotational inertia matrix with non-zero
// off-diagonal elements for which the six entries need to be specified.
// Also test SetZero() and SetNaN() methods.
GTEST_TEST(RotationalInertia, GeneralConstructor) {
  const Vector3d moments(2.0, 2.3, 2.4);
  const Vector3d product(0.1, -0.1, 0.2);
  RotationalInertia<double> I(moments(0), moments(1), moments(2), product(0),
                              product(1), product(2));
  EXPECT_EQ(I.get_moments(), moments);
  EXPECT_EQ(I.get_products(), product);

  // Test SetZero().
  I.SetZero();
  EXPECT_TRUE((I.CopyToFullMatrix3().array() == 0).all());

  // Test SetToNaN().
  I.SetToNaN();
  EXPECT_TRUE(I.CopyToFullMatrix3().array().isNaN().all());
}

// Test calculation of trace of rotational inertia (sum of diagonal elements).
GTEST_TEST(RotationalInertia, TraceIsSumOfDiagonalElements) {
  const Vector3d moments(2.0, 2.3, 2.4);
  const Vector3d product(0.1, -0.1, 0.2);
  const RotationalInertia<double> I(moments(0), moments(1), moments(2),
                                    product(0), product(1), product(2));
  const double trace = moments(0) + moments(1) + moments(2);
  EXPECT_EQ(trace, I.Trace());
}

// Test access by (i, j) indexes.
GTEST_TEST(RotationalInertia, AccessByIndexes) {
  const Vector3d m(2.0, 2.3, 2.4);                     // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);                    // p for products.
  const RotationalInertia<double> I(m(0), m(1), m(2),  /* moments of inertia */
                                    p(0), p(1), p(2)); /* products of inertia */

  // Diagonal elements.
  EXPECT_EQ(I(0, 0), m(0));
  EXPECT_EQ(I(1, 1), m(1));
  EXPECT_EQ(I(2, 2), m(2));

  // Off diagonal elements.
  EXPECT_EQ(I(0, 1), p(0));
  EXPECT_EQ(I(0, 2), p(1));
  EXPECT_EQ(I(1, 2), p(2));

  // And their symmetric counterparts.
  EXPECT_EQ(I(1, 0), p(0));
  EXPECT_EQ(I(2, 0), p(1));
  EXPECT_EQ(I(2, 1), p(2));
}

// Tests that even when the underlying dense Eigen representation holds NaN
// entries for unused entries, RotationalInertia behaves as a symmetric matrix.
GTEST_TEST(RotationalInertia, Symmetry) {
  const Vector3d m(2.0, 2.3, 2.4);                     // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);                    // p for products.
  const RotationalInertia<double> I(m(0), m(1), m(2),  /* moments of inertia */
                                    p(0), p(1), p(2)); /* products of inertia */

  // Tests that the copy to a full Matrix3 object is well defined and
  // leads to a symmetric matrix.
  const Matrix3d Imatrix = I.CopyToFullMatrix3();
  EXPECT_FALSE(Imatrix.array().isNaN().any());  // no entry is NaN.
  EXPECT_EQ(Imatrix(0, 0), m(0));
  EXPECT_EQ(Imatrix(1, 1), m(1));
  EXPECT_EQ(Imatrix(2, 2), m(2));
  EXPECT_EQ(Imatrix(0, 1), p(0));
  EXPECT_EQ(Imatrix(0, 2), p(1));
  EXPECT_EQ(Imatrix(1, 2), p(2));
  EXPECT_EQ(Imatrix(1, 0), p(0));
  EXPECT_EQ(Imatrix(2, 0), p(1));
  EXPECT_EQ(Imatrix(2, 1), p(2));
}

// TestB: Rotational inertia expressed in frame R then re-expressed in frame E.
GTEST_TEST(RotationalInertia, IsNearlyEqualTo) {
  // Rotate rigid-body B from frame A by a Euler body-fixed sequence
  // characterized by BodyXYZ q1, q2, q3.  Calculations from MotionGenesis.

  // Form an arbitrary (but valid) rotational inertia for B.
  const double Ixx = 17.36933842091061;
  const double Iyy = 13.60270381717411;
  const double Izz = 10.02795776191528;
  const double Ixy = -3.084298624204901;
  const double Ixz = -3.634144189476002;
  const double Iyz = -6.539290790868233;
  const double trace = Ixx + Iyy + Izz;
  const double epsilonI = kEpsilon * trace / 2;
  const RotationalInertia<double> I1(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
  const RotationalInertia<double> I2(Ixx, Iyy, Izz, Ixy, Ixz,
                                     Iyz + 3 * epsilonI);
  const RotationalInertia<double> I3(Ixx, Iyy, Izz, Ixy, Ixz,
                                     Iyz + 8 * epsilonI);

  // Ensure rotational inertias I1 and I2 are nearly equal.
  // Ensure rotational inertias I1 and I3 are not equal.
  EXPECT_TRUE(I1.IsNearlyEqualTo(I2, 4 * kEpsilon));
  EXPECT_FALSE(I1.IsNearlyEqualTo(I3, 7 * kEpsilon));
}

// TestA: Rotational inertia expressed in frame R then re-expressed in frame E.
GTEST_TEST(RotationalInertia, ReExpressInAnotherFrameA) {
  // Rod R has its frame origin Ro located at the rod's center of mass and its
  // frame's z-axis is the rod's axial direction (i.e., the rod symmetry axis).
  const double mass = 1.0;
  const double radius = 0.1;
  const double length = 1.0;

  // Moment of inertia about rod symmetry axis (z-axis) and perpendicular to it.
  const double I_axial = mass * radius * radius / 2.0;
  const double I_perp = mass * length * length / 12.0;

  // I_RRo_R is rod R's rotational inertia about-point Ro, expressed-in frame R.
  const RotationalInertia<double> I_RRo_R(I_perp, I_perp, I_axial);

  // Rotation of +90 degrees about x (F's y-axis is aligned with R's z-axis).
  const RotationMatrixd R_FR = RotationMatrixd::MakeXRotation(M_PI_2);

  // Re-express in frame F using the above rotation.
  const RotationalInertia<double> I_RRo_F = I_RRo_R.ReExpress(R_FR);

  // Verify I_RRo_F(1,1) (rod R's moment of inertia about-point Ro for y-axis of
  // expressed-in frame F) is equal to I_RRo_R(2,2) (rod R's moment of inertia
  // about-point Ro for z-axis of expressed-in frame R).
  EXPECT_NEAR(I_RRo_F(0, 0), I_RRo_R(0, 0), kEpsilon);  // F x-axis = R +x-axis.
  EXPECT_NEAR(I_RRo_F(1, 1), I_RRo_R(2, 2), kEpsilon);  // F y-axis = R +z-axis.
  EXPECT_NEAR(I_RRo_F(2, 2), I_RRo_R(1, 1), kEpsilon);  // F z-axis = R -y-axis.

  // Ensure re-expressing in frame F still produces a physically valid inertia.
  EXPECT_TRUE(I_RRo_F.CouldBePhysicallyValid());
}

// TestB: Rotational inertia expressed in frame R then re-expressed in frame E.
GTEST_TEST(RotationalInertia, ReExpressInAnotherFrameB) {
  // Rotate rigid-body B from frame A by a Euler body-fixed sequence
  // characterized by BodyXYZ q1, q2, q3.  Calculations from MotionGenesis.
  const double deg_to_rad = M_PI / 180;
  const double q1 = 30 * deg_to_rad, q2 = 70 * deg_to_rad,
               q3 = -70 * deg_to_rad;
  const double R_BAxx = cos(q2) * cos(q3);
  const double R_BAxy = sin(q3) * cos(q1) + sin(q1) * sin(q2) * cos(q3);
  const double R_BAxz = sin(q1) * sin(q3) - sin(q2) * cos(q1) * cos(q3);
  const double R_BAyx = -sin(q3) * cos(q2);
  const double R_BAyy = cos(q1) * cos(q3) - sin(q1) * sin(q2) * sin(q3);
  const double R_BAyz = sin(q1) * cos(q3) + sin(q2) * sin(q3) * cos(q1);
  const double R_BAzx = sin(q2);
  const double R_BAzy = -sin(q1) * cos(q2);
  const double R_BAzz = cos(q1) * cos(q2);
  const RotationMatrixd R_BA = RotationMatrixd::MakeFromOrthonormalRows(
      Vector3d(R_BAxx, R_BAxy, R_BAxz), Vector3d(R_BAyx, R_BAyy, R_BAyz),
      Vector3d(R_BAzx, R_BAzy, R_BAzz));

  // Form an arbitrary (but valid) rotational inertia for B about-point Bo,
  // expressed-in frame A.  These results are from MotionGenesis and arise by
  // starting with I_BBcm_B = [3, 0, 0;  0, 4, 0;  0, 0, 5] and then shifting
  // to point Bo via position vector [1, 1.5, 2], then expressing in A.
  const double I_BBo_Axx = 17.36933842091061;
  const double I_BBo_Ayy = 13.60270381717411;
  const double I_BBo_Azz = 10.02795776191528;
  const double I_BBo_Axy = -3.084298624204901;
  const double I_BBo_Axz = -3.634144189476002;
  const double I_BBo_Ayz = -6.539290790868233;
  const RotationalInertia<double> I_BBo_A(I_BBo_Axx, I_BBo_Ayy, I_BBo_Azz,
                                          I_BBo_Axy, I_BBo_Axz, I_BBo_Ayz);

  // Ensure rotational inertia I_BBo_A is physically valid.
  EXPECT_TRUE(I_BBo_A.CouldBePhysicallyValid());

  // Re-express I_BBo_A from expressed-in frame A to expressed-in frame B.
  const RotationalInertia<double> I_BBo_B = I_BBo_A.ReExpress(R_BA);

  // Form rotational inertia I_BBo_B with MotionGenesis.
  const double I_BBo_Bxx = 6.369894432933752;
  const double I_BBo_Byy = 18.38428249710324;
  const double I_BBo_Bzz = 16.24582306996301;
  const double I_BBo_Bxy = 1.134877977228511;
  const double I_BBo_Bxz = 6.018249975302059;
  const double I_BBo_Byz = -0.6136491084717202;
  const RotationalInertia<double> expected_I_BBo_B(
      I_BBo_Bxx, I_BBo_Byy, I_BBo_Bzz, I_BBo_Bxy, I_BBo_Bxz, I_BBo_Byz);

  // Compare Drake results versus MotionGenesis results using a comparison
  // that tests moments/products of inertia to within kEpsilon multiplied
  // by trace / 2, where trace is the smallest trace of the two matrices.
  EXPECT_TRUE(I_BBo_B.IsNearlyEqualTo(expected_I_BBo_B, kEpsilon));
}

// Test the method ShiftFromCenterOfMass for a body B's rotational inertia
// about-point Bcm (B's center of mass) to about-point Q.
GTEST_TEST(RotationalInertia, ShiftFromCenterOfMass) {
  const double I_BBcm_Bxx = 2, I_BBcm_Byy = 3, I_BBcm_Bzz = 4;
  const double I_BBcm_Bxy = 0, I_BBcm_Bxz = 0, I_BBcm_Byz = 0;
  const RotationalInertia<double> I_BBcm_B(I_BBcm_Bxx, I_BBcm_Byy, I_BBcm_Bzz,
                                           I_BBcm_Bxy, I_BBcm_Bxz, I_BBcm_Byz);
  const double mass = 1.1, xQ = 2.234, yQ = 3.14, zQ = 0.56;
  const Vector3d p_BcmQ_B(xQ, yQ, zQ);
  const RotationalInertia<double> I_BQ_B =
      I_BBcm_B.ShiftFromCenterOfMass(mass, p_BcmQ_B);

  // Compare Drake results versus by-hand results.
  const double Ixx = I_BBcm_Bxx + mass * (yQ * yQ + zQ * zQ);
  const double Iyy = I_BBcm_Byy + mass * (xQ * xQ + zQ * zQ);
  const double Izz = I_BBcm_Bzz + mass * (xQ * xQ + yQ * yQ);
  const double Ixy = I_BBcm_Bxy - mass * xQ * yQ;
  const double Ixz = I_BBcm_Bxz - mass * xQ * zQ;
  const double Iyz = I_BBcm_Byz - mass * yQ * zQ;
  const RotationalInertia<double> expected_I_BQ_B(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
  EXPECT_TRUE(I_BQ_B.IsNearlyEqualTo(expected_I_BQ_B, 2 * kEpsilon));
}

// Test the method ShiftToCenterOfMass for a body B's rotational inertia
// about-point P to about-point Bcm (B's center of mass).
GTEST_TEST(RotationalInertia, ShiftToCenterOfMass) {
  const double I_BP_Bxx = 13.2, I_BP_Byy = 8.8, I_BP_Bzz = 20.3;
  const double I_BP_Bxy = -7.7, I_BP_Bxz = -1.3, I_BP_Byz = -1.9;
  const RotationalInertia<double> I_BP_B(I_BP_Bxx, I_BP_Byy, I_BP_Bzz, I_BP_Bxy,
                                         I_BP_Bxz, I_BP_Byz);
  const double mass = 1.1, xBcm = 2.234, yBcm = 3.14, zBcm = 0.56;
  const Vector3d p_PBcm_B(xBcm, yBcm, zBcm);
  const RotationalInertia<double> I_BBcm_B =
      I_BP_B.ShiftToCenterOfMass(mass, p_PBcm_B);

  // Compare Drake results versus by-hand results.
  const double Ixx = I_BP_Bxx - mass * (yBcm * yBcm + zBcm * zBcm);
  const double Iyy = I_BP_Byy - mass * (xBcm * xBcm + zBcm * zBcm);
  const double Izz = I_BP_Bzz - mass * (xBcm * xBcm + yBcm * yBcm);
  const double Ixy = I_BP_Bxy + mass * xBcm * yBcm;
  const double Ixz = I_BP_Bxz + mass * xBcm * zBcm;
  const double Iyz = I_BP_Byz + mass * yBcm * zBcm;
  const RotationalInertia<double> expected_I_BBcm_B(Ixx, Iyy, Izz, Ixy, Ixz,
                                                    Iyz);
  EXPECT_TRUE(I_BBcm_B.IsNearlyEqualTo(expected_I_BBcm_B, 4 * kEpsilon));
}

// Test the method ShiftToThenAwayFromCenterOfMass for a body B's
// rotational inertia from about-point P to about-point Q.
GTEST_TEST(RotationalInertia, ShiftToThenAwayFromCenterOfMass) {
  const double I_BP_xx = 13.2, I_BP_yy = 8.8, I_BP_zz = 20.3;
  const double I_BP_xy = -7.7, I_BP_xz = -1.3, I_BP_yz = -1.9;
  const RotationalInertia<double> I_BP_B(I_BP_xx, I_BP_yy, I_BP_zz, I_BP_xy,
                                         I_BP_xz, I_BP_yz);
  const double mass = 1.1, xBcm = 2.234, yBcm = 3.14, zBcm = 0.56;
  const double xP = -1.234, yP = -2.11, zP = 1.98;
  const Vector3d p_PBcm(xBcm, yBcm, zBcm);
  const Vector3d p_QP(xP, yP, zP);
  const Vector3d p_QBcm = p_QP + p_PBcm;

  // Calculate with two shifts, each involving Bcm (B's center of mass).
  // Shift 1 is from P to Bcm.
  // Shift 2 is from Bcm to Q.
  const RotationalInertia<double> I_BBcm_B =
      I_BP_B.ShiftToCenterOfMass(mass, p_PBcm);
  const RotationalInertia<double> expected_I_BQ_B =
      I_BBcm_B.ShiftFromCenterOfMass(mass, p_QBcm);

  // Calculate with single method that does it slightly more efficiently.
  const RotationalInertia<double> I_BQ_B =
      I_BP_B.ShiftToThenAwayFromCenterOfMass(mass, p_PBcm, p_QBcm);
  EXPECT_TRUE(I_BQ_B.IsNearlyEqualTo(expected_I_BQ_B, 2 * kEpsilon));

  // Test that negating position vectors have no affect on results.
  EXPECT_TRUE(I_BBcm_B.IsNearlyEqualTo(
      I_BP_B.ShiftToCenterOfMass(mass, -p_PBcm), 2 * kEpsilon));
  EXPECT_TRUE(I_BQ_B.IsNearlyEqualTo(
      I_BBcm_B.ShiftFromCenterOfMass(mass, -p_QBcm), 2 * kEpsilon));
  EXPECT_TRUE(I_BQ_B.IsNearlyEqualTo(
      I_BP_B.ShiftToThenAwayFromCenterOfMass(mass, -p_PBcm, -p_QBcm),
      2 * kEpsilon));
}

// Test the method CouldBePhysicallyValid after a body B's rotational inertia
// is shifted from about-point P to about-point Bcm (B's center of mass).
GTEST_TEST(RotationalInertia, CouldBePhysicallyValidA) {
  const double I_BP_xx = 2, I_BP_yy = 3, I_BP_zz = 4;
  const double I_BP_xy = 0, I_BP_xz = 0, I_BP_yz = 0;
  const RotationalInertia<double> I_BP(I_BP_xx, I_BP_yy, I_BP_zz, I_BP_xy,
                                       I_BP_xz, I_BP_yz);
  const double mass = 1.1, xBcm = 2.234, yBcm = 3.14, zBcm = 0.56;
  const Vector3d p_PBcm(xBcm, yBcm, zBcm);

  // Shifting to calculate I_BBcm should throw an exception in debug builds
  // because some of the moments of inertia are negative.
  RotationalInertia<double> I_BBcm;
  EXPECT_THROW_IF_ARMED(I_BBcm = I_BP.ShiftToCenterOfMass(mass, p_PBcm),
                        std::logic_error);
}

// Test the method CouldBePhysicallyValid for a rod-like object.
GTEST_TEST(RotationalInertia, CouldBePhysicallyValidB) {
  const double I_transverse = 20;
  const double I_axial = -1.0E-15;  // Although negative, this is effectively 0.
  const RotationalInertia<double> rod(I_transverse, I_transverse, I_axial);
  EXPECT_TRUE(rod.CouldBePhysicallyValid());

  const RotationalInertia<double> sphere(1.0E-5, 1.0E-5, 1.0E-5);
  EXPECT_TRUE(sphere.CouldBePhysicallyValid());

  // Subtracting the sphere from the rod creates an invalid rotational inertia.
  EXPECT_THROW_IF_ARMED(rod - sphere, std::logic_error);
}

// Test the method CouldBePhysicallyValid to violate triangle inequality.
GTEST_TEST(RotationalInertia, CouldBePhysicallyValidC) {
  EXPECT_THROW_IF_ARMED(RotationalInertia<double> bad_inertia(10, 10, 30),
                        std::logic_error);
}

// Test the method CouldBePhysicallyValid for bad product of inertia
// sizing relative to moments of inertia.
GTEST_TEST(RotationalInertia, CouldBePhysicallyValidD) {
  EXPECT_THROW_IF_ARMED(RotationalInertia<double> bad_I(4, 4, 4, 2.1, 0, 0),
                        std::logic_error);
}

// Test the method CouldBePhysicallyValid for principal moments of inertia
// (i.e., eigenvalues of the inertia matrix) that violate triangle inequality.
// This test is courtesy of Steve Peters via Michael Sherman.
GTEST_TEST(RotationalInertia, CouldBePhysicallyValidE) {
  EXPECT_THROW_IF_ARMED(
      RotationalInertia<double> bad_inertia(2, 2, 2, -0.8, 0, -0.8),
      std::logic_error);
}

// Test the method RotationalInertia::CalcPrincipalMomentsOfInertia() that
// computes a rotational inertia's principal moments of inertia via eigenvalues.
// Also test CalcPrincipalMomentsAndAxesOfInertia().
GTEST_TEST(RotationalInertia, PrincipalMomentsOfInertiaEtc) {
  const double mass = 1.0;
  const double Lx = 3.0;
  const double Ly = 1.0;
  const double Lz = 5.0;

  // Rotational inertia of a box B computed about Bcm (B's center of mass).
  const double Imed = mass * (Ly * Ly + Lz * Lz) / 12.0;  // Ixx is medium.
  const double Imax = mass * (Lx * Lx + Lz * Lz) / 12.0;  // Iyy is largest.
  const double Imin = mass * (Lx * Lx + Ly * Ly) / 12.0;  // Izz is smallest.
  double Ixx = Imed, Iyy = Imax, Izz = Imin;
  RotationalInertia<double> I_BBcm_Q(Ixx, Iyy, Izz);

  // Orient a frame Q relative to a frame W by subjecting frame Q to a SpaceXYZ
  // rotation sequence of 20, 25, 30 degrees (i.e., BodyZYX by 30, 25, 20).
  const double deg_to_rad = M_PI / 180.0;
  const drake::math::RollPitchYaw<double> rpy(20 * deg_to_rad, 25 * deg_to_rad,
                                              30 * deg_to_rad);
  const drake::math::RotationMatrix<double> R_WQ(rpy);

  // Compute B's rotational inertia about-point Bcm, expressed-in frame W.
  // This rotational inertia has all non-zero entries (not diagonal).
  RotationalInertia<double> I_BBcm_W = I_BBcm_Q.ReExpress(R_WQ);

  // Verify I_BBcm_W contains relatively "large" non-zero diagonal elements.
  EXPECT_TRUE((I_BBcm_W.CopyToFullMatrix3().array().abs() > 0.1).all());

  // Compute the principal moments of I_BBcm_W.
  Vector3d principal_moments = I_BBcm_W.CalcPrincipalMomentsOfInertia();

  // The expected moments are those originally computed in I_BBcm_Q, though the
  // return from RotationalInertia::CalcPrincipalMomentsOfInertia() is sorted
  // in ascending order. Therefore reorder before performing the comparison.
  Vector3d expected_principal_moments = I_BBcm_Q.get_moments();
  std::sort(
      expected_principal_moments.data(),
      expected_principal_moments.data() + expected_principal_moments.size());

  // Verify principal moments against their expected value.
  const double inertia_tolerance =
      4 * std::numeric_limits<double>::epsilon() *
      I_BBcm_W.CalcMaximumPossibleMomentOfInertia();
  EXPECT_TRUE(CompareMatrices(expected_principal_moments, principal_moments,
                              inertia_tolerance, MatrixCompareType::absolute));

  // Reform I_BBcm_Q by re-expressing, i.e., I_BBcm_Q = R_QW * I_BBcm_W * R_WQ,
  // Due to round-off, it is not expected that I_BBcm_Q is perfectly reformed.
  I_BBcm_Q = I_BBcm_W.ReExpress(R_WQ.inverse());

  // Verify products of inertia (due to express and reexpress) are nearly zero.
  // Note: In CI machines June 2023, products of inertia != 0 due to round-off.
  const Vector3d products_of_inertia = I_BBcm_Q.get_products();
  EXPECT_TRUE(CompareMatrices(products_of_inertia, Vector3<double>::Zero(),
                              inertia_tolerance, MatrixCompareType::absolute));

  // Show principal moments of inertia are ≈ unchanged to multiple rotations.
  // Note: It is reasonable to expect a slight difference due to round-off.
  std::pair<Vector3<double>, drake::math::RotationMatrix<double>> I_BBcm_P =
      I_BBcm_Q.CalcPrincipalMomentsAndAxesOfInertia();
  principal_moments = I_BBcm_P.first;
  EXPECT_TRUE(CompareMatrices(expected_principal_moments, principal_moments,
                              inertia_tolerance, MatrixCompareType::absolute));

  // Show rotation matrix R_QP relating principal frame P to frame Q is exactly
  // the rotation matrix whose columns are determined by the smallest,
  // intermediate, and largest moments of inertia (commented above).
  // Note: Before July 2023, CalcPrincipalMomentsAndAxesOfInertia() did not use
  // a "tolerance", so the general eigenvalue/eigenvector routine was called if
  // any product of inertia ≠ 0. Hence, the test below failed before July 2023
  // due to tiny numbers instead of zeroes and reordering of columns.
  drake::math::RotationMatrix<double> R_QP = I_BBcm_P.second;
  Matrix3<double> m_expected;
  // clang-format off
  m_expected << 0, 1, 0,  // Izz is smallest moment of inertia (1st column).
                0, 0, 1,  // Ixx is intermediate moment of inertia (2nd column).
                1, 0, 0;  // Iyy is largest moment of inertia (3rd column).
  // clang-format on
  EXPECT_TRUE(CompareMatrices(R_QP.matrix(), m_expected, 0.0,
                              MatrixCompareType::absolute));

  // Create a test that makes a rotation matrix whose 3rd column is [0 0 -1].
  Ixx = Imed, Iyy = Imin, Izz = Imax;
  I_BBcm_Q = RotationalInertia<double>(Ixx, Iyy, Izz);
  I_BBcm_W = I_BBcm_Q.ReExpress(R_WQ);
  I_BBcm_Q = I_BBcm_W.ReExpress(R_WQ.inverse());
  I_BBcm_P = I_BBcm_Q.CalcPrincipalMomentsAndAxesOfInertia();
  principal_moments = I_BBcm_P.first;
  EXPECT_TRUE(CompareMatrices(expected_principal_moments, principal_moments,
                              inertia_tolerance, MatrixCompareType::absolute));
  R_QP = I_BBcm_P.second;
  // clang-format off
  m_expected << 0, 1, 0,   // Iyy is smallest moment of inertia (1st column).
                1, 0, 0,   // Ixx is intermediate moment of inertia (2nd column)
                0, 0, -1;  // Izz is largest moment of inertia (3rd column).
  // clang-format on
  EXPECT_TRUE(CompareMatrices(R_QP.matrix(), m_expected, 0.0,
                              MatrixCompareType::absolute));

  // Create a test that ideally makes an identity rotation matrix.
  Ixx = Imax + 0.5 * inertia_tolerance;  // Intermediate moment of inertia.
  Iyy = Imax;                            // Minimum moment of inertia.
  Izz = Imax + inertia_tolerance;        // Maximum moment of inertia.
  I_BBcm_W = RotationalInertia<double>(Ixx, Iyy, Izz);
  I_BBcm_Q = I_BBcm_W.ReExpress(R_WQ.inverse());
  I_BBcm_P = I_BBcm_Q.CalcPrincipalMomentsAndAxesOfInertia();
  principal_moments = I_BBcm_P.first;
  EXPECT_TRUE(CompareMatrices(Vector3<double>(Iyy, Ixx, Izz), principal_moments,
                              inertia_tolerance, MatrixCompareType::absolute));
  R_QP = I_BBcm_P.second;
  // clang-format off
  m_expected << 1, 0, 0,  // Since Ixx ≈ Iyy ≈ Izz (triaxially symmetric), it is
                0, 1, 0,  // ideal ("canonical") if the rotation matrix for
                0, 0, 1;  // principal axes is the identity matrix..
  // clang-format on
  EXPECT_TRUE(CompareMatrices(R_QP.matrix(), m_expected, 0.0,
                              MatrixCompareType::absolute));

  // TODO(Mitiguy) Add more tests after adding canonical calculations for
  //  principal directions associated with axially symmetric shapes and shapes
  //  with three distinct principal moments of inertia.
}

// Tests the method to obtain the principal moments of inertia and axes.
GTEST_TEST(RotationalInertia, CalcPrincipalMomentsAndAxesOfInertia) {
  // For a solid ellipsoid B, form B's rotational inertia about Bcm (B's center
  // of mass) for principal directions Bx, By, Bz.
  const double a = 5.0, b = 4.0, c = 3.0;
  const double mass = 3.0;
  const double Imin =
      0.2 * mass * (b * b + c * c);  // Ixx = 1/5 m (b² + c²) small
  const double Imed =
      0.2 * mass * (a * a + c * c);  // Iyy = 1/5 m (a² + c²) medium
  const double Imax =
      0.2 * mass * (a * a + b * b);  // Izz = 1/5 m (a² + b²) large
  constexpr double kTolerance = 64 * std::numeric_limits<double>::epsilon();

  // Verify a special case of a RotationalInertia with zero products of inertia,
  // constructed with equal principal moments of inertia: Imin = Imed = Imax.
  // Note: Although there are an infinite number of principal directions, it is
  // reasonable for a user to expect (and get) R_BP = identity matrix.
  RotationalInertia<double> I_BBcm_B =
      RotationalInertia<double>(Imed, Imed, Imed);
  std::pair<Vector3<double>, drake::math::RotationMatrix<double>> I_BBcm_P =
      I_BBcm_B.CalcPrincipalMomentsAndAxesOfInertia();
  EXPECT_TRUE(CompareMatrices(Vector3<double>(Imed, Imed, Imed), I_BBcm_P.first,
                              kTolerance));
  drake::math::RotationMatrix<double> R_BP = I_BBcm_P.second;
  drake::math::RotationMatrix<double> R_BP_expected =
      drake::math::RotationMatrix<double>::Identity();
  EXPECT_TRUE(R_BP.IsExactlyEqualTo(R_BP_expected));

  // Verify a special case of a RotationalInertia with zero products of inertia,
  // constructed with principal moments of inertia having Imin < Imed < Imax.
  // Calculate I_BBcm_P, which contains B's principal inertia moments and axes.
  I_BBcm_B = RotationalInertia<double>(Imin, Imed, Imax);
  I_BBcm_P = I_BBcm_B.CalcPrincipalMomentsAndAxesOfInertia();
  EXPECT_TRUE(CompareMatrices(Vector3<double>(Imin, Imed, Imax), I_BBcm_P.first,
                              kTolerance));
  R_BP = I_BBcm_P.second;  // Columns of R_BP are eigenvectors (principal axes).
  R_BP_expected = drake::math::RotationMatrix<double>::Identity();
  EXPECT_TRUE(R_BP.IsExactlyEqualTo(R_BP_expected));

  // Verify reordering for a special case of a RotationalInertia constructed
  // with principal moments of inertia having Imed < Imin (must reorder).
  I_BBcm_B = RotationalInertia<double>(Imed, Imin, Imax);
  I_BBcm_P = I_BBcm_B.CalcPrincipalMomentsAndAxesOfInertia();
  EXPECT_TRUE(CompareMatrices(Vector3<double>(Imin, Imed, Imax), I_BBcm_P.first,
                              kTolerance));
  R_BP = I_BBcm_P.second;  // Columns of R_BP are eigenvectors (principal axes).
  Vector3<double> col_min(0.0, 1.0, 0.0);  // Direction for minimum axis.
  Vector3<double> col_med(1.0, 0.0, 0.0);  // Direction for intermediate axis.
  Vector3<double> col_max(0.0, 0.0, 1.0);  // Direction for maximum axis.
  R_BP_expected =
      drake::math::RotationMatrix<double>::MakeFromOrthonormalColumns(
          col_min, col_med, -col_max);
  EXPECT_TRUE(R_BP.IsNearlyEqualTo(R_BP_expected, kTolerance));

  // Verify reordering for a special case of a RotationalInertia constructed
  // with principal moments of inertia having Imax < Imin (must reorder).
  I_BBcm_B = RotationalInertia<double>(Imax, Imin, Imed);
  I_BBcm_P = I_BBcm_B.CalcPrincipalMomentsAndAxesOfInertia();
  EXPECT_TRUE(CompareMatrices(Vector3<double>(Imin, Imed, Imax), I_BBcm_P.first,
                              kTolerance));
  R_BP = I_BBcm_P.second;  // Columns of R_BP are eigenvectors (principal axes).
  col_min = Vector3<double>(0.0, 1.0, 0.0);  // Direction for minimum axis.
  col_med = Vector3<double>(0.0, 0.0, 1.0);  // Direction for intermediate axis.
  col_max = Vector3<double>(1.0, 0.0, 0.0);  // Direction for maximum axis.
  R_BP_expected =
      drake::math::RotationMatrix<double>::MakeFromOrthonormalColumns(
          col_min, col_med, col_max);
  EXPECT_TRUE(R_BP.IsNearlyEqualTo(R_BP_expected, kTolerance));

  // Rotate body B by 30 degrees and verify principal moments/axes. This tests
  // a general case for a rotational inertia with non-zero products of inertia.
  I_BBcm_B = RotationalInertia<double>(Imin, Imed, Imax);
  drake::math::RotationMatrix<double> R_BC =
      drake::math::RotationMatrix<double>::MakeZRotation(M_PI / 6.0);
  RotationalInertia<double> I_BBcm_C = I_BBcm_B.ReExpress(R_BC);
  I_BBcm_P = I_BBcm_C.CalcPrincipalMomentsAndAxesOfInertia();
  EXPECT_TRUE(CompareMatrices(Vector3<double>(Imin, Imed, Imax), I_BBcm_P.first,
                              kTolerance));
  // The orthogonal unit length eigenvectors Px_B, Py_B, Pz_B stored in the
  // columns of R_BP are parallel to the principal axes (lines). Since lines
  // do not have a fully-qualified direction (they lack sense), all we can check
  // is whether these principal axes (represented by Px_B, Py_B, Pz_B) are
  // parallel to the right-handed unit vectors Cx_B, Cy_B, Cz_B stored in the
  // columns of R_BC and whether they form a right-handed set.
  R_BP = I_BBcm_P.second;  // Columns of R_BP are eigenvectors (principal axes).
  const Vector3<double> Px_B = R_BP.col(0), Cx_B = R_BC.col(0);
  const Vector3<double> Py_B = R_BP.col(1), Cy_B = R_BC.col(1);
  const Vector3<double> Pz_B = R_BP.col(2), Cz_B = R_BC.col(2);
  EXPECT_NEAR(std::abs(Pz_B(2)), 1.0, kTolerance);  // Pz = [0 0 1] or [0 0 -1]
  EXPECT_NEAR(std::abs(Px_B.dot(Cx_B)), 1.0, kTolerance);  // Px parallel to Cx.
  EXPECT_NEAR(std::abs(Py_B.dot(Cy_B)), 1.0, kTolerance);  // Py parallel to Cy.
  EXPECT_NEAR(std::abs(Pz_B.dot(Cz_B)), 1.0, kTolerance);  // Pz parallel to Cz.
  EXPECT_NEAR(Px_B.cross(Py_B).dot(Pz_B), 1.0, kTolerance);  // Right-handed.
}

// Test the method RotationalInertia::CalcPrincipalMomentsOfInertia() for a
// symmetric positive-definite matrix. This type of tri-diagonal matrix arises
// when discretizing the Laplacian operator using either finite differences of
// the Finite Element Method with iso-parametric linear elements in 1D.
// This Laplacian matrix takes the form:
//     [ 2 -1  0]
// L = [-1  2 -1]
//     [ 0 -1  2]
// and has eigenvalues lambda = [2 - sqrt(2), 2, 2 + sqrt(2)] which do not
// satisfy the triangle inequality.
GTEST_TEST(RotationalInertia, PrincipalMomentsOfInertiaLaplacianTest) {
  const double Idiag = 2.0;  // The diagonal entries.
  const double Ioff = -1.0;  // The off-diagonal entries.

  // Although the inertia matrix is symmetric and positive definite, it does not
  // satisfy the triangle inequality. Hence the constructor throws an exception.
  EXPECT_THROW_IF_ARMED(
      RotationalInertia<double> I(Idiag, Idiag, Idiag, Ioff, 0.0, Ioff),
      std::logic_error);
}

// Test the correctness of multiplication with a scalar from the left.
GTEST_TEST(RotationalInertia, MultiplicationWithScalarFromTheLeft) {
  const Vector3d m(2.0, 2.3, 2.4);                     // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);                    // p for products.
  const RotationalInertia<double> I(m(0), m(1), m(2),  /* moments of inertia */
                                    p(0), p(1), p(2)); /* products of inertia */
  const double scalar = 3.0;
  const RotationalInertia<double> sxI = scalar * I;
  EXPECT_EQ(sxI.get_moments(), scalar * m);
  EXPECT_EQ(sxI.get_products(), scalar * p);

  // Multiplication by a scalar must be commutative.
  const RotationalInertia<double> Ixs = I * scalar;
  EXPECT_EQ(Ixs.get_moments(), sxI.get_moments());
  EXPECT_EQ(Ixs.get_products(), sxI.get_products());

  // Verify the scalar can be a variable symbolic expression
  const Variable a("a");  // A "variable" scalar.
  const RotationalInertia<Expression> axI = a * I.cast<Expression>();
  EXPECT_EQ(axI.get_moments(), a * m);
  EXPECT_EQ(axI.get_products(), a * p);
  // Multiplication by a scalar must be commutative.
  const RotationalInertia<Expression> Ixa = I.cast<Expression>() * a;
  EXPECT_EQ(Ixa.get_moments(), axI.get_moments());
  EXPECT_EQ(Ixa.get_products(), axI.get_products());
}

// Test the correctness of:
//  - operator+=(const RotationalInertia<T>&)
//  - operator*=(const T&)
//  - operator/=(const T&)
GTEST_TEST(RotationalInertia, OperatorPlusEqual) {
  const Vector3d m(2.0, 2.3, 2.4);                // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);               // p for products.
  RotationalInertia<double> Ia(m(0), m(1), m(2),  /* moments of inertia */
                               p(0), p(1), p(2)); /* products of inertia */
  // A second inertia.
  RotationalInertia<double> Ib = 2.0 * Ia;

  // Use of operator+=() results in: Ib = Ib + Ia.
  Ib += Ia;
  EXPECT_EQ(Ib.get_moments(), 3.0 * m);
  EXPECT_EQ(Ib.get_products(), 3.0 * p);

  // Verify correctness of operator*=().
  const double scalar = 2.2;
  Ia *= scalar;
  EXPECT_EQ(Ia.get_moments(), scalar * m);
  EXPECT_EQ(Ia.get_products(), scalar * p);
  EXPECT_THROW_IF_ARMED(Ia * (-2.2), std::exception);

  // Verify correctness of operator/=().
  Ia /= scalar;
  EXPECT_EQ(Ia.get_moments(), m);
  EXPECT_EQ(Ia.get_products(), p);

  // Verify correctness of MultiplyByScalarSkipValidityCheck() and verify it
  // does not throw if its argument causes multiplication by negative scalar.
  RotationalInertia<double> Is = Ia.MultiplyByScalarSkipValidityCheck(scalar);
  EXPECT_EQ(Is.get_moments(), scalar * m);
  EXPECT_EQ(Is.get_products(), scalar * p);

  // Verify MultiplyByScalarSkipValidityCheck() does not throw if its argument
  // causes multiplication by negative scalar (and returns an invalid inertia).
  EXPECT_NO_THROW(Is = Ia.MultiplyByScalarSkipValidityCheck(-2.2));
  EXPECT_FALSE(Is.CouldBePhysicallyValid());

  // Verify MultiplyByScalarSkipValidityCheck() does not throw if `this` is an
  // invalid RotationalInertia regardless of whether its argument is negative.
  EXPECT_NO_THROW(Is = Is.MultiplyByScalarSkipValidityCheck(-3.3));
  EXPECT_TRUE(Is.CouldBePhysicallyValid());
  EXPECT_NO_THROW(Is = Is.MultiplyByScalarSkipValidityCheck(-1.0));
  EXPECT_FALSE(Is.CouldBePhysicallyValid());

  // Show that once an invalid RotationalInertia is created, it can be used in a
  // copy constructor or assignment, etc.
  RotationalInertia<double> IsCopy(Is);
  EXPECT_FALSE(IsCopy.CouldBePhysicallyValid());
  IsCopy = 3.0 * Is;
  EXPECT_FALSE(IsCopy.CouldBePhysicallyValid());
  IsCopy *= 0.5;
  EXPECT_FALSE(IsCopy.CouldBePhysicallyValid());

  // For symbolic::Expression.
  const Variable a("a");  // A "variable" scalar.
  const RotationalInertia<Expression> Ia_over_a = Ia.cast<Expression>() / a;
  EXPECT_EQ(Ia_over_a.get_moments(), m / a);
  EXPECT_EQ(Ia_over_a.get_products(), p / a);
}

// TODO(2026-06-01): delete test ShiftOperator.
// Test the shift operator to write into a stream.
GTEST_TEST(RotationalInertia, ShiftOperator) {
  std::stringstream stream;
  RotationalInertia<double> I(1, 2.718, 3.14);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  stream << I;
#pragma GCC diagnostic pop
  std::string expected_string =
      "[    1      0      0]\n"
      "[    0  2.718      0]\n"
      "[    0      0   3.14]\n";
  EXPECT_EQ(expected_string, stream.str());
}

// Verify the output string from RotationalInertia's fmt formatter.
GTEST_TEST(RotationalInertia, ToStringFmtFormatter) {
  RotationalInertia<double> I(1, 2.718, 3.14);
  std::string expected_string =
      "[    1      0      0]\n"
      "[    0  2.718      0]\n"
      "[    0      0   3.14]\n";
  EXPECT_EQ(fmt::to_string(I), expected_string);
}

// Tests that we can correctly cast a RotationalInertia<double> to a
// RotationalInertia templated on an AutoDiffXd type.
GTEST_TEST(RotationalInertia, CastToAutoDiff) {
  const RotationalInertia<double> I_double(1, 2.718, 3.14);

  // Cast from double to AutoDiff.
  const RotationalInertia<AutoDiffXd> I_cast = I_double.cast<AutoDiffXd>();
  const Matrix3<AutoDiffXd> I_cast_matrix = I_cast.CopyToFullMatrix3();

  // Check that the values were preserved.
  EXPECT_TRUE(CompareMatrices(drake::math::ExtractValue(I_cast_matrix),
                              I_double.CopyToFullMatrix3()));

  // Check that the gradients are all empty.
  EXPECT_TRUE(CompareMatrices(drake::math::ExtractGradient(I_cast_matrix),
                              Eigen::MatrixXd(9, 0)));
}

// Tests that we can instantiate a rotational inertia with AutoDiffXd and
// we can perform some basic operations with it.
// As an example, we define the rotational inertia I_B of a body B. The
// orientation of this body in the world frame W is given by the time dependent
// rotation R_WB = Rz(theta(t)) about the z-axis with angle theta(t).
// The time derivative of theta(t) is the angular velocity wz.
// We then re-express the inertia of B in the world frame and verify the value
// of its time derivative with the expected result.
GTEST_TEST(RotationalInertia, AutoDiff) {
  // Helper lambda to extract from a matrix of auto-diff scalar's the matrix of
  // values and the matrix of derivatives.
  auto extract_derivatives = [](const Matrix3<AutoDiffXd>& M, Matrix3d& Mvalue,
                                Matrix3d& Mdot) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        Mvalue(i, j) = M(i, j).value();
        const int num_derivatives = M(i, j).derivatives().size();
        DRAKE_DEMAND((num_derivatives == 0) || (num_derivatives == 1));
        if (num_derivatives == 1) {
          Mdot(i, j) = M(i, j).derivatives()[0];
        } else {
          Mdot(i, j) = 0;
        }
      }
    }
  };

  // Construct a rotational inertia in the frame of a body B.
  const double Ix(1.0), Iy(2.0), Iz(3.0);
  RotationalInertia<AutoDiffXd> I_B(Ix, Iy, Iz);

  // Assume B has a pose rotated +20 degrees about z with respect to the
  // world frame W. The body rotates with angular velocity wz in the z-axis.
  const double angle_value = 20 * M_PI / 180.0;
  const double wz = 1.0;  // Angular velocity in the z-axis.

  AutoDiffXd angle = angle_value;
  angle.derivatives() = Vector1d(wz);
  const drake::math::RotationMatrix<AutoDiffXd> R_WB =
      drake::math::RotationMatrix<AutoDiffXd>::MakeZRotation(angle);

  // Split the rotational inertia into two Matrix3d; one with the values and
  // another one with the time derivatives.
  Matrix3<double> Rvalue_WB, Rdot_WB;
  extract_derivatives(R_WB.matrix(), Rvalue_WB, Rdot_WB);

  // The time derivative of the rotation matrix should be:
  //  Rdot = [w] * R, with w the angular velocity.
  // Therefore we have [w] = Rdot * R.transpose().
  Matrix3<double> wcross = Rdot_WB * Rvalue_WB.transpose();
  Matrix3<double> wcross_expected;
  // clang-format off
  wcross_expected << 0.0,  -wz, 0.0,
                      wz,  0.0, 0.0,
                     0.0,  0.0, 0.0;
  // clang-format on
  EXPECT_TRUE(wcross.isApprox(wcross_expected, kEpsilon));

  // Re-express inertia into another frame.
  const RotationalInertia<AutoDiffXd> I_W = I_B.ReExpress(R_WB);

  // Extract value and derivatives of I_W into two separate matrices.
  Matrix3d Ivalue_W, Idot_W;
  extract_derivatives(I_W.CopyToFullMatrix3(), Ivalue_W, Idot_W);

  // Alternatively, compute the time derivative of I_W directly in terms of the
  // known angular velocity. Since I_B is diagonal with entries Iᵢ, we can
  // expand the time derivative of I_W as:
  //  dI_W/dt = d/dt(R_WB * I_B * R_WBᵀ) = d/dt(∑ Iᵢ * x̂ᵢ * x̂ᵢᵀ) =
  //          = ∑ Iᵢ * {[w] * x̂ᵢ * x̂ᵢᵀ + ([w] * x̂ᵢ * x̂ᵢᵀ)ᵀ}
  const auto xhat = Rvalue_WB.col(0);
  const auto yhat = Rvalue_WB.col(1);
  const auto zhat = Rvalue_WB.col(2);

  Matrix3d Rdot_x = wcross * xhat * xhat.transpose();
  Rdot_x += Rdot_x.transpose().eval();
  Matrix3d Rdot_y = wcross * yhat * yhat.transpose();
  Rdot_y += Rdot_y.transpose().eval();
  Matrix3d Rdot_z = wcross * zhat * zhat.transpose();
  Rdot_z += Rdot_z.transpose().eval();

  const Matrix3d Idot_W_expected = Ix * Rdot_x + Iy * Rdot_y + Iz * Rdot_z;

  EXPECT_TRUE(Idot_W.isApprox(Idot_W_expected, kEpsilon));

  // Test method that compares to inertia matrices using the original rotational
  // inertia and then the rotated/semi-unrotated rotational inertia.
  const drake::math::RotationMatrix<AutoDiffXd> R_BW =
      drake::math::RotationMatrix<AutoDiffXd>::MakeZRotation(-angle);
  const RotationalInertia<AutoDiffXd> expectedI_B = I_W.ReExpress(R_BW);
  EXPECT_TRUE(expectedI_B.IsNearlyEqualTo(I_B, kEpsilon));
}

GTEST_TEST(RotationalInertia, CompatibleWithSymbolicExpression) {
  const Variable Ixx("Ixx");
  const Variable Iyy("Iyy");
  const Variable Izz("Izz");
  // Inertia of a body B, about its center of mass, expressed in a frame E.
  const RotationalInertia<Expression> I_Bcm_E(Ixx, Iyy, Izz);
  const Variable ell("L");
  const Variable mass("m");
  // Position vector from Bcm to a point Q, expressed in frame E.
  const Vector3<Expression> p_BcmQ_E(ell, 0.0, 0.0);
  // Same inertia but computed about point Q.
  const RotationalInertia<Expression> I_BQ_E =
      I_Bcm_E.ShiftFromCenterOfMass(mass, p_BcmQ_E);
  // By the parallel axis theorem we should get:
  const std::string Ixx_string("Ixx");
  const std::string Iyy_string("(Iyy + (pow(L, 2) * m))");
  const std::string Izz_string("(Izz + (pow(L, 2) * m))");
  EXPECT_EQ(I_BQ_E(0, 0).to_string(), Ixx_string);
  EXPECT_EQ(I_BQ_E(1, 1).to_string(), Iyy_string);
  EXPECT_EQ(I_BQ_E(2, 2).to_string(), Izz_string);

  // The expression cannot be evaluated to bool given it contains free
  // variables.
  EXPECT_THROW(I_BQ_E.CouldBePhysicallyValid(), std::exception);
}

// Verifies we can still call IsPhysicallyValid() when T = symbolic::Expression
// and get a result whenever the expression represents a constant.
GTEST_TEST(RotationalInertia, SymbolicConstant) {
  using T = symbolic::Expression;
  RotationalInertia<T> I = RotationalInertia<T>::TriaxiallySymmetric(1.0);
  ASSERT_TRUE(I.CouldBePhysicallyValid());
}

}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
