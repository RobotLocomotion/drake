#include "drake/multibody/multibody_tree/rotational_inertia.h"

#include <iomanip>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/eigen_autodiff_types.h"


namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::NumTraits;
using Eigen::Vector3d;
using std::sort;

constexpr double epsilon = std::numeric_limits<double>::epsilon();

// Test default constructor - all elements should be NaN.
GTEST_TEST(RotationalInertia, DefaultRotationalInertiaConstructorIsNaN) {
  RotationalInertia<double> default_rotational_inertia;
  Matrix3d inertia_matrix = default_rotational_inertia.CopyToFullMatrix3();
  EXPECT_TRUE(inertia_matrix.array().isNaN().all());
}

// Test constructor for a diagonal rotational inertia with all elements equal.
GTEST_TEST(RotationalInertia, DiagonalInertiaConstructor) {
  const double I0 = 3.14;
  RotationalInertia<double> I(I0);
  Vector3d moments_expected;
  moments_expected.setConstant(I0);
  Vector3d products_expected = Vector3d::Zero();
  EXPECT_EQ(I.get_moments(), moments_expected);
  EXPECT_EQ(I.get_products(), products_expected);
}

// Test constructor for a principal axes rotational inertia matrix for which
// off-diagonal elements are zero.
GTEST_TEST(RotationalInertia, PrincipalAxesConstructor) {
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  RotationalInertia<double> I(m(0), m(1), m(2));
  Vector3d moments_expected = m;
  Vector3d products_expected = Vector3d::Zero();
  EXPECT_EQ(I.get_moments(), moments_expected);
  EXPECT_EQ(I.get_products(), products_expected);
}

// Test constructor for a general rotational inertia matrix with non-zero
// off-diagonal elements for which the six entries need to be specified.
// Also test SetZero() and SetNaN methods.
GTEST_TEST(RotationalInertia, GeneralConstructor) {
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);  // p for products.
  RotationalInertia<double> I(m(0), m(1), m(2), /* moments of inertia */
                              p(0), p(1), p(2));/* products of inertia */
  Vector3d moments_expected = m;
  Vector3d products_expected = p;
  EXPECT_EQ(I.get_moments(), moments_expected);
  EXPECT_EQ(I.get_products(), products_expected);

  // Test SetZero().
  I.SetZero();
  EXPECT_TRUE((I.CopyToFullMatrix3().array() == 0).all());

  // Test SetToNaN().
  I.SetToNaN();
  EXPECT_TRUE(I.CopyToFullMatrix3().array().isNaN().all());
}

// Test calculation of trace of rotational inertia (sum of diagonal elements).
GTEST_TEST(RotationalInertia, TraceIsSumOfDiagonalElements) {
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);  // p for products.
  const RotationalInertia<double> I(m(0), m(1), m(2),
                                    p(0), p(1), p(2));
  const double trace = m(0) + m(1) + m(2);
  EXPECT_EQ(trace, I.get_trace());
}

// Test access by (i, j) indexes.
GTEST_TEST(RotationalInertia, AccessByIndexes) {
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);  // p for products.
  const RotationalInertia<double> I(m(0), m(1), m(2), /* moments of inertia */
                                    p(0), p(1), p(2));/* products of inertia */

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
// entries for unused entries, the RotationalInertia behaves as a symmetric
// matrix.
GTEST_TEST(RotationalInertia, Symmetry) {
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);  // p for products.
  RotationalInertia<double> I(m(0), m(1), m(2), /* moments of inertia */
                              p(0), p(1), p(2));/* products of inertia */

  // Tests that the copy to a full Matrix3 object is well defined and
  // leads to a symmetric matrix.
  Matrix3d Imatrix = I.CopyToFullMatrix3();
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

// TestA: Rotational inertia expressed in frame R then re-expressed in frame E.
GTEST_TEST(RotationalInertia, ReExpressInAnotherFrameA) {
  // Rod frame R located at the rod's geometric center, oriented along its
  // principal axes and z-axis along the rod's axial direction.
  // Inertia computed about Ro and expressed in R.
  const double radius = 0.1;
  const double length = 1.0;

  // Moment about its axis aligned with R's z-axis.
  const double Irr = radius * radius / 2.0;
  // Moment of inertia about an axis perpendicular to the rod's axis.
  const double Iperp = length * length / 12.0;

  RotationalInertia<double> I_Ro_R(Iperp, Iperp, Irr);

  // Rotation of +90 degrees about x.
  Matrix3<double> R_FR =
      AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();

  // Re-express in frame F using the above rotation.
  const RotationalInertia<double> I_Ro_F = I_Ro_R.ReExpress(R_FR);

  // Verify that now R's z-axis is oriented along F's y-axis.
  EXPECT_NEAR(I_Ro_F(0, 0), Iperp, epsilon);
  EXPECT_NEAR(I_Ro_F(1, 1), Irr, epsilon);
  EXPECT_NEAR(I_Ro_F(2, 2), Iperp, epsilon);

  // While at it, check if after transformation this still is a physically
  // valid inertia.
  EXPECT_TRUE(I_Ro_F.CouldBePhysicallyValid());
}

// TestB: Rotational inertia expressed in frame R then re-expressed in frame E.
GTEST_TEST(RotationalInertia, ReExpressInAnotherFrameB) {
  // Rotate rigid-body B from frame A by sequence: BodyXYZ q1, q2, q3.
  // Calculations from MotionGenesis.
  const double deg_to_rad = M_PI/180;
  const double q1 = 30*deg_to_rad,  q2 = 70*deg_to_rad,  q3 = -70*deg_to_rad;
  const double R_BAxx = cos(q2)*cos(q3);
  const double R_BAxy = sin(q3)*cos(q1) + sin(q1)*sin(q2)*cos(q3);
  const double R_BAxz = sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3);
  const double R_BAyx = -sin(q3)*cos(q2);
  const double R_BAyy = cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3);
  const double R_BAyz = sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1);
  const double R_BAzx = sin(q2);
  const double R_BAzy = -sin(q1)*cos(q2);
  const double R_BAzz = cos(q1)*cos(q2);
  Matrix3d R_BA;
  R_BA << R_BAxx, R_BAxy, R_BAxz,
          R_BAyx, R_BAyy, R_BAyz,
          R_BAzx, R_BAzy, R_BAzz;

  // Form B's rotational inertia expressed-in-frame A (irrelevant about-point).
  // const double mB = 2.0;
  const double I_Axx = 17.36933842091061;
  const double I_Ayy = 13.60270381717411;
  const double I_Azz = 10.02795776191528;
  const double I_Axy = -3.084298624204901;
  const double I_Axz = -3.634144189476002;
  const double I_Ayz = -6.539290790868233;
  const RotationalInertia<double> I_A(I_Axx, I_Ayy, I_Azz,
                                      I_Axy, I_Axz, I_Ayz);

  // Ensure rotational inertia I_A is physically valid inertia.
  EXPECT_TRUE(I_A.CouldBePhysicallyValid());

  // Re-express B's inertia matrix from expressed-in-frame A to frame B.
  const RotationalInertia<double> IDrake_B = I_A.ReExpress(R_BA);

  // Form a rotational inertia with MotionGenesis results.
  const double I_Bxx = 6.369894432933752;
  const double I_Byy = 18.38428249710324;
  const double I_Bzz = 16.24582306996301;
  const double I_Bxy = 1.134877977228511;
  const double I_Bxz = 6.018249975302059;
  const double I_Byz = -0.6136491084717202;
  const RotationalInertia<double> IMotionGenesis_B(I_Bxx, I_Byy, I_Bzz,
                                                   I_Bxy, I_Bxz, I_Byz);

  // Compare Drake results versus MotionGenesis results using generic compare.
  EXPECT_TRUE(IDrake_B.IsApproxEqualBasedOnMaximumPossibleMomentOfInertia(
                       IMotionGenesis_B, epsilon));

  // Compare Drake results versus MotionGenesis results.
  EXPECT_TRUE(IDrake_B.IsApproxMomentsAndProducts(IMotionGenesis_B, 8*epsilon));
}

// Test the method ShiftFromCenterOfMass for a body B's rotational inertia
// about-point Bcm (B's center of mass) to about-point Q.
GTEST_TEST(RotationalInertia, ShiftFromCenterOfMass) {
  const double I_BBcm_xx = 2,  I_BBcm_yy = 3,  I_BBcm_zz = 4;
  const double I_BBcm_xy = 0,  I_BBcm_xz = 0,  I_BBcm_yz = 0;
  const RotationalInertia<double> I_BBcm(I_BBcm_xx, I_BBcm_yy, I_BBcm_zz,
                                         I_BBcm_xy, I_BBcm_xz, I_BBcm_yz);
  const double mass = 1.1, xQ = 2.234, yQ = 3.14, zQ = 0.56;
  const Vector3d p_BcmQ(xQ, yQ, zQ);
  const RotationalInertia<double> I_BQ = I_BBcm.ShiftFromCenterOfMass(
      mass, p_BcmQ);

  // Compare Drake results versus by-hand results.
  const double Ixx = I_BBcm_xx + mass * (yQ*yQ + zQ*zQ);
  const double Iyy = I_BBcm_yy + mass * (xQ*xQ + zQ*zQ);
  const double Izz = I_BBcm_zz + mass * (xQ*xQ + yQ*yQ);
  const double Ixy = I_BBcm_xy - mass * xQ*yQ;
  const double Ixz = I_BBcm_xz - mass * xQ*zQ;
  const double Iyz = I_BBcm_yz - mass * yQ*zQ;
  const RotationalInertia<double> shift_inertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
  EXPECT_TRUE(I_BQ.IsApproxMomentsAndProducts(shift_inertia, 16*epsilon));
}

// Test the method ShiftToCenterOfMass for a body B's rotational inertia
// about-point P to about-point Bcm (B's center of mass).
GTEST_TEST(RotationalInertia, ShiftToCenterOfMass) {
  const double I_BP_xx = 13.2,  I_BP_yy = 8.8,   I_BP_zz = 20.3;
  const double I_BP_xy = -7.7,  I_BP_xz = -1.3,  I_BP_yz = -1.9;
  const RotationalInertia<double> I_BP(I_BP_xx, I_BP_yy, I_BP_zz,
                                       I_BP_xy, I_BP_xz, I_BP_yz);
  const double mass = 1.1, xBcm = 2.234, yBcm = 3.14, zBcm = 0.56;
  const Vector3d p_PBcm(xBcm, yBcm, zBcm);
  const RotationalInertia<double> I_BBcm = I_BP.ShiftToCenterOfMass(
      mass, p_PBcm);

  // Compare Drake results versus by-hand results.
  const double Ixx = I_BP_xx - mass * (yBcm*yBcm + zBcm*zBcm);
  const double Iyy = I_BP_yy - mass * (xBcm*xBcm + zBcm*zBcm);
  const double Izz = I_BP_zz - mass * (xBcm*xBcm + yBcm*yBcm);
  const double Ixy = I_BP_xy + mass * xBcm*yBcm;
  const double Ixz = I_BP_xz + mass * xBcm*zBcm;
  const double Iyz = I_BP_yz + mass * yBcm*zBcm;
  const RotationalInertia<double> shift_inertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
  EXPECT_TRUE(I_BBcm.IsApproxMomentsAndProducts(shift_inertia, 8*epsilon));
}

// Test the methods ShiftToOtherPointViaThisToCenterOfMass and
// ShiftToOtherPointViaOtherToCenterOfMass for a body B's
// rotational inertia about-point P to about-point Q.
GTEST_TEST(RotationalInertia, ShiftToOtherPointViaThisToCenterOfMass) {
  const double I_BP_xx = 13.2,  I_BP_yy = 8.8,   I_BP_zz = 20.3;
  const double I_BP_xy = -7.7,  I_BP_xz = -1.3,  I_BP_yz = -1.9;
  const RotationalInertia<double> I_BP(I_BP_xx, I_BP_yy, I_BP_zz,
                                       I_BP_xy, I_BP_xz, I_BP_yz);
  const double mass = 1.1, xBcm = 2.234, yBcm = 3.14, zBcm = 0.56;
  const double xP = -1.234, yP = -2.11, zP = 1.98;
  const Vector3d p_PBcm(xBcm, yBcm, zBcm);
  const Vector3d p_QP(xP, yP, zP);
  const Vector3d p_QBcm = p_QP + p_PBcm;
  const Vector3d p_BcmQ = -p_QBcm;

  // Calculate with two shifts: Shift 1 is from P to Bcm (B's center of mass).
  // Shift 2 is from Bcm to Q.
  const RotationalInertia<double> I_BBcm = I_BP.ShiftToCenterOfMass(
      mass, p_PBcm);
  const RotationalInertia<double> shift_inertia = I_BBcm.ShiftFromCenterOfMass(
      mass, p_BcmQ);

  // Calculate then test ShiftToOtherPointViaThisToCenterOfMass.
  RotationalInertia<double> I_BQ;
  I_BQ = I_BP.ShiftToOtherPointViaThisToCenterOfMass(mass, p_PBcm, p_QP);
  EXPECT_TRUE(I_BQ.IsApproxMomentsAndProducts(shift_inertia, 8*epsilon));

  // Calculate then test ShiftToOtherPointViaOtherToCenterOfMass.
  I_BQ = I_BP.ShiftToOtherPointViaOtherToCenterOfMass(mass, p_QBcm, p_QP);
  EXPECT_TRUE(I_BQ.IsApproxMomentsAndProducts(shift_inertia, 8*epsilon));
}

// Test the method CouldBePhysicallyValid after a body B's rotational inertia
// is shifted from about-point P to about-point Bcm (B's center of mass).
GTEST_TEST(RotationalInertia, CouldBePhysicallyValidA) {
  const double I_BP_xx = 2,  I_BP_yy =  3,   I_BP_zz = 4;
  const double I_BP_xy = 0,   I_BP_xz = 0,   I_BP_yz = 0;
  const RotationalInertia<double> I_BP(I_BP_xx, I_BP_yy, I_BP_zz,
                                       I_BP_xy, I_BP_xz, I_BP_yz);
  const double mass = 1.1, xBcm = 2.234, yBcm = 3.14, zBcm = 0.56;
  const Vector3d p_PBcm(xBcm, yBcm, zBcm);
  const RotationalInertia<double> I_BBcm = I_BP.ShiftToCenterOfMass(
      mass, p_PBcm);
  EXPECT_FALSE(I_BBcm.CouldBePhysicallyValid());
}

// Test the method CouldBePhysicallyValid for a rod-like object.
GTEST_TEST(RotationalInertia, CouldBePhysicallyValidB) {
  const double I_transverse = 20;
  const double I_axial = -1.0E-15;  // Although negative, this is effectively 0.
  const RotationalInertia<double> rod(I_transverse, I_transverse, I_axial);
  EXPECT_TRUE(rod.CouldBePhysicallyValid());

  const RotationalInertia<double> sphere(1.0E-5, 1.0E-5, 1.0E-5);
  EXPECT_TRUE(sphere.CouldBePhysicallyValid());

  const RotationalInertia<double> rod_minus_sphere = rod - sphere;
  EXPECT_FALSE(rod_minus_sphere.CouldBePhysicallyValid());
}

// Test the method CouldBePhysicallyValid to violate triangle inequality.
GTEST_TEST(RotationalInertia, CouldBePhysicallyValidC) {
  EXPECT_THROW(RotationalInertia<double> bad_inertia(10, 10, 30),
               std::runtime_error);
}

// Test the method CouldBePhysicallyValid for bad product of inertia
// sizing relative to moments of inertia.
GTEST_TEST(RotationalInertia, CouldBePhysicallyValidD) {
  EXPECT_THROW(RotationalInertia<double> inertia(10, 10, 10, 5.1, 0, 0),
               std::runtime_error);
}

// Test the method CouldBePhysicallyValid for principal moments of inertia
// (i.e., eigenvalues of the inertia matrix) that violate triangle inequality.
// This test is courtesy of Steve Peters via Michael Sherman.
GTEST_TEST(RotationalInertia, CouldBePhysicallyValidE) {
  const double Ixx = 2,  Iyy = 2,  Izz = 2,  Ixy = -0.8,  Ixz = 0,  Iyz = -0.8;
  EXPECT_THROW(RotationalInertia<double> inertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz),
               std::runtime_error);
}

// Test the method RotationalInertia::CalcPrincipalMomentsOfInertia() that
// computes the principal moments of inertia of a general rotational inertia
// by solving an eigenvalue problem.
GTEST_TEST(RotationalInertia, PrincipalMomentsOfInertia) {
  const double Lx = 3.0;
  const double Ly = 1.0;
  const double Lz = 5.0;

  // Rotational inertia of a box computed about its center of mass.
  const double Lx2 = Lx * Lx, Ly2 = Ly * Ly, Lz2 = Lz * Lz;
  RotationalInertia<double> I_Bc_Q(
      (Ly2 + Lz2) / 12.0,
      (Lx2 + Lz2) / 12.0,
      (Lx2 + Ly2) / 12.0);

  // Define frame Q to be rotated +20 degrees about x and another +20
  // degrees about z with respect to a frame W.
  const double angle = 20 * M_PI / 180.0;
  Matrix3<double> R_WQ =
      (AngleAxisd(angle, Vector3d::UnitZ()) *
       AngleAxisd(angle, Vector3d::UnitX())).toRotationMatrix();

  // Compute the cube's spatial inertia in this frame Q.
  // This results in a rotational inertia with all entries being non-zero, i.e
  // far away from being diagonal or diagonalizable in any trivial way.
  RotationalInertia<double> I_Bc_W = I_Bc_Q.ReExpress(R_WQ);

  // Verify that indeed this inertia in frame W contains non-zero diagonal
  // elements.
  EXPECT_TRUE((I_Bc_W.CopyToFullMatrix3().array().abs() > 0.1).all());

  // Compute the principal moments of I_Bc_W.
  Vector3d principal_moments = I_Bc_W.CalcPrincipalMomentsOfInertia();

  // The expected moments are those originally computed in I_Bc_Q, though the
  // return from RotationalInertia::CalcPrincipalMomentsOfInertia() is sorted
  // in ascending order. Therefore reorder before performing the comparison.
  Vector3d expected_principal_moments = I_Bc_Q.get_moments();
  std::sort(expected_principal_moments.data(),
            expected_principal_moments.data() +
                expected_principal_moments.size());

  // Verify against the expected value.
  EXPECT_TRUE(expected_principal_moments.isApprox(
      principal_moments, NumTraits<double>::epsilon()));
}

// Test the method RotationalInertia::CalcPrincipalMomentsOfInertia() for a
// matrix that is symmetric and positive definite. This kind of tri-diagonal
// matrix arises when discretizing the Laplacian operator using either finite
// differences or the Finite Element Method with iso-parametric linear elements
// in 1D.
// This Laplacian matrix takes the form:
//     [ 2 -1  0]
// L = [-1  2 -1]
//     [ 0 -1  2]
// and has eigenvalues lambda = [2 - sqrt(2), 2, 2 + sqrt(2)] which do not
// satisfy the triangle inequality.
GTEST_TEST(RotationalInertia, PrincipalMomentsOfInertiaLaplacianTest) {
  const double Idiag =  2.0;  // The diagonal entries.
  const double Ioff  = -1.0;  // The off-diagonal entries.

  // Even though the inertia matrix is symmetric and positive definite, it does
  // not satisfy the triangle inequality. Therefore the constructor throws an
  // exception.
  EXPECT_THROW(
      RotationalInertia<double> I(Idiag, Idiag, Idiag, Ioff, 0.0, Ioff),
      std::runtime_error);
}

// Test the correctness of multiplication with a scalar from the left.
GTEST_TEST(RotationalInertia, MultiplicationWithScalarFromTheLeft) {
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);  // p for products.
  RotationalInertia<double> I(m(0), m(1), m(2), /* moments of inertia */
                              p(0), p(1), p(2));/* products of inertia */
  const double scalar = 3.0;
  RotationalInertia<double> sxI = scalar * I;
  EXPECT_EQ(sxI.get_moments(), scalar * m);
  EXPECT_EQ(sxI.get_products(), scalar * p);

  // Multiplication by a scalar must be commutative.
  RotationalInertia<double> Ixs = I * scalar;
  EXPECT_EQ(Ixs.get_moments(), sxI.get_moments());
  EXPECT_EQ(Ixs.get_products(), sxI.get_products());
}

// Test the correctness of:
//  - operator+=(const RotationalInertia<T>&)
//  - operator*=(const T&)
//  - operator/=(const T&)
GTEST_TEST(RotationalInertia, OperatorPlusEqual) {
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);  // p for products.
  RotationalInertia<double> Ia(m(0), m(1), m(2), /* moments of inertia */
                               p(0), p(1), p(2));/* products of inertia */
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

  // Verify correctness of operator/=().
  Ia /= scalar;
  EXPECT_EQ(Ia.get_moments(), m);
  EXPECT_EQ(Ia.get_products(), p);
}

// Test the shift operator to write into a stream.
GTEST_TEST(RotationalInertia, ShiftOperator) {
  std::stringstream stream;
  RotationalInertia<double> I(1, 2.718, 3.14);
  stream << std::fixed << std::setprecision(4) << I;
  std::string expected_string =
                  "[1.0000, 0.0000, 0.0000]\n"
                  "[0.0000, 2.7180, 0.0000]\n"
                  "[0.0000, 0.0000, 3.1400]\n";
  EXPECT_EQ(expected_string, stream.str());
}

// Tests that we can instantiate a rotational inertia with AutoDiffScalar and
// we can perform some basic operations with it.
// As an example, we define the rotational inertia I_B of a body B. The
// orientation of this body in the world frame W is given by the time dependent
// rotation R_WB = Rz(theta(t)) about the z-axis with angle theta(t).
// The time derivative of theta(t) is the angular velocity wz.
// We then re-express the inertia of B in the world frame and verify the value
// of its time derivative with the expected result.
GTEST_TEST(RotationalInertia, AutoDiff) {
  typedef Eigen::AutoDiffScalar<Vector1<double>> ADScalar;

  // Helper lambda to extract from a matrix of auto-diff scalar's the matrix of
  // values and the matrix of derivatives.
  auto extract_derivatives = [](
      const Matrix3<ADScalar>& M, Matrix3d& Mvalue, Matrix3d& Mdot) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        Mvalue(i, j) = M(i, j).value();
        Mdot(i, j) = M(i, j).derivatives()[0];
      }
    }
  };

  // Construct a rotational inertia in the frame of a body B.
  double Ix(1.0), Iy(2.0), Iz(3.0);
  RotationalInertia<ADScalar> I_B(Ix, Iy, Iz);

  // Assume B has a pose rotated +20 degrees about z with respect to the
  // world frame W. The body rotates with angular velocity wz in the z-axis.
  const double angle_value = 20 * M_PI / 180.0;
  const double wz = 1.0;  // Angular velocity in the z-axis.

  ADScalar angle = angle_value;
  angle.derivatives()[0] = wz;
  Matrix3<ADScalar> R_WB =
      (AngleAxis<ADScalar>(angle, Vector3d::UnitZ())).toRotationMatrix();

  // Split the rotational inertia into two Matrix3d; one with the values and
  // another one with the time derivatives.
  Matrix3<double> Rvalue_WB, Rdot_WB;
  extract_derivatives(R_WB, Rvalue_WB, Rdot_WB);

  // The time derivative of the rotation matrix should be:
  //  Rdot = [w] * R, with w the angular velocity.
  // Therefore we have [w] = Rdot * R.transpose().
  Matrix3<double> wcross = Rdot_WB * Rvalue_WB.transpose();
  Matrix3<double> wcross_expected;
  wcross_expected << 0.0,  -wz, 0.0,
                      wz,  0.0, 0.0,
                     0.0,  0.0, 0.0;
  EXPECT_TRUE(wcross.isApprox(
      wcross_expected, epsilon));

  // Re-express inertia into another frame.
  const RotationalInertia<ADScalar> I_W = I_B.ReExpress(R_WB);

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

  EXPECT_TRUE(Idot_W.isApprox(Idot_W_expected, epsilon));
}

}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
