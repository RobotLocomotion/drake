#include "drake/multibody/tree/unit_inertia.h"

#include <limits>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/rotational_inertia.h"

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::NumTraits;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using symbolic::Environment;
using symbolic::Expression;
using symbolic::Variable;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Test default constructor which leaves entries initialized to NaN for a
// quick detection of un-initialized values.
GTEST_TEST(UnitInertia, DefaultConstructor) {
  UnitInertia<double> I;
  ASSERT_TRUE(I.IsNaN());
}

// Test constructor for a diagonal unit inertia with all elements equal.
GTEST_TEST(UnitInertia, DiagonalInertiaConstructor) {
  const double I0 = 3.14;
  UnitInertia<double> I = UnitInertia<double>::TriaxiallySymmetric(I0);
  Vector3d moments_expected;
  moments_expected.setConstant(I0);
  Vector3d products_expected = Vector3d::Zero();
  EXPECT_EQ(I.get_moments(), moments_expected);
  EXPECT_EQ(I.get_products(), products_expected);
}

// Test constructor for a principal axes unit inertia matrix for which
// off-diagonal elements are zero.
GTEST_TEST(UnitInertia, PrincipalAxesConstructor) {
  const Vector3d m(2.0, 2.3, 2.4);  // m for moments.
  UnitInertia<double> I(m(0), m(1), m(2));
  Vector3d moments_expected = m;
  Vector3d products_expected = Vector3d::Zero();
  EXPECT_EQ(I.get_moments(), moments_expected);
  EXPECT_EQ(I.get_products(), products_expected);
}

// Test constructor for a general unit inertia matrix with non-zero
// off-diagonal elements for which the six entries need to be specified.
// Also test SetZero() and SetNaN methods.
GTEST_TEST(UnitInertia, GeneralConstructor) {
  const Vector3d m(2.0, 2.3, 2.4);         // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);        // p for products.
  UnitInertia<double> I(m(0), m(1), m(2),  /* moments of inertia */
                        p(0), p(1), p(2)); /* products of inertia */
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

// Test constructor from a RotationalInertia.
GTEST_TEST(UnitInertia, ConstructorFromRotationalInertia) {
  const Vector3d m(2.0, 2.3, 2.4);  // m for moments.
  RotationalInertia<double> I(m(0), m(1), m(2));
  UnitInertia<double> G(I);
  EXPECT_EQ(G.get_moments(), I.get_moments());
  EXPECT_EQ(G.get_products(), I.get_products());
}

// Tests we can take a unit inertia expressed in a frame R and express it
// in another frame F.
GTEST_TEST(UnitInertia, ReExpressInAnotherFrame) {
  // Rod frame R located at the rod's geometric center, oriented along its
  // principal axes and z-axis along the rod's axial direction.
  // Inertia computed about Ro and expressed in R.
  const double radius = 0.1;
  const double length = 1.0;

  // Moment about its axis aligned with R's z-axis.
  const double Irr = radius * radius / 2.0;
  // Moment of inertia about an axis perpendicular to the rod's axis.
  const double Iperp = length * length / 12.0;

  UnitInertia<double> G_Ro_R(Iperp, Iperp, Irr);

  // Rotation of +90 degrees about x.
  const drake::math::RotationMatrix<double> R_FR =
      drake::math::RotationMatrix<double>::MakeXRotation(M_PI_2);

  // Re-express in frame F using the above rotation.
  const UnitInertia<double> G_Ro_F = G_Ro_R.ReExpress(R_FR);

  // Verify that now R's z-axis is oriented along F's y-axis.
  EXPECT_NEAR(G_Ro_F(0, 0), Iperp, kEpsilon);
  EXPECT_NEAR(G_Ro_F(1, 1), Irr, kEpsilon);
  EXPECT_NEAR(G_Ro_F(2, 2), Iperp, kEpsilon);
  EXPECT_TRUE(G_Ro_F.get_products().isZero(kEpsilon));

  // While at it, check if after transformation this still is a physically
  // valid inertia.
  EXPECT_TRUE(G_Ro_F.CouldBePhysicallyValid());
}

// Tests the static method to obtain the unit inertia of a point mass.
GTEST_TEST(UnitInertia, PointMass) {
  Vector3d v(1, 2, 4.2);
  Vector3d u(-1.5, 2.2, -2.0);

  // Reference triple vector product.
  Vector3d uxuxv = -u.cross(u.cross(v));
  UnitInertia<double> G = UnitInertia<double>::PointMass(u);

  // Verify that G(u) * v = -u x (u x v).
  EXPECT_TRUE(uxuxv.isApprox(G * v, kEpsilon));
}

// Tests the static method to obtain the unit inertia of a solid ellipsoid.
GTEST_TEST(UnitInertia, SolidEllipsoid) {
  const double a = 3.0;
  const double b = 4.0;
  const double c = 5.0;
  const UnitInertia<double> G_expected = UnitInertia<double>(8.2, 6.8, 5.0);
  const UnitInertia<double> G = UnitInertia<double>::SolidEllipsoid(a, b, c);
  EXPECT_TRUE(
      CompareMatrices(G_expected.get_moments(), G.get_moments(), 1e-14));
  EXPECT_TRUE(G_expected.get_products() == G.get_products());

  // The ellipsoid degenerates into a solid sphere when all semi-axes are equal.
  const UnitInertia<double> G_degenerate_ellipsoid =
      UnitInertia<double>::SolidEllipsoid(a, a, a);
  const UnitInertia<double> G_solid_sphere =
      UnitInertia<double>::SolidSphere(a);
  EXPECT_TRUE(CompareMatrices(G_degenerate_ellipsoid.get_moments(),
                              G_solid_sphere.get_moments(), 1e-14));
  EXPECT_TRUE(G_degenerate_ellipsoid.get_products() ==
              G_solid_sphere.get_products());
}

// Tests the static method to obtain the unit inertia of a solid sphere.
GTEST_TEST(UnitInertia, SolidSphere) {
  const double radius = 3.5;
  const double sphere_I = 4.9;
  const UnitInertia<double> G_expected =
      UnitInertia<double>::TriaxiallySymmetric(sphere_I);
  UnitInertia<double> G = UnitInertia<double>::SolidSphere(radius);
  EXPECT_TRUE(G_expected.get_moments() == G.get_moments());
  EXPECT_TRUE(G_expected.get_products() == G.get_products());
}

// Tests the static method to obtain the unit inertia of a hollow sphere.
GTEST_TEST(UnitInertia, HollowSphere) {
  const double radius = 3.5;
  const double sphere_I = 2.0 * radius * radius / 3.0;
  const UnitInertia<double> G_expected =
      UnitInertia<double>::TriaxiallySymmetric(sphere_I);
  UnitInertia<double> G = UnitInertia<double>::HollowSphere(radius);
  EXPECT_TRUE(G_expected.get_moments() == G.get_moments());
  EXPECT_TRUE(G_expected.get_products() == G.get_products());
}

// Tests the static method to obtain the unit inertia of a solid box.
GTEST_TEST(UnitInertia, SolidBox) {
  const double Lx = 1.0;
  const double Ly = 2.0;
  const double Lz = 3.0;
  const double Lx2 = Lx * Lx, Ly2 = Ly * Ly, Lz2 = Lz * Lz;
  const double Ixx = (Ly2 + Lz2) / 12.0;
  const double Iyy = (Lx2 + Lz2) / 12.0;
  const double Izz = (Lx2 + Ly2) / 12.0;
  const UnitInertia<double> G_expected(Ixx, Iyy, Izz);
  UnitInertia<double> G = UnitInertia<double>::SolidBox(Lx, Ly, Lz);
  EXPECT_TRUE(
      G.CopyToFullMatrix3().isApprox(G_expected.CopyToFullMatrix3(), kEpsilon));
}

// Tests the static method to obtain the unit inertia of a solid cube.
GTEST_TEST(UnitInertia, SolidCube) {
  const double L = 1.5;
  const double I = L * L / 6.0;
  const UnitInertia<double> G_expected =
      UnitInertia<double>::TriaxiallySymmetric(I);
  UnitInertia<double> G = UnitInertia<double>::SolidCube(L);
  EXPECT_TRUE(
      G.CopyToFullMatrix3().isApprox(G_expected.CopyToFullMatrix3(), kEpsilon));
}

// Tests the static method to obtain the unit inertia of a solid cylinder.
GTEST_TEST(UnitInertia, SolidCylinder) {
  const double r = 2.5;
  const double L = 1.5;
  const double I_perp = (3.0 * r * r + L * L) / 12.0;
  const double I_axial = r * r / 2.0;
  const UnitInertia<double> Gz_expected(I_perp, I_perp, I_axial);
  // Compute the unit inertia for a cylinder oriented along the z-axis.
  UnitInertia<double> Gz =
      UnitInertia<double>::SolidCylinder(r, L, Vector3d::UnitZ());
  EXPECT_TRUE(Gz.CopyToFullMatrix3().isApprox(Gz_expected.CopyToFullMatrix3(),
                                              kEpsilon));

  // Compute the unit inertia for a cylinder oriented along the x-axis.
  const UnitInertia<double> Gx =
      UnitInertia<double>::SolidCylinder(r, L, Vector3d::UnitX());
  const UnitInertia<double> Gx_expected(I_axial, I_perp, I_perp);
  EXPECT_TRUE(Gx.CopyToFullMatrix3().isApprox(Gx_expected.CopyToFullMatrix3(),
                                              kEpsilon));

  // Compute the unit inertia for a cylinder oriented along the y-axis.
  const UnitInertia<double> Gy =
      UnitInertia<double>::SolidCylinder(r, L, Vector3d::UnitY());
  const UnitInertia<double> Gy_expected(I_perp, I_axial, I_perp);
  EXPECT_TRUE(Gy.CopyToFullMatrix3().isApprox(Gy_expected.CopyToFullMatrix3(),
                                              kEpsilon));

  // Compute the unit inertia for a cylinder oriented along a non-axial vector.
  const Vector3d v = Vector3d(1.0, 2.0, 3.0).normalized();
  const UnitInertia<double> Gv = UnitInertia<double>::SolidCylinder(r, L, v);
  // Generate a rotation matrix from a Frame V in which Vz = v to frame Z where
  // Zz = zhat.
  const drake::math::RotationMatrix<double> R_ZV(
      Quaterniond::FromTwoVectors(Vector3d::UnitZ(), v));
  // Generate expected solution by computing it in the V frame and re-expressing
  // in the Z frame.
  const UnitInertia<double> Gv_expected = Gz.ReExpress(R_ZV);
  EXPECT_TRUE(Gv.CopyToFullMatrix3().isApprox(Gv_expected.CopyToFullMatrix3(),
                                              kEpsilon));
}

// Tests the static method to obtain the unit inertia of a solid cylinder B
// computed about a point Bp at the center of its base.
GTEST_TEST(UnitInertia, SolidCylinderAboutEnd) {
  const double r = 2.5;
  const double L = 1.5;
  const double I_perp = (3.0 * r * r + L * L) / 12.0 + L * L / 4.0;
  const double I_axial = r * r / 2.0;

  // Create G_BBp_A, body B's unit inertia about point Bp expressed in terms of
  // a frame A, where unit vector Az is the axial direction.
  const UnitInertia<double> G_BBp_A(I_perp, I_perp, I_axial);
  // Create a non-identity rotation matrix.
  const drake::math::RotationMatrix<double> R_AB(
      drake::math::RollPitchYaw<double>(0.1, 0.2, 0.3));
  // Form G_BBp_B (body B's unit inertia about point Bp expressed in terms
  // of body B) by reexpressing G_BBp_A using the rotation matrix R_BA.
  const UnitInertia<double> G_BBp_B = G_BBp_A.ReExpress(R_AB.inverse());

  // Create G_BBp_B more directly via SolidCylinderAboutEnd().
  const Vector3<double> unit_vec_A = Vector3<double>::UnitZ();
  const Vector3<double> unit_vec_B = R_AB.inverse() * unit_vec_A;
  UnitInertia<double> G_BBp_B_test =
      UnitInertia<double>::SolidCylinderAboutEnd(r, L, unit_vec_B);
  EXPECT_TRUE(G_BBp_B.CopyToFullMatrix3().isApprox(
      G_BBp_B_test.CopyToFullMatrix3(), kEpsilon));

  // Ensure a bad unit vector throws an exception.
  const Vector3<double> bad_vec(1, 0.1, 0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      UnitInertia<double>::SolidCylinderAboutEnd(r, L, bad_vec),
      "[^]* The unit_vector argument .* is not a unit vector.[^]*");
}

// Tests the static method to obtain the unit inertia of a solid capsule
// computed about its center of mass.
GTEST_TEST(UnitInertia, SolidCapsule) {
  const double r = 0.5;
  const double L = 1.5;
  const double density = 0.1;

  const double r2 = r * r;
  const double r3 = r2 * r;
  const double L2 = L * L;

  const double volume_cylinder = M_PI * r2 * L;
  const double volume_sphere = M_PI * 4.0 * r3 / 3.0;

  const double mass_cylinder = volume_cylinder * density;
  const double mass_sphere = volume_sphere * density;
  const double mass_capsule = mass_sphere + mass_cylinder;

  // The inertia properties of a capsule is calculated three ways.
  // Calculation 1: Multiply the capsule's mass with its unit inertia.
  const UnitInertia<double> G_capsule =
      UnitInertia<double>::SolidCapsule(r, L, Vector3d::UnitZ());
  RotationalInertia<double> I_capsule = mass_capsule * G_capsule;

  // Calculation 2: Calculate the inertia analytically.
  // I = Ic + Is, where Ic is the inertia of the cylinder part of the capsule,
  // and Is is the inertia of the two half spheres.
  //
  // Suppose m₁ is the mass of the cylinder part and m₂ is the mass of
  // the spherical part (sum of the two half-spheres).
  //
  // Ic = diag(Ic_xx, Ic_yy, Ic_zz).
  // Ic_xx = Ic_yy = m₁(L²/12 + r²/4).
  // Ic_zz = m₁r²/2.
  //
  // Is = diag(Is_xx, Is_yy, Is_zz).
  // Is_xx = Is_yy = m₂(2r²/5 + L²/4 + 3Lr/8), which is calculated by shifting
  // each half of the sphere twice (first from Ho, the geometrically significant
  // point at the center of the connection between the half-sphere and the
  // cylinder to Hcm, the center of mass of the half sphere, then from Hcm to
  // the capsule's center of mass).
  // Is_zz = 2m₂r²/5.
  const double Ixx =
      mass_cylinder * (L2 / 12.0 + r2 / 4.0) +
      mass_sphere * (2.0 * r2 / 5.0 + L2 / 4.0 + 3 * L * r / 8.0);
  const double Iyy = Ixx;
  const double Izz = mass_cylinder * r2 / 2.0 + mass_sphere * 2.0 * r2 / 5.0;
  const RotationalInertia<double> I_capsule_expected(Ixx, Iyy, Izz);
  EXPECT_TRUE(CompareMatrices(I_capsule.get_moments(),
                              I_capsule_expected.get_moments(), kEpsilon));
  EXPECT_TRUE(CompareMatrices(I_capsule.get_products(),
                              I_capsule_expected.get_products(), kEpsilon));

  // Calculation 3: Use built-in shift methods.
  // The capsule is regarded as a cylinder C of length L and radius r and two
  // half-spheres (each of radius r). The first half-sphere H is rigidly fixed
  // to one end of cylinder C so that the intersection between H and C forms
  // a circle centered at point Ho.  Similarly, the other half-sphere is rigidly
  // fixed to the other end of cylinder C.

  // Form the inertia for a unit mass solid half sphere H about Ho.
  const double mh = 0.5 * mass_sphere;  // mh is the mass of half-sphere H.
  RotationalInertia<double> Ih = mh * UnitInertia<double>::SolidSphere(r);

  // Form the position vector from Ho to Hcm (H's center of mass), expressed in
  // the capsule frame C.  H is the "lower" half-sphere (negative z direction).
  const Vector3<double> p_HoHcm_C(0, 0, -3.0 * r / 8.0);
  // The position vector from Hcm to Ccm (the capsule's center of mass) is
  const Vector3<double> p_HcmCcm_C(0, 0, 3.0 * r / 8.0 + 0.5 * L);
  // Shift H's inertia from Hcm to Ccm (the other half-sphere shifts similarly).
  Ih.ShiftToThenAwayFromCenterOfMassInPlace(mh, p_HoHcm_C, p_HcmCcm_C);

  // Form the cylinder's inertia about its center of mass and verify results.
  const RotationalInertia<double> I_cylinder =
      mass_cylinder *
      UnitInertia<double>::SolidCylinder(r, L, Vector3d::UnitZ());
  I_capsule = I_cylinder + 2 * Ih;
  EXPECT_TRUE(CompareMatrices(I_capsule.get_moments(),
                              I_capsule_expected.get_moments(), kEpsilon));
  EXPECT_TRUE(CompareMatrices(I_capsule.get_products(),
                              I_capsule_expected.get_products(), kEpsilon));

  // Ensure a bad unit vector throws an exception.
  const Vector3<double> bad_uvec(1, 0.1, 0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      UnitInertia<double>::SolidCapsule(r, L, bad_uvec),
      "[^]* The unit_vector argument .* is not a unit vector.[^]*");
}

// Tests a degenerate capsule (into sphere) has the same unit inertia as a
// sphere.
GTEST_TEST(UnitInertia, SolidCapsuleDegenerateIntoSolidSphere) {
  const double r = 2.5;
  const double L = 0;
  const UnitInertia<double> G_expected = UnitInertia<double>::SolidSphere(r);
  const UnitInertia<double> G =
      UnitInertia<double>::SolidCapsule(r, L, Vector3d::UnitZ());
  EXPECT_TRUE(CompareMatrices(G.get_moments(), G_expected.get_moments()));
  EXPECT_TRUE(CompareMatrices(G.get_products(), G_expected.get_products()));
}

// Tests a degenerate capsule (into a thin rod) has the same unit inertia as a
// thin rod.
GTEST_TEST(UnitInertia, SolidCapsuleDegenerateIntoThinRod) {
  const double r = 0;
  const double L = 1.5;
  const UnitInertia<double> G_expected =
      UnitInertia<double>::ThinRod(L, Vector3d::UnitZ());
  const UnitInertia<double> G_degenerate =
      UnitInertia<double>::SolidCapsule(r, L, Vector3d::UnitZ());
  EXPECT_TRUE(
      CompareMatrices(G_degenerate.get_moments(), G_expected.get_moments()));
  EXPECT_TRUE(
      CompareMatrices(G_degenerate.get_products(), G_expected.get_products()));
  // The unit inertia of the capsule should be close to that of a thin rod when
  // the radius is vanishingly small.
  const UnitInertia<double> G_close_to_degenerate =
      UnitInertia<double>::SolidCapsule(kEpsilon, L, Vector3d::UnitZ());
  EXPECT_TRUE(CompareMatrices(G_close_to_degenerate.get_moments(),
                              G_expected.get_moments(), kEpsilon));
  EXPECT_TRUE(CompareMatrices(G_close_to_degenerate.get_products(),
                              G_expected.get_products()));
}

// Unit tests for the factory method UnitInertia::AxiallySymmetric().
// This test creates the unit inertia for a cylinder of radius r and length L
// with its longitudinal axis aligned with a vector b using two different
// methods:
// 1. Using the AxiallySymmetric() factory.
// 2. Using the SolidCylinder() factory to create the unit inertia of a cylinder
//    aligned with the z axis which is then re-expressed to the same frame E as
//    the vector b_E.
// The two unit inertias are then compared to verify AxiallySymmetric().
GTEST_TEST(UnitInertia, AxiallySymmetric) {
  const double kTolerance = 5 * kEpsilon;

  // Cylinder's radius and length.
  const double r = 2.5;
  const double L = 1.5;
  // Cylinder's moments about its longitudinal axis (I_axial) and about any
  // other axis perpendicular to its longitudinal axis (I_perp).
  const double I_perp = (3.0 * r * r + L * L) / 12.0;
  const double I_axial = r * r / 2.0;

  // Cylinder's axis. A vector on the y-z plane, at -pi/4 from the z axis.
  const Vector3d b_E = (Vector3d::UnitY() + Vector3d::UnitZ()).normalized();

  // Rotation of -pi/4 about the x axis, from a Z frame having its z axis
  // aligned with the z-axis of the cylinder to the expressed-in frame E.
  const drake::math::RotationMatrix<double> R_EZ =
      drake::math::RotationMatrix<double>::MakeXRotation(-M_PI_4);

  // Unit inertia computed with AxiallySymmetric().
  UnitInertia<double> G_E =
      UnitInertia<double>::AxiallySymmetric(I_axial, I_perp, b_E);

  // The expected inertia is that of a cylinder of radius r and height L with
  // its longitudinal axis aligned with b.
  UnitInertia<double> G_Z =
      UnitInertia<double>::SolidCylinder(r, L, Vector3d::UnitZ());
  UnitInertia<double> G_E_expected = G_Z.ReExpress(R_EZ);

  // Verify the computed values.
  EXPECT_TRUE(G_E.CopyToFullMatrix3().isApprox(G_E_expected.CopyToFullMatrix3(),
                                               kEpsilon));

  // Verify the principal moments indeed are I_perp and I_axial:
  Vector3d moments = G_E.CalcPrincipalMomentsOfInertia();
  // The two smallest moments should match I_perp in this case.
  EXPECT_NEAR(moments(0), I_perp, kTolerance);
  EXPECT_NEAR(moments(1), I_perp, kTolerance);
  // The largest moments should match I_axial in this case.
  EXPECT_NEAR(moments(2), I_axial, kTolerance);
}

// Unit test for the factory methods:
//   - UnitInertia::StraightLine().
//   - UnitInertia::ThinRod().
// This test creates the unit inertia for a thin rod or wire of length L with
// its axis aligned with an arbitrary vector b. The unit inertia is computed
// using three methods:
// 1. Using the factory StraightLine().
// 2. Using the SolidCylinder() factory to create the unit inertia of a zero
//    radius cylinder aligned with the z axis which is then re-expressed to the
//    same frame E as the vector b_E.
// 3. Using the factory ThinRod().
// The three unit inertia objects are then compared to verify the results.
GTEST_TEST(UnitInertia, ThinRod) {
  const double L = 1.5;  // Rod's length.

  // Moment of inertia for an infinitesimally thin rod of length L.
  const double I_rod = L * L / 12.0;

  // Rod's axis. A vector on the y-z plane, at -pi/4 from the z axis.
  const Vector3d b_E = (Vector3d::UnitY() + Vector3d::UnitZ()).normalized();

  // Rotation of -pi/4 about the x axis, from a Z frame having its z-axis
  // aligned with the rod to the expressed-in frame E.
  const drake::math::RotationMatrix<double> R_EZ =
      drake::math::RotationMatrix<double>::MakeXRotation(-M_PI_4);

  // Unit inertia computed with StraightLine().
  UnitInertia<double> G_E = UnitInertia<double>::StraightLine(I_rod, b_E);

  // The expected inertia is that of a cylinder of zero radius and height L with
  // its longitudinal axis aligned with b.
  UnitInertia<double> G_Z =
      UnitInertia<double>::SolidCylinder(0, L, Vector3d::UnitZ());
  UnitInertia<double> G_E_expected = G_Z.ReExpress(R_EZ);

  // Verify the computed values.
  EXPECT_TRUE(G_E.CopyToFullMatrix3().isApprox(G_E_expected.CopyToFullMatrix3(),
                                               kEpsilon));

  // Verify the result from ThinRod():
  UnitInertia<double> G_rod = UnitInertia<double>::ThinRod(L, b_E);
  EXPECT_TRUE(G_rod.CopyToFullMatrix3().isApprox(
      G_E_expected.CopyToFullMatrix3(), kEpsilon));
}

// Helper function to test the unit inertia of a tetrahedron.
UnitInertia<double> CalcSolidTetrahedronUnitInertia(const Vector3<double>& p1,
                                                    const Vector3<double>& p2,
                                                    const Vector3<double>& p3) {
  // Note: Position vectors p1, p2, p3 start at Bo (a vertex of the tetrahedron)
  // and go to the other 3 vertices B1, B2, B3. They must all be be expressed in
  // the same frame.  The code below uses the algorithms from:
  // https://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
  // http://number-none.com/blow/inertia/bb_inertia.doc
  // The co-variance matrix of a canonical tetrahedron B with vertices at
  // (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1) with assumed *unit* density.
  Matrix3<double> C_canonical;
  // clang-format off
  C_canonical << 1 / 60.0,  1 / 120.0, 1 / 120.0,
                 1 / 120.0, 1 / 60.0,  1 / 120.0,
                 1 / 120.0, 1 / 120.0, 1 / 60.0;
  // clang-format on

  // The *transpose* of the affine transformation takes us from the canonical
  // co-variance matrix to the matrix for the particular tetrahedron.
  Matrix3<double> A_T = Matrix3<double>::Zero();
  A_T.row(0) = p1;  // Position vector from vertex Bo to vertex B1.
  A_T.row(1) = p2;  // Position vector from vertex Bo to vertex B2.
  A_T.row(2) = p3;  // Position vector from vertex Bo to vertex B3.
  // We're computing C += det(A)·ACAᵀ. Fortunately, det(A) is equal to 6.
  const Matrix3<double> C = 6 * A_T.transpose() * C_canonical * A_T;

  // B's unit inertia about Bo is calculated G_BBo = C.trace * 1₃ - C.
  // Since G_BBo is symmetric, it is more efficient to directly calculate only
  // six elements than to perform the matrix multiplication.
  const double trace_C = C.trace();
  const double Ixx = trace_C - C(0, 0);
  const double Iyy = trace_C - C(1, 1);
  const double Izz = trace_C - C(2, 2);
  const double Ixy = -C(1, 0);
  const double Ixz = -C(2, 0);
  const double Iyz = -C(2, 1);
  return UnitInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);  // G_BBo
}

// Test the 3-argument method that forms unit inertia of a solid tetrahedron.
GTEST_TEST(UnitInertia, SolidTetrahedronAboutVertex) {
  const Vector3<double> p1(1, 0, 0);
  const Vector3<double> p2(0, 2, 0);
  const Vector3<double> p3(0, 0, 3);

  UnitInertia<double> G_expected = CalcSolidTetrahedronUnitInertia(p1, p2, p3);
  UnitInertia<double> G_dut =
      UnitInertia<double>::SolidTetrahedronAboutVertex(p1, p2, p3);

  // An empirical tolerance: two bits = 2^2 times machine epsilon.
  const double kTolerance = 4 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(G_expected.CopyToFullMatrix3(),
                              G_dut.CopyToFullMatrix3(), kTolerance));

  // Show no change if shuffle last 2 arguments.
  G_dut = UnitInertia<double>::SolidTetrahedronAboutVertex(p1, p3, p2);
  EXPECT_TRUE(CompareMatrices(G_expected.CopyToFullMatrix3(),
                              G_dut.CopyToFullMatrix3(), kTolerance));
}

// Test the 4-argument method that forms unit inertia of a solid tetrahedron.
GTEST_TEST(UnitInertia, SolidTetrahedronAboutPoint) {
  Vector3<double> p_AB0(0, 0, 0);
  Vector3<double> p_AB1(1, 1, 0);
  Vector3<double> p_AB2(0, 2, 0);
  Vector3<double> p_AB3(0, 3, 3);

  // Do a sanity check that SolidTetrahedronAboutPoint() simplifies
  // to SolidTetrahedronAboutVertex() when p_AB0 is the zero vector.
  UnitInertia<double> G_BA_expected =
      UnitInertia<double>::SolidTetrahedronAboutVertex(p_AB1, p_AB2, p_AB3);
  UnitInertia<double> G_BA = UnitInertia<double>::SolidTetrahedronAboutPoint(
      p_AB0, p_AB1, p_AB2, p_AB3);

  // An empirical tolerance: two bits = 2^2 times machine epsilon.
  const double kTolerance = 4 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(G_BA_expected.CopyToFullMatrix3(),
                              G_BA.CopyToFullMatrix3(), kTolerance));

  // As will be momentarily useful, form position from B0 to Bcm.
  const Vector3<double> p_B0Bcm = 0.25 * (p_AB1 + p_AB2 + p_AB3);

  // Check a more general case in which p_AB0 is a non-zero vector.
  p_AB0 = Vector3<double>(0.1, 0.4, 0.5);
  p_AB1 += p_AB0;
  p_AB2 += p_AB0;
  p_AB3 += p_AB0;
  G_BA = UnitInertia<double>::SolidTetrahedronAboutPoint(p_AB0, p_AB1, p_AB2,
                                                         p_AB3);

  // Shift G_BA_expected from about-point B0 to about-point A.
  const Vector3<double> p_ABcm = p_AB0 + p_B0Bcm;
  G_BA_expected.ShiftToThenAwayFromCenterOfMassInPlace(
      /* mass = */ 1, p_B0Bcm, p_ABcm);
  EXPECT_TRUE(CompareMatrices(G_BA_expected.CopyToFullMatrix3(),
                              G_BA.CopyToFullMatrix3(), kTolerance));

  // Show no change if shuffle last 2 arguments to SolidTetrahedronAboutPoint().
  G_BA = UnitInertia<double>::SolidTetrahedronAboutPoint(p_AB0, p_AB1, p_AB3,
                                                         p_AB2);
  EXPECT_TRUE(CompareMatrices(G_BA_expected.CopyToFullMatrix3(),
                              G_BA.CopyToFullMatrix3(), kTolerance));
}

// Tests the methods:
//  - ShiftFromCenterOfMassInPlace()
//  - ShiftFromCenterOfMass()
//  - ShiftToCenterOfMassInPlace()
//  - ShiftToCenterOfMass()
GTEST_TEST(UnitInertia, ShiftFromCenterOfMassInPlace) {
  const double r = 2.5;
  const double L = 1.5;
  const UnitInertia<double> G_expected =
      UnitInertia<double>::SolidCylinderAboutEnd(r, L,
                                                 Vector3<double>::UnitZ());
  UnitInertia<double> G =
      UnitInertia<double>::SolidCylinder(r, L, Vector3<double>::UnitZ());
  EXPECT_FALSE(G.CopyToFullMatrix3().isApprox(G_expected.CopyToFullMatrix3(),
                                              kEpsilon));  // Not equal yet.
  G.ShiftFromCenterOfMassInPlace({0.0, 0.0, L / 2.0});
  EXPECT_TRUE(G.CopyToFullMatrix3().isApprox(
      G_expected.CopyToFullMatrix3(), kEpsilon));  // Equal after shifting.
  EXPECT_TRUE(G.CouldBePhysicallyValid());

  // Now test that we can perform the inverse operation and obtain the original
  // unit inertia.
  // As a shift into a new object:
  const UnitInertia<double> G2 = G.ShiftToCenterOfMass({0.0, 0.0, -L / 2.0});
  // As a shift in place:
  G.ShiftToCenterOfMassInPlace({0.0, 0.0, -L / 2.0});
  const UnitInertia<double> G_cylinder =
      UnitInertia<double>::SolidCylinder(r, L, Vector3<double>::UnitZ());
  EXPECT_TRUE(
      G.CopyToFullMatrix3().isApprox(G_cylinder.CopyToFullMatrix3(), kEpsilon));
  EXPECT_TRUE(G2.CopyToFullMatrix3().isApprox(G_cylinder.CopyToFullMatrix3(),
                                              kEpsilon));

  // Create a new object.
  const UnitInertia<double> G3 =
      G_cylinder.ShiftFromCenterOfMass({0.0, 0.0, L / 2.0});
  EXPECT_TRUE(G3.CopyToFullMatrix3().isApprox(G_expected.CopyToFullMatrix3(),
                                              kEpsilon));
  EXPECT_TRUE(G3.CouldBePhysicallyValid());
}

// Test the UnitInertia function that determines an inertia-equivalent shape.
GTEST_TEST(UnitInertia, CalcPrincipalHalfLengthsAndAxesForEquivalentShape) {
  // Consider a body B whose shape (e.g., an ellipsoid or box) is defined by
  // semi-diameters (half-lengths) a, b, c.
  const double a = 5.0, b = 4.0, c = 3.0;
  constexpr double kTolerance = 64 * std::numeric_limits<double>::epsilon();
  const drake::math::RotationMatrix R_identity =
      drake::math::RotationMatrix<double>::Identity();

  // Form the unit inertia G_BBcm_B for a solid ellipsoid B. Verify the function
  // under test reproduces semi-diameters lmax = a, lmed = b, lmin = c.
  // Verify principal directions Ax, Ay, Az (R_BA is an identity matrix).
  const double shape_factor_solid_ellipsoid = 0.2;  // Inertia shape factor.
  double Gmin =
      shape_factor_solid_ellipsoid * (b * b + c * c);  // 1/5 (b² + c²)
  double Gmed =
      shape_factor_solid_ellipsoid * (a * a + c * c);  // 1/5 (a² + c²)
  double Gmax =
      shape_factor_solid_ellipsoid * (a * a + b * b);  // 1/5 (a² + b²)
  UnitInertia<double> G_BBcm_B = UnitInertia<double>(Gmin, Gmed, Gmax);
  auto [abc, R_BA] = G_BBcm_B.CalcPrincipalHalfLengthsAndAxesForEquivalentShape(
      shape_factor_solid_ellipsoid);
  EXPECT_TRUE(CompareMatrices(Vector3<double>(a, b, c), abc, kTolerance));
  EXPECT_TRUE(R_BA.IsExactlyEqualTo(R_identity));

  // Form the unit inertia G_BBcm_B for a solid box B. Verify the function
  // under test reproduces half-lengths lmax = a, lmed = b, lmin = c.
  // Verify principal directions Ax, Ay, Az (R_BA is an identity matrix).
  const double shape_factor_solid_box = 1.0 / 3.0;  // Inertia shape factor.
  Gmin = shape_factor_solid_box * (b * b + c * c);  // Gxx = 1/3 (b² + c²) small
  Gmed =
      shape_factor_solid_box * (a * a + c * c);  // Gyy = 1/3 (a² + c²)  medium
  Gmax = shape_factor_solid_box * (a * a + b * b);  // Gzz = 1/3 (a² + b²) large
  G_BBcm_B = UnitInertia<double>(Gmin, Gmed, Gmax);
  std::tie(abc, R_BA) =
      G_BBcm_B.CalcPrincipalHalfLengthsAndAxesForEquivalentShape(
          shape_factor_solid_box);
  EXPECT_TRUE(CompareMatrices(Vector3<double>(a, b, c), abc, kTolerance));
  EXPECT_TRUE(R_BA.IsExactlyEqualTo(R_identity));

  // For a solid box B with principal moments of inertia Gmed < Gmin
  // (not Gmin < Gmed < Gmax), verify the function under test produces
  // lmax = a, lmed = b, lmin = c (unchanged order).
  // Verify reordered principal directions (R_BA is not an identity matrix).
  G_BBcm_B = UnitInertia<double>(Gmed, Gmin, Gmax);
  std::tie(abc, R_BA) =
      G_BBcm_B.CalcPrincipalHalfLengthsAndAxesForEquivalentShape(
          shape_factor_solid_box);
  EXPECT_TRUE(CompareMatrices(Vector3<double>(a, b, c), abc, kTolerance));
  Vector3<double> col_min(0.0, 1.0, 0.0);  // Direction for minimum axis.
  Vector3<double> col_med(1.0, 0.0, 0.0);  // Direction for intermediate axis.
  Vector3<double> col_max(0.0, 0.0, 1.0);  // Direction for maximum axis.
  drake::math::RotationMatrix<double> R_BA_expected =
      drake::math::RotationMatrix<double>::MakeFromOrthonormalColumns(
          col_min, col_med, -col_max);
  EXPECT_TRUE(R_BA.IsNearlyEqualTo(R_BA_expected, kTolerance));

  // For a solid box B, verify the function under test with argument
  // shape_factor_solid_ellipsoid produces properly scaled semi-diameters
  // (half-lengths) [a, b, c], i.e., a solid box reshaped as an ellipsoid.
  // Verify principal directions Ax, Ay, Az (R_BA is an identity matrix).
  std::tie(abc, R_BA) =
      G_BBcm_B.CalcPrincipalHalfLengthsAndAxesForEquivalentShape(
          shape_factor_solid_ellipsoid);
  const double ratio =
      std::sqrt(shape_factor_solid_box / shape_factor_solid_ellipsoid);
  EXPECT_NEAR(std::abs(ratio), std::sqrt(1.0 / 0.6), kTolerance);
  EXPECT_TRUE(
      CompareMatrices(ratio * Vector3<double>(a, b, c), abc, kTolerance));

  // For a hollow sphere B, verify the function under test produces
  // semi-diameters lmax = a, lmed = a, lmin = a.
  // Verify principal directions Ax, Ay, Az (R_BA is an identity matrix).
  // For a hollow sphere with radius a, Gmin = Gmed = Gmax = 2/3 a².
  const double shape_factor_hollow_ellipsoid = 1.0 / 3.0;
  const double G_hollow_sphere = 2.0 / 3.0 * a * a;
  G_BBcm_B =
      UnitInertia<double>(G_hollow_sphere, G_hollow_sphere, G_hollow_sphere);
  std::tie(abc, R_BA) =
      G_BBcm_B.CalcPrincipalHalfLengthsAndAxesForEquivalentShape(
          shape_factor_hollow_ellipsoid);
  EXPECT_TRUE(CompareMatrices(Vector3<double>(a, a, a), abc, kTolerance));
  EXPECT_TRUE(R_BA.IsExactlyEqualTo(R_identity));

  // For a particle B, verify the function under test produces
  // lmax = lmed = lmin = 0 (the inertia_shape_factor is not relevant).
  // Verify principal directions Ax, Ay, Az (R_BA is an identity matrix).
  G_BBcm_B = UnitInertia<double>(0, 0, 0);
  std::tie(abc, R_BA) =
      G_BBcm_B.CalcPrincipalHalfLengthsAndAxesForEquivalentShape(0.123);
  EXPECT_TRUE(CompareMatrices(Vector3<double>(0, 0, 0), abc, kTolerance));
  EXPECT_TRUE(R_BA.IsExactlyEqualTo(R_identity));

  // For a thin solid rod B, verify the function under test produces
  // lmax = a, lmed = 0, lmin = 0 (rod dimensions).
  // Verify principal directions Ax, Ay, Az (R_BA is an identity matrix).
  const double I_rod = 1.0 / 3.0 * a * a;  // Imed = Imax = 1/12 (2*a)².
  G_BBcm_B = UnitInertia<double>(0, I_rod, I_rod);
  std::tie(abc, R_BA) =
      G_BBcm_B.CalcPrincipalHalfLengthsAndAxesForEquivalentShape(
          shape_factor_solid_box);
  EXPECT_TRUE(CompareMatrices(Vector3<double>(a, 0, 0), abc, kTolerance));
  EXPECT_TRUE(R_BA.IsExactlyEqualTo(R_identity));

  // For a thin solid flat plate B, verify the function under test produces
  // lmax = a, lmed = b, lmin = 0 (plate dimensions).
  // Verify principal directions Ax, Ay, Az (R_BA is an identity matrix).
  const double Imin_plate = 1.0 / 3.0 * b * b;        // Gxx = 1/12 (2*b)²
  const double Imed_plate = 1.0 / 3.0 * a * a;        // Gyy = 1/12 (2*a)²
  const double Imax_plate = Imin_plate + Imed_plate;  // Gzz = 1/3 (a² + b²)
  G_BBcm_B = UnitInertia<double>(Imin_plate, Imed_plate, Imax_plate);
  std::tie(abc, R_BA) =
      G_BBcm_B.CalcPrincipalHalfLengthsAndAxesForEquivalentShape(
          shape_factor_solid_box);
  EXPECT_TRUE(CompareMatrices(Vector3<double>(a, b, 0), abc, kTolerance));
  EXPECT_TRUE(R_BA.IsExactlyEqualTo(R_identity));

  // For a solid box B whose principal inertia axes are rotated 30 degrees from
  // frame B, verify the function under test produces
  // lmax = a, lmed = b, lmin = c (unchanged order).
  // Verify principal directions Ax, Ay, Az (R_BA is not an identity matrix).
  // Note: This tests a unit inertia with non-zero products of inertia.
  G_BBcm_B = UnitInertia<double>(Gmin, Gmed, Gmax);
  drake::math::RotationMatrix<double> R_BC =
      drake::math::RotationMatrix<double>::MakeZRotation(M_PI / 6.0);
  UnitInertia<double> G_BBcm_C = G_BBcm_B.ReExpress(R_BC);
  std::tie(abc, R_BA) =
      G_BBcm_C.CalcPrincipalHalfLengthsAndAxesForEquivalentShape(
          shape_factor_solid_box);
  EXPECT_TRUE(CompareMatrices(Vector3<double>(a, b, c), abc, kTolerance));

  // The orthogonal unit length eigenvectors Ax_B, Ay_B, Az_B stored in the
  // columns of R_BA are parallel to the principal axes (lines). Since lines
  // do not have a fully-qualified direction (they lack sense), all we can check
  // is whether these principal axes (represented by Ax_B, Ay_B, Az_B) are
  // parallel to the right-handed unit vectors Cx_B, Cy_B, Cz_B stored in the
  // columns of R_BC and whether they form a right-handed set.
  const Vector3<double> Ax_B = R_BA.col(0), Cx_B = R_BC.col(0);
  const Vector3<double> Ay_B = R_BA.col(1), Cy_B = R_BC.col(1);
  const Vector3<double> Az_B = R_BA.col(2), Cz_B = R_BC.col(2);
  EXPECT_NEAR(std::abs(Az_B(2)), 1.0, kTolerance);  // Az = [0 0 1] or [0 0 -1]
  EXPECT_NEAR(std::abs(Ax_B.dot(Cx_B)), 1.0, kTolerance);  // Ax parallel to Cx.
  EXPECT_NEAR(std::abs(Ay_B.dot(Cy_B)), 1.0, kTolerance);  // Ay parallel to Cy.
  EXPECT_NEAR(std::abs(Az_B.dot(Cz_B)), 1.0, kTolerance);  // Az parallel to Cz.
  EXPECT_NEAR(Ax_B.cross(Ay_B).dot(Az_B), 1.0, kTolerance);  // Right-handed.
}

// Tests that we can correctly cast a UnitInertia<double> to a UnitInertia
// templated on AutoDiffXd.
GTEST_TEST(UnitInertia, CastToAutoDiff) {
  const UnitInertia<double> I_double(1, 2.718, 3.14);

  // Cast from double to AutoDiff.
  const UnitInertia<AutoDiffXd> I_cast = I_double.cast<AutoDiffXd>();
  const Matrix3<AutoDiffXd> I_cast_matrix = I_cast.CopyToFullMatrix3();

  // Check that the values were preserved.
  EXPECT_TRUE(CompareMatrices(drake::math::ExtractValue(I_cast_matrix),
                              I_double.CopyToFullMatrix3()));

  // Check that the gradients are all empty.
  EXPECT_TRUE(CompareMatrices(drake::math::ExtractGradient(I_cast_matrix),
                              Eigen::MatrixXd(9, 0)));
}

// Tests that we can instantiate a unit inertia with AutoDiffXd and we can
// perform some basic operations with it.
// As an example, we define the unit inertia G_B of a body B. The
// orientation of this body in the world frame W is given by the time dependent
// rotation R_WB = Rz(theta(t)) about the z-axis with angle theta(t).
// The time derivative of theta(t) is the angular velocity wz.
// We then re-express the inertia of B in the world frame and verify the value
// of its time derivative with the expected result.
GTEST_TEST(UnitInertia, AutoDiff) {
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

  // Construct a unit inertia in the frame of a body B.
  double Ix(1.0), Iy(2.0), Iz(3.0);
  UnitInertia<AutoDiffXd> G_B(Ix, Iy, Iz);

  // Assume B has a pose rotated +20 degrees about z with respect to the
  // world frame W. The body rotates with angular velocity wz in the z-axis.
  const double angle_value = 20 * M_PI / 180.0;
  const double wz = 1.0;  // Angular velocity in the z-axis.

  AutoDiffXd angle = angle_value;
  angle.derivatives() = Vector1d(wz);
  const drake::math::RotationMatrix<AutoDiffXd> R_WB =
      drake::math::RotationMatrix<AutoDiffXd>::MakeZRotation(angle);

  // Split the unit inertia into two Matrix3d; one with the values and
  // another one with the time derivatives.
  Matrix3<double> Rvalue_WB, Rdot_WB;
  extract_derivatives(R_WB.matrix(), Rvalue_WB, Rdot_WB);

  // The time derivative of the rotation matrix should be:
  //  Rdot = w× * R, with w the angular velocity.
  // Therefore we have w× = Rdot * R.transpose().
  Matrix3<double> wcross = Rdot_WB * Rvalue_WB.transpose();
  Matrix3<double> wcross_expected;
  // clang-format off
  wcross_expected << 0.0,  -wz, 0.0,
                      wz,  0.0, 0.0,
                     0.0,  0.0, 0.0;
  // clang-format on
  EXPECT_TRUE(wcross.isApprox(wcross_expected, kEpsilon));

  // Re-express inertia into another frame.
  const UnitInertia<AutoDiffXd> I_W = G_B.ReExpress(R_WB);

  // Extract value and derivatives of I_W into two separate matrices.
  Matrix3d Ivalue_W, Idot_W;
  extract_derivatives(I_W.CopyToFullMatrix3(), Ivalue_W, Idot_W);

  // Alternatively, compute the time derivative of I_W directly in terms of the
  // known angular velocity. Since G_B is diagonal with entries Iᵢ, we can
  // expand the time derivative of I_W as:
  //  dI_W/dt = d/dt(R_WB * G_B * R_WBᵀ) = d/dt(∑ Iᵢ * x̂ᵢ * x̂ᵢᵀ) =
  //          = ∑ Iᵢ * {w× * x̂ᵢ * x̂ᵢᵀ + (w× * x̂ᵢ * x̂ᵢᵀ)ᵀ}
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
}

// The code below is in support of the goal to use a unit-test to confirm that
// disallowed operators have not been introduced. This uses SFINAE template
// trickery to introduce compile-time code that can be evaluated at run-time.
// If the operator is introduced, the "failing" version of the method will be
// instantiated and invoked. Otherwise, only the passing version will be
// instantiated. There is a variation of this block of code for each disallowed
// operator.

// This overload gets chosen if the *= double would compile.
template <typename T, typename = decltype(std::declval<T&>() *= 1.)>
bool has_times_equal_helper(int) {
  return true;
}

// This overload gets chosen if the above can't compile.
// It is made to take any other argument but the above is a better match to an
// int argument if it got compiled, and therefore gets selected for a class with
// an operator*=() defined.
template <typename T>
bool has_times_equal_helper(...) {
  return false;
}

// This method returns true at runtime if type T has an operator*=().
template <typename T>
bool has_times_equal() {
  return has_times_equal_helper<T>(1);
}

// Tests that operator*=() is indeed not available for a UnitInertia while it is
// available for a general RotationalInertia.
GTEST_TEST(UnitInertia, TimesEqualScalar) {
  // While we can multiply a RotationalInertia by a scalar...
  EXPECT_TRUE(has_times_equal<RotationalInertia<double>>());

  // ... we cannot perform the same operation on a UnitInertia.
  EXPECT_FALSE(has_times_equal<UnitInertia<double>>());
}

// See the explanation for these template helpers in the analogous
// implementation for has_times_equal() above.
template <typename T, typename = decltype(std::declval<T&>() /= 1.)>
bool has_divide_equal_helper(int) {
  return true;
}
template <typename T>
bool has_divide_equal_helper(...) {
  return false;
}
template <typename T>
bool has_divide_equal() {
  return has_divide_equal_helper<T>(1);
}

// Tests that operator/=() is indeed not available for a UnitInertia while it is
// available for a general RotationalInertia.
GTEST_TEST(UnitInertia, DivideEqualScalar) {
  // While we can divide a RotationalInertia by a scalar...
  EXPECT_TRUE(has_divide_equal<RotationalInertia<double>>());

  // ... we cannot perform the same operation on a UnitInertia.
  EXPECT_FALSE(has_divide_equal<UnitInertia<double>>());
}

// See the explanation for this template helpers in the analogous implementation
// for has_times_equal() above.
template <typename T, typename = decltype(std::declval<T&>() += T())>
bool has_plus_equal_helper(int) {
  return true;
}
template <typename T>
bool has_plus_equal_helper(...) {
  return false;
}
template <typename T>
bool has_plus_equal() {
  return has_plus_equal_helper<T>(1);
}

// Tests that operator+=() is indeed not available for a UnitInertia while it is
// available for a general RotationalInertia.
GTEST_TEST(UnitInertia, PlusEqualAnInertia) {
  // While we can add a RotationalInertia to a RotationalInertia....
  EXPECT_TRUE(has_plus_equal<RotationalInertia<double>>());

  // ... we cannot perform the same operation on a UnitInertia.
  EXPECT_FALSE(has_plus_equal<UnitInertia<double>>());
}

// Verify that UnitInertia works with symbolic::Expression.
GTEST_TEST(UnitInertia, CompatibleWithSymbolicExpression) {
  const Variable r("r");
  const Variable L("L");
  // Compute the unit inertia for a cylinder oriented along the z-axis
  // (the default).
  UnitInertia<Expression> Gz =
      UnitInertia<Expression>::SolidCylinder(r, L, Vector3<double>::UnitZ());

  // Let's give the variables above some values.
  const double r_value = 0.025;
  const double L_value = 0.2;
  // And the expected principal moments.
  const double I_perp = (3.0 * r_value * r_value + L_value * L_value) / 12.0;
  const double I_axial = r_value * r_value / 2.0;
  // Create an environment in which variables have set values.
  const Environment env{{r, r_value}, {L, L_value}};

  EXPECT_NEAR(Gz(0, 0).Evaluate(env), I_perp,
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(Gz(1, 1).Evaluate(env), I_perp,
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(Gz(2, 2).Evaluate(env), I_axial,
              std::numeric_limits<double>::epsilon());
}

// This test verifies that we can robustly handle near zero inertias for point
// masses.
GTEST_TEST(UnitInertia, NearZeroInertia) {
  const double eps = std::numeric_limits<double>::epsilon();

  // Notice that, if using exact arithmetic, this inertia is not physically
  // valid given it does not satisfy the triangle inequality.
  const UnitInertia<double> G(0.0, 0.0, eps);

  // However we treat it as valid given that "catastrophic cancellation"
  // occurring when using floating point arithmetic might lead to similar
  // results when working with point inertias.
  EXPECT_TRUE(G.CouldBePhysicallyValid());
}

// With inertias for point masses, shifting to and from the center of mass (the
// actual point's position) might lead to "catastrophic cancellation" that
// triggers false negatives in CouldBePhysicallyValid(). We verify that we can
// handle this case without generating spurious false negatives due to the loss
// of significance occurring with floating point precision.
GTEST_TEST(UnitInertia, CatastrophicCancellationForPointInertias) {
  // We use a value eps smaller than machine precision in order to trigger a
  // "catastrophic cancellation", see notes below.
  const double eps = std::numeric_limits<double>::epsilon() / 2.0;

  // Unit inertia for a unit mass B located at Bo, therefore close to zero.
  const UnitInertia<double> G_BBo_W =
      UnitInertia<double>::TriaxiallySymmetric(eps);

  const Vector3d p_BoP_W = Vector3d::UnitZ();
  const Vector3d p_PBo_W = -p_BoP_W;

  // For p_BoP_W = Vector3d::UnitZ(), ShiftFromCenterOfMassInPlace() has the
  // effect of adding the inertia of a unit point mass located at P, with
  // inertia:
  //            [1 0 0]
  // G_PBcm_W = [0 1 0]
  //            [0 0 0]
  const UnitInertia<double> G_BP_W = G_BBo_W.ShiftFromCenterOfMass(p_BoP_W);

  // Therefore the new inertia will be:
  //          [1+eps     0     0]
  // G_BP_W = [    0 1+eps     0]
  //          [    0     0   eps]

  // Now ShiftToCenterOfMassInPlace() has the effect of subtracting G_PBcm_W.
  const UnitInertia<double> Gtilde_BBo_W = G_BP_W.ShiftToCenterOfMass(p_PBo_W);

  // We should be getting G_BBo_W, triaxially symmetric with eps in its
  // diagonal entries. However, due to "catastrophic cancellation", we instead
  // get the unphysical inertia:
  //                [0   0   0]
  // Gtilde_BBo_W = [0   0   0]
  //                [0   0 eps]
  // which does not satisfy the triangle inequality and therefore it is
  // unphysical.

  // What we want is to treat G_BBo_W and Gtilde_BBo_W as what they actually
  // are: the rotational inertia of a point mass B about B, and thus zero.
  // Therefore, even when "catastrophic cancellation" leads to an unphysical
  // result in exact arithmetic, we should understand it as being a zero
  // inertia. Therefore, we verify here that CouldBePhysicallyValid() correctly
  // reports a physical inertia.
  EXPECT_TRUE(Gtilde_BBo_W.CouldBePhysicallyValid());
}

GTEST_TEST(UnitInertia, ToStringFmtFormatter) {
  UnitInertia<double> I(1, 2.718, 3.14);
  std::string expected_string =
      "[    1      0      0]\n"
      "[    0  2.718      0]\n"
      "[    0      0   3.14]\n";
  EXPECT_EQ(fmt::to_string(I), expected_string);
}

}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
