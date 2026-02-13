#include "drake/multibody/tree/spatial_inertia.h"

#include <limits>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/rotational_inertia.h"
#include "drake/multibody/tree/unit_inertia.h"

namespace drake {
namespace multibody {
namespace {

using drake::math::RotationMatrixd;
using drake::math::VectorToSkewSymmetric;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using std::numeric_limits;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Makes a spatial inertia with arbitrary numerical values.
SpatialInertia<double> MakeArbitrarySpatialInertia() {
  // Parameters for a cylindrical body of a given mass, length and radius.
  const double mass = 1.2;
  const double radius = 0.05, length = 1.5;

  // Axis for a cylinder arbitrarily oriented, expressed in world W.
  const Vector3<double> axis_W = Vector3<double>(1, 2, 3).normalized();
  // Create rotational inertia for the cylinder:
  const RotationalInertia<double> I_Bcm_W =
      mass * UnitInertia<double>::SolidCylinder(radius, length, axis_W);

  // Create the spatial inertia of body B about an arbitrary point P.
  const Vector3<double> p_BcmP_W(1, -2, 5);
  return SpatialInertia<double>::MakeFromCentralInertia(mass, -p_BcmP_W,
                                                        I_Bcm_W);
}

GTEST_TEST(SpatialInertia, ZeroFactory) {
  auto I = SpatialInertia<double>::Zero();
  EXPECT_EQ(I.get_mass(), 0);
  EXPECT_EQ(I.get_com(), Vector3d::Zero());
  EXPECT_EQ(I.get_unit_inertia().get_moments(), Vector3d::Zero());
  EXPECT_EQ(I.get_unit_inertia().get_products(), Vector3d::Zero());
  EXPECT_TRUE(I.IsZero());
}

GTEST_TEST(SpatialInertia, NaNFactory) {
  auto I = SpatialInertia<double>::NaN();
  EXPECT_TRUE(I.IsNaN());
  EXPECT_FALSE(I.IsZero());
}

GTEST_TEST(SpatialInertia, MakeUnitary) {
  const double expected_mass = 1;
  const SpatialInertia<double> M = SpatialInertia<double>::MakeUnitary();
  EXPECT_TRUE(M.IsPhysicallyValid());
  EXPECT_FALSE(M.IsZero());
  EXPECT_EQ(M.get_mass(), expected_mass);
  EXPECT_EQ(M.get_com(), Vector3<double>::Zero());
  const Vector3<double> M_unit_moments = M.get_unit_inertia().get_moments();
  const Vector3<double> M_unit_products = M.get_unit_inertia().get_products();
  const double Ixx = 1;  // Expected Ixx = Iyy = Izz unit moment of inertia.
  EXPECT_EQ(M_unit_moments, Vector3<double>(Ixx, Ixx, Ixx));
  EXPECT_EQ(M_unit_products, Vector3<double>::Zero());
}

// Tests the static method for the spatial inertia of a single particle.
GTEST_TEST(SpatialInertia, PointMass) {
  // This example models a rigid body B as having all of its mass concentrated
  // at a single point Bcm (B's center of mass) and calculates B's spatial
  // inertia about an arbitrary point Bp of body B.
  const double mass = 3;
  const double lx = 1.0;
  const double ly = 2.0;
  const double lz = 3.0;
  const Vector3<double> p_BpBcm_B = Vector3<double>(lx, ly, lz);
  const UnitInertia<double> G_BBp_B = UnitInertia<double>::PointMass(p_BpBcm_B);
  const SpatialInertia<double> M_expected(mass, p_BpBcm_B, G_BBp_B);
  const SpatialInertia<double> M_BBp_B =
      SpatialInertia<double>::PointMass(mass, p_BpBcm_B);
  EXPECT_TRUE(CompareMatrices(M_expected.CopyToFullMatrix6(),
                              M_BBp_B.CopyToFullMatrix6()));

  // Verify PointMass() with a zero position vector produces the same spatial
  // inertia as shifting the spatial inertia from M_BBp_B to M_BBcm_B.
  const SpatialInertia<double> M_BBcm_B_expected = M_BBp_B.Shift(p_BpBcm_B);
  const SpatialInertia<double> M_BBcm_B =
      SpatialInertia<double>::PointMass(mass, Vector3<double>::Zero());
  EXPECT_TRUE(CompareMatrices(M_BBcm_B_expected.CopyToFullMatrix6(),
                              M_BBcm_B.CopyToFullMatrix6()));

  // Ensure a negative mass throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(SpatialInertia<double>::PointMass(-1, p_BpBcm_B),
                              "[^]* mass is not positive and finite: .*");
}

// Tests the static method for the spatial inertia of a solid box.
GTEST_TEST(SpatialInertia, SolidBoxWithDensityOrMass) {
  const double density = 1000;  // Water is 1 g/ml = 1000 kg/m³.
  const double lx = 1.0;
  const double ly = 2.0;
  const double lz = 3.0;
  const double volume = lx * ly * lz;
  const double mass = density * volume;
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  const UnitInertia<double> G_BBo_B = UnitInertia<double>::SolidBox(lx, ly, lz);
  const SpatialInertia<double> M_expected(mass, p_BoBcm_B, G_BBo_B);
  const SpatialInertia<double> M_with_density =
      SpatialInertia<double>::SolidBoxWithDensity(density, lx, ly, lz);
  EXPECT_TRUE(CompareMatrices(M_expected.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Ensure SolidBoxWithDensity() matches SolidBoxWithMass().
  const SpatialInertia<double> M_with_mass =
      SpatialInertia<double>::SolidBoxWithMass(mass, lx, ly, lz);
  EXPECT_TRUE(CompareMatrices(M_with_mass.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Ensure a negative density or mass throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidBoxWithMass(-0.1, lx, ly, lz),
      "[^]* mass is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidBoxWithDensity(-9.3, lx, ly, lz),
      "[^]* density is not positive and finite: .*.");

  // Ensure a negative or zero length, width, or height throws an exception.
  // There is not an exhaustive test of each parameter being zero or negative.
  // Instead, each parameter is tested with a single bad value and we assume a
  // single value sufficiently tests the full domain of invalid values.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidBoxWithDensity(density, 0, ly, lz),
      "[^]* x-length is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidBoxWithMass(mass, 0, ly, lz),
      "[^]* x-length is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidBoxWithDensity(density, lx, -0.1, lz),
      "[^]* y-length is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidBoxWithMass(mass, lx, -0.1, lz),
      "[^]* y-length is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidBoxWithDensity(density, lx, ly, -1E-15),
      "[^]* z-length is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidBoxWithMass(mass, lx, ly, -1E-15),
      "[^]* z-length is not positive and finite: .*.");
}

// Tests the static method for the spatial inertia of a solid cube.
GTEST_TEST(SpatialInertia, SolidCubeWithDensity) {
  const double density = 1000;  // Water is 1 g/ml = 1000 kg/m³.
  const double length = 2.0;
  const double volume = length * length * length;
  const double mass = density * volume;
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  const UnitInertia<double> G_BBo_B = UnitInertia<double>::SolidCube(length);
  const SpatialInertia<double> M_expected(mass, p_BoBcm_B, G_BBo_B);
  const SpatialInertia<double> M_with_density =
      SpatialInertia<double>::SolidCubeWithDensity(density, length);
  EXPECT_TRUE(CompareMatrices(M_expected.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Also test against a solid box with length = width = height.
  const SpatialInertia<double> Mbox =
      SpatialInertia<double>::SolidBoxWithDensity(density, length, length,
                                                  length);
  EXPECT_TRUE(CompareMatrices(Mbox.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Ensure SolidCubeWithDensity() matches SolidCubeWithMass().
  const SpatialInertia<double> M_with_mass =
      SpatialInertia<double>::SolidCubeWithMass(mass, length);
  EXPECT_TRUE(CompareMatrices(M_with_mass.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Ensure a negative density or mass throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCubeWithDensity(-9.3, length),
      "[^]* density is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCubeWithMass(-0.1, length),
      "[^]* mass is not positive and finite: .*.");

  // Ensure a negative or zero length throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCubeWithDensity(density, 0),
      "[^]* length is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCubeWithMass(mass, 0),
      "[^]* length is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCubeWithDensity(density, -1),
      "[^]* length is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCubeWithMass(mass, -1),
      "[^]* length is not positive and finite: .*.");
}

// Tests the static method for the spatial inertia of a solid capsule.
GTEST_TEST(SpatialInertia, SolidCapsuleWithDensityOrMass) {
  const double density = 1000;  // Water is 1 g/ml = 1000 kg/m³.
  const double r = 1.0;
  const double l = 2.0;
  const double volume = 4.0 / 3.0 * M_PI * std::pow(r, 3) + M_PI * r * r * l;
  const double mass = density * volume;
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();

  // Test a solid capsule B whose unit_vector is in the z-direction.
  Vector3<double> unit_vec(0, 0, 1);
  UnitInertia<double> G_BBo_B =
      UnitInertia<double>::SolidCapsule(r, l, unit_vec);
  SpatialInertia<double> M_expected(mass, p_BoBcm_B, G_BBo_B);
  SpatialInertia<double> M_with_density =
      SpatialInertia<double>::SolidCapsuleWithDensity(density, r, l, unit_vec);
  EXPECT_TRUE(CompareMatrices(M_expected.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Test a solid capsule B with a different unit vector direction.
  unit_vec = Vector3<double>(0.5, -0.5, 1.0 / std::sqrt(2));
  G_BBo_B = UnitInertia<double>::SolidCapsule(r, l, unit_vec);
  M_expected = SpatialInertia<double>(mass, p_BoBcm_B, G_BBo_B);
  M_with_density =
      SpatialInertia<double>::SolidCapsuleWithDensity(density, r, l, unit_vec);
  EXPECT_TRUE(CompareMatrices(M_expected.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Ensure SolidCapsuleWithDensity() matches SolidCapsuleWithMass().
  const SpatialInertia<double> M_with_mass =
      SpatialInertia<double>::SolidCapsuleWithMass(mass, r, l, unit_vec);
  EXPECT_TRUE(CompareMatrices(M_with_mass.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Ensure a negative density or mass throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCapsuleWithDensity(-9.3, r, l, unit_vec),
      "[^]* density is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCapsuleWithMass(-9.3, r, l, unit_vec),
      "[^]* mass is not positive and finite: .*.");

  // Ensure a negative or zero radius or length throws an exception.
  // There is not an exhaustive test of each parameter being zero or negative.
  // Instead, each parameter is tested with a single bad value and we assume a
  // single value sufficiently tests the full domain of invalid values.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCapsuleWithDensity(density, 0, l, unit_vec),
      "[^]* radius is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCapsuleWithMass(mass, 0, l, unit_vec),
      "[^]* radius is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCapsuleWithDensity(density, r, -2, unit_vec),
      "[^]* length is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCapsuleWithMass(mass, r, -2, unit_vec),
      "[^]* length is not positive and finite: .*.");
}

// Tests the static method for the spatial inertia of a solid cylinder.
GTEST_TEST(SpatialInertia, SolidCylinderWithDensityOrMass) {
  const double density = 1000;  // Water is 1 g/ml = 1000 kg/m³.
  const double r = 1.0;         // radius
  const double l = 2.0;         // length
  const double volume = M_PI * r * r * l;
  const double mass = density * volume;
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();

  // Test a solid cylinder B about Bcm whose axis is aligned with unit_vec.
  const Vector3<double> unit_vec(0.5, -0.5, 1.0 / std::sqrt(2));
  UnitInertia<double> G_BBo_B =
      UnitInertia<double>::SolidCylinder(r, l, unit_vec);
  SpatialInertia<double> M_expected(mass, p_BoBcm_B, G_BBo_B);
  SpatialInertia<double> M_with_density =
      SpatialInertia<double>::SolidCylinderWithDensity(density, r, l, unit_vec);
  EXPECT_TRUE(CompareMatrices(M_expected.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Ensure SolidCylinderWithMass() matches SolidCylinderWithDensity().
  SpatialInertia<double> M_with_mass =
      SpatialInertia<double>::SolidCylinderWithMass(mass, r, l, unit_vec);
  EXPECT_TRUE(CompareMatrices(M_with_mass.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Test a solid cylinder B about the point Bp, where Bp is at the center of a
  // circular end of the cylinder with the following position from Bp to Bcm.
  const Vector3<double> p_BpBcm_B = 0.5 * l * unit_vec;
  const UnitInertia<double> G_BBp_B =
      UnitInertia<double>::SolidCylinderAboutEnd(r, l, unit_vec);
  M_expected = SpatialInertia<double>(mass, p_BpBcm_B, G_BBp_B);
  M_with_density = SpatialInertia<double>::SolidCylinderWithDensityAboutEnd(
      density, r, l, unit_vec);
  EXPECT_TRUE(CompareMatrices(M_expected.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Match SolidCylinderWithMassAboutEnd() & SolidCylinderWithDensityAboutEnd().
  M_with_mass = SpatialInertia<double>::SolidCylinderWithMassAboutEnd(
      mass, r, l, unit_vec);
  EXPECT_TRUE(CompareMatrices(M_with_mass.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Also test a solid cylinder B about Bp using Shift() function.
  const SpatialInertia<double> M_BBcm_B =
      SpatialInertia<double>::SolidCylinderWithDensity(density, r, l, unit_vec);
  const Vector3<double> p_BcmBp_B = -p_BpBcm_B;
  M_expected = M_BBcm_B.Shift(p_BcmBp_B);

  // The algorithms to calculate M_expected and M_with_density differ, so
  // use an empirical tolerance of two bits = 2^2 times machine epsilon.
  const double kTolerance = 4 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(M_expected.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6(), kTolerance,
                              MatrixCompareType::relative));

  // Ensure a negative density or mass throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithDensity(-9.3, r, l, unit_vec),
      "[^]* density is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithMass(-9.3, r, l, unit_vec),
      "[^]* mass is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithDensityAboutEnd(-9.3, r, l,
                                                               unit_vec),
      "[^]* density is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithMassAboutEnd(-9.3, r, l,
                                                            unit_vec),
      "[^]* mass is not positive and finite: .*.");

  // Ensure a negative or zero radius throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithDensity(density, 0, l, unit_vec),
      "[^]* radius is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithMass(mass, 0, l, unit_vec),
      "[^]* radius is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithDensityAboutEnd(density, -0.1, l,
                                                               unit_vec),
      "[^]* radius is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithMassAboutEnd(mass, -0.1, l,
                                                            unit_vec),
      "[^]* radius is not positive and finite: .*.");

  // Ensure a negative or zero length throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithDensity(density, r, 0, unit_vec),
      "[^]* length is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithMass(mass, r, 0, unit_vec),
      "[^]* length is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithDensityAboutEnd(density, r, -0.1,
                                                               unit_vec),
      "[^]* length is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithMassAboutEnd(density, r, -0.1,
                                                            unit_vec),
      "[^]* length is not positive and finite: .*.");

  // Ensure a bad unit vector throws an exception.
  const Vector3<double> bad_vec(1, 0.1, 0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithDensity(density, r, l, bad_vec),
      "[^]* The unit_vector argument .* is not a unit vector.[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithMass(mass, r, l, bad_vec),
      "[^]* The unit_vector argument .* is not a unit vector.[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithDensityAboutEnd(density, r, l,
                                                               bad_vec),
      "[^]* The unit_vector argument .* is not a unit vector.[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidCylinderWithMassAboutEnd(mass, r, l,
                                                            bad_vec),
      "[^]* The unit_vector argument .* is not a unit vector.[^]*");
}

// Tests the static method for the spatial inertia of a thin rod.
GTEST_TEST(SpatialInertia, ThinRodWithMass) {
  const double mass = 1.2;  // units of kg.
  const double length = 2.0;
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();

  // Test a thin rod B whose unit_vector is in the z-direction.
  Vector3<double> unit_vec(0, 0, 1);
  UnitInertia<double> G_BBcm_B = UnitInertia<double>::ThinRod(length, unit_vec);
  SpatialInertia<double> M_expected(mass, p_BoBcm_B, G_BBcm_B);
  SpatialInertia<double> M =
      SpatialInertia<double>::ThinRodWithMass(mass, length, unit_vec);
  // Use an empirical tolerance of two bits = 2^2 times machine epsilon.
  const double kTolerance = 4 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(M_expected.CopyToFullMatrix6(),
                              M.CopyToFullMatrix6(), kTolerance));

  // Test a thin rod B with a different and less simple unit vector direction.
  unit_vec = Vector3<double>(0.5, -0.5, 1.0 / std::sqrt(2));
  G_BBcm_B = UnitInertia<double>::ThinRod(length, unit_vec);
  M_expected = SpatialInertia<double>(mass, p_BoBcm_B, G_BBcm_B);
  M = SpatialInertia<double>::ThinRodWithMass(mass, length, unit_vec);
  EXPECT_TRUE(CompareMatrices(M_expected.CopyToFullMatrix6(),
                              M.CopyToFullMatrix6(), kTolerance));

  // Test a thin rod B about Bp, where Bp is at an end of the rod.
  const Vector3<double> p_BpBcm_B = 0.5 * length * unit_vec;
  UnitInertia<double> G_BBp_B = G_BBcm_B.ShiftFromCenterOfMass(-p_BpBcm_B);
  M_expected = SpatialInertia<double>(mass, p_BpBcm_B, G_BBp_B);
  M = SpatialInertia<double>::ThinRodWithMassAboutEnd(mass, length, unit_vec);
  EXPECT_TRUE(CompareMatrices(M_expected.CopyToFullMatrix6(),
                              M.CopyToFullMatrix6(), kTolerance));

  // Another way to perform the previous calculation and test.
  const SpatialInertia<double> M_BBp_B =
      SpatialInertia<double>::MakeFromCentralInertia(mass, p_BpBcm_B,
                                                     mass * G_BBcm_B);
  EXPECT_TRUE(CompareMatrices(M_BBp_B.CopyToFullMatrix6(),
                              M.CopyToFullMatrix6(), kTolerance));

  // Ensure a negative or zero mass throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::ThinRodWithMass(-1.23, length, unit_vec),
      "[^]* mass is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::ThinRodWithMass(0, length, unit_vec),
      "[^]* mass is not positive and finite: .*.");

  // Ensure a negative or zero length throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::ThinRodWithMass(mass, -4.56, unit_vec),
      "[^]* length is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::ThinRodWithMass(mass, 0, unit_vec),
      "[^]* length is not positive and finite: .*.");

  // Ensure a bad unit vector throws an exception.
  const Vector3<double> bad_vec(1, 0.1, 0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::ThinRodWithMass(mass, length, bad_vec),
      "[^]* The unit_vector argument .* is not a unit vector.[^]*");
}

// Tests the static method for the spatial inertia of a solid ellipsoid.
GTEST_TEST(SpatialInertia, SolidEllipsoidWithDensityOrMass) {
  const double density = 1000;  // Water is 1 g/ml = 1000 kg/m³.
  const double a = 0.2;
  const double b = 0.3;
  const double c = 0.4;
  const double volume = 4.0 / 3.0 * M_PI * a * b * c;
  const double mass = density * volume;
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  UnitInertia<double> G_BBo_B = UnitInertia<double>::SolidEllipsoid(a, b, c);
  SpatialInertia<double> M_expected(mass, p_BoBcm_B, G_BBo_B);
  SpatialInertia<double> M_with_density =
      SpatialInertia<double>::SolidEllipsoidWithDensity(density, a, b, c);
  EXPECT_TRUE(CompareMatrices(M_expected.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Ensure SolidEllipsoidWithDensity() matches SolidEllipsoidWithMass().
  const SpatialInertia<double> M_with_mass =
      SpatialInertia<double>::SolidEllipsoidWithMass(mass, a, b, c);
  EXPECT_TRUE(CompareMatrices(M_with_mass.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Ensure a negative density or mass throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidEllipsoidWithDensity(-9.3, a, b, c),
      "[^]* density is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidEllipsoidWithMass(-9.3, a, b, c),
      "[^]* mass is not positive and finite: .*.");

  // Ensure a negative or zero semi-axis length throws an exception.
  // There is not an exhaustive test of each parameter being zero or negative.
  // Instead, each parameter is tested with a single bad value and we assume a
  // single value sufficiently tests the full domain of invalid values.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidEllipsoidWithDensity(density, 0, b, c),
      "[^]* semi-axis a is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidEllipsoidWithMass(mass, 0, b, c),
      "[^]* semi-axis a is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidEllipsoidWithDensity(density, a, -2, c),
      "[^]* semi-axis b is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidEllipsoidWithMass(mass, a, -2, c),
      "[^]* semi-axis b is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidEllipsoidWithDensity(density, a, b, -0.01),
      "[^]* semi-axis c is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidEllipsoidWithMass(mass, a, b, -0.01),
      "[^]* semi-axis c is not positive and finite: .*.");
}

// Tests the static method for the spatial inertia of a solid sphere.
GTEST_TEST(SpatialInertia, SolidSphereWithDensityOrMass) {
  const double density = 1000;  // Water is 1 g/ml = 1000 kg/m³.
  const double radius = 0.2;
  const double volume = 4.0 / 3.0 * M_PI * std::pow(radius, 3);  // 4/3 π r³
  const double mass = density * volume;
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  const UnitInertia<double> G_BBo_B = UnitInertia<double>::SolidSphere(radius);
  const SpatialInertia<double> M_expected(mass, p_BoBcm_B, G_BBo_B);
  const SpatialInertia<double> M_with_density =
      SpatialInertia<double>::SolidSphereWithDensity(density, radius);
  EXPECT_TRUE(CompareMatrices(M_expected.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Ensure SolidSphereWithDensity() matches SolidSphereWithMass().
  const SpatialInertia<double> M_with_mass =
      SpatialInertia<double>::SolidSphereWithMass(mass, radius);
  EXPECT_TRUE(CompareMatrices(M_with_mass.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Ensure a negative mass or density throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidSphereWithMass(-9.3, radius),
      "[^]* mass is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidSphereWithDensity(-9.3, radius),
      "[^]* density is not positive and finite: .*.");

  // Ensure a negative or zero radius throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidSphereWithMass(mass, 0),
      "[^]* radius is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidSphereWithMass(mass, -0.2),
      "[^]* radius is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidSphereWithDensity(density, 0),
      "[^]* radius is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidSphereWithDensity(density, -0.2),
      "[^]* radius is not positive and finite: .*.");
}

// Tests the static method for the spatial inertia of a thin hollow sphere.
GTEST_TEST(SpatialInertia, HollowSphereWithDensityOrMass) {
  const double area_density = 80;  // density per unit area is 80 kg/m².
  const double radius = 0.2;
  const double surface_area = 4.0 * M_PI * std::pow(radius, 2);  // 4 π r²
  const double mass = area_density * surface_area;
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  const UnitInertia<double> G_BBo_B = UnitInertia<double>::HollowSphere(radius);
  const SpatialInertia<double> M_expected(mass, p_BoBcm_B, G_BBo_B);
  const SpatialInertia<double> M_with_density =
      SpatialInertia<double>::HollowSphereWithDensity(area_density, radius);
  EXPECT_TRUE(CompareMatrices(M_expected.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Ensure HollowSphereWithDensity() matches HollowSphereWithMass().
  const SpatialInertia<double> M_with_mass =
      SpatialInertia<double>::HollowSphereWithMass(mass, radius);
  EXPECT_TRUE(CompareMatrices(M_with_mass.CopyToFullMatrix6(),
                              M_with_density.CopyToFullMatrix6()));

  // Ensure a negative density or mass throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::HollowSphereWithMass(-9.3, radius),
      "[^]* mass is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::HollowSphereWithDensity(-9.3, radius),
      "[^]* area_density is not positive and finite: .*.");

  // Ensure a negative or zero radius throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::HollowSphereWithMass(mass, 0),
      "[^]* radius is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::HollowSphereWithMass(mass, -0.2),
      "[^]* radius is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::HollowSphereWithDensity(area_density, 0),
      "[^]* radius is not positive and finite: .*.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::HollowSphereWithDensity(area_density, -0.2),
      "[^]* radius is not positive and finite: .*.");
}

// Test spatial inertia of a solid tetrahedron B about its vertex B0.
GTEST_TEST(SpatialInertia, SolidTetrahedronAboutVertex) {
  const double density = 0.12345;
  const Vector3<double> p1(1, 0, 0);  // Position vector from B0 to vertex B1.
  const Vector3<double> p2(0, 2, 0);  // Position vector from B0 to vertex B2.
  const Vector3<double> p3(0, 0, 3);  // Position vector from B0 to vertex B3.

  const double volume = 1.0 / 6.0 * p1.cross(p2).dot(p3);
  const double mass = density * volume;
  const Vector3<double> p_B0Bcm = 0.25 * (p1 + p2 + p3);
  const UnitInertia<double> G_BB0 =
      UnitInertia<double>::SolidTetrahedronAboutVertex(p1, p2, p3);
  const SpatialInertia<double> M_BB0_expected(mass, p_B0Bcm, G_BB0);
  SpatialInertia<double> M_BB0 =
      SpatialInertia<double>::SolidTetrahedronAboutVertexWithDensity(
          density, p1, p2, p3);

  // An empirical tolerance: two bits = 2^2 times machine epsilon.
  const double kTolerance = 4 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(M_BB0_expected.CopyToFullMatrix6(),
                              M_BB0.CopyToFullMatrix6(), kTolerance));

  // Ensure nothing changes if two arguments are switched (e.g., p1 and p2).
  M_BB0 = SpatialInertia<double>::SolidTetrahedronAboutVertexWithDensity(
      density, p2, p1, p3);
  EXPECT_TRUE(CompareMatrices(M_BB0_expected.CopyToFullMatrix6(),
                              M_BB0.CopyToFullMatrix6(), kTolerance));

  // Ensure a negative density throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidTetrahedronAboutVertexWithDensity(-9.3, p1,
                                                                     p2, p3),
      "[^]* density is not positive and finite: .*.");
}

// Test spatial inertia of a solid tetrahedron about an arbitrary point A.
GTEST_TEST(SpatialInertia, SolidTetrahedronAboutPoint) {
  const double density = 0.54321;
  Vector3<double> p_AB0(0, 0, 0);
  Vector3<double> p_AB1(1, 1, 0);
  Vector3<double> p_AB2(0, 2, 0);
  Vector3<double> p_AB3(0, 3, 3);

  // Do a sanity check that SolidTetrahedronAboutPointWithDensity() simplifies
  // to SolidTetrahedronAboutVertexWithDensity() when p_AB0 is the zero vector.
  SpatialInertia<double> M_BA_expected =
      SpatialInertia<double>::SolidTetrahedronAboutVertexWithDensity(
          density, p_AB1, p_AB2, p_AB3);
  SpatialInertia<double> M_BA =
      SpatialInertia<double>::SolidTetrahedronAboutPointWithDensity(
          density, p_AB0, p_AB1, p_AB2, p_AB3);

  // An empirical tolerance: two bits = 2^2 times machine epsilon.
  const double kTolerance = 4 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(M_BA_expected.CopyToFullMatrix6(),
                              M_BA.CopyToFullMatrix6(), kTolerance));

  // Check a more general case in which p_AB0 is a non-zero vector.
  p_AB0 = Vector3<double>(1, 4, 5);
  p_AB1 += p_AB0;
  p_AB2 += p_AB0;
  p_AB3 += p_AB0;
  M_BA_expected.ShiftInPlace(-p_AB0);
  M_BA = SpatialInertia<double>::SolidTetrahedronAboutPointWithDensity(
      density, p_AB0, p_AB1, p_AB2, p_AB3);
  EXPECT_TRUE(CompareMatrices(M_BA_expected.CopyToFullMatrix6(),
                              M_BA.CopyToFullMatrix6(), kTolerance));

  // Ensure nothing changes if two arguments are switched (e.g., p_AB2, p_AB3).
  M_BA = SpatialInertia<double>::SolidTetrahedronAboutPointWithDensity(
      density, p_AB0, p_AB1, p_AB3, p_AB2);
  EXPECT_TRUE(CompareMatrices(M_BA_expected.CopyToFullMatrix6(),
                              M_BA.CopyToFullMatrix6(), kTolerance));

  // Ensure a negative density throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::SolidTetrahedronAboutPointWithDensity(
          -9.3, p_AB0, p_AB1, p_AB3, p_AB2),
      "[^]* density is not positive and finite: .*.");
}

// Test the construction from the mass, center of mass, and unit inertia of a
// body. Also tests:
//   - Getters.
//   - CopyToFullMatrix6().
//   - SetNan()
GTEST_TEST(SpatialInertia, ConstructionFromMassCmAndUnitInertia) {
  const double mass = 2.5;
  const Vector3d com(0.1, -0.2, 0.3);
  const Vector3d m(2.0, 2.3, 2.4);         // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);        // p for products.
  UnitInertia<double> G(m(0), m(1), m(2),  /* moments of inertia */
                        p(0), p(1), p(2)); /* products of inertia */
  SpatialInertia<double> M(mass, com, G);
  ASSERT_TRUE(M.IsPhysicallyValid());

  ASSERT_EQ(M.get_mass(), mass);
  ASSERT_EQ(M.get_com(), com);
  // Asserts we can retrieve the original unit inertia.
  ASSERT_TRUE(M.get_unit_inertia().CopyToFullMatrix3().isApprox(
      G.CopyToFullMatrix3(), kEpsilon));

  Matrix6<double> Mmatrix = M.CopyToFullMatrix6();
  Matrix6<double> expected_matrix;
  expected_matrix.block<3, 3>(0, 0) = mass * G.CopyToFullMatrix3();
  expected_matrix.block<3, 3>(3, 3) = mass * Matrix3d::Identity();
  expected_matrix.block<3, 3>(0, 3) = mass * VectorToSkewSymmetric(com);
  expected_matrix.block<3, 3>(3, 0) =
      expected_matrix.block<3, 3>(0, 3).transpose();

  EXPECT_TRUE(Mmatrix.isApprox(expected_matrix, kEpsilon));

  EXPECT_FALSE(M.IsNaN());
  M.SetNaN();
  EXPECT_TRUE(M.IsNaN());
}

// Tests that we can correctly cast a SpatialInertia<double> to a
// SpatialInertia<AutoDiffXd>.
// The cast from a SpatialInertia<double>, a constant, results in a spatial
// inertia with zero gradients. Since we are using AutoDiffXd with dynamic size
// derivatives(), this results in gradient vectors with zero size.
GTEST_TEST(SpatialInertia, CastToAutoDiff) {
  const double mass_double = 2.5;
  const Vector3d com_double(0.1, -0.2, 0.3);
  const Vector3d m_double(2.0, 2.3, 2.4);   // m for moments.
  const Vector3d p_double(0.1, -0.1, 0.2);  // p for products.
  const UnitInertia<double> G_double(
      m_double(0), m_double(1), m_double(2),  /* moments of inertia */
      p_double(0), p_double(1), p_double(2)); /* products of inertia */
  const SpatialInertia<double> M_double(mass_double, com_double, G_double);
  ASSERT_TRUE(M_double.IsPhysicallyValid());

  // Cast from double to AutoDiffXd.
  SpatialInertia<AutoDiffXd> M_autodiff = M_double.cast<AutoDiffXd>();

  // Verify values and gradients of M_autodiff.
  // Since there are no independent variables in this case the size of all
  // gradients must be zero.

  // Value and gradient of the mass.
  const auto& mass_autodiff = M_autodiff.get_mass();
  const double mass_value = mass_autodiff.value();
  EXPECT_NEAR(mass_value, mass_double, kEpsilon);
  const auto& mass_gradient = mass_autodiff.derivatives();
  ASSERT_EQ(mass_gradient.size(), 0);

  // Values and gradients of the com vector.
  const auto& G_autodiff = M_autodiff.get_unit_inertia();
  const Matrix3<AutoDiffXd> G_autodiff_matrix = G_autodiff.CopyToFullMatrix3();
  auto G_value = math::ExtractValue(G_autodiff_matrix);
  G_value.resize(3, 3);
  EXPECT_TRUE(G_value.isApprox(G_double.CopyToFullMatrix3(), kEpsilon));
  MatrixXd G_gradient = math::ExtractGradient(G_autodiff_matrix);
  ASSERT_EQ(G_gradient.size(), 0);

  // Values and gradients of the unit inertia.
  const auto& com_autodiff = M_autodiff.get_com();
  const Vector3d com_value = math::ExtractValue(com_autodiff);
  EXPECT_TRUE(com_value.isApprox(com_double, kEpsilon));
  MatrixXd com_gradient = math::ExtractGradient(com_autodiff);
  ASSERT_EQ(com_gradient.size(), 0);
}

// TODO(2026-06-01): delete test ShiftOperator.
// Test the shift operator to write into a stream.
GTEST_TEST(SpatialInertia, ShiftOperator) {
  const double mass = 2.5;
  const Vector3d com(0.1, -0.2, 0.3);
  const Vector3d m(2.0, 2.3, 2.4);         // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);        // p for products.
  UnitInertia<double> G(m(0), m(1), m(2),  /* moments of inertia */
                        p(0), p(1), p(2)); /* products of inertia */
  SpatialInertia<double> M(mass, com, G);

  std::stringstream stream;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  stream << M;
#pragma GCC diagnostic pop
  std::string expected_string =
      "\n"
      " mass = 2.5\n"
      " Center of mass = [0.1  -0.2  0.3]\n"
      " Inertia about point P, I_BP =\n"
      "[    5   0.25  -0.25]\n"
      "[ 0.25   5.75    0.5]\n"
      "[-0.25    0.5      6]\n";
  EXPECT_EQ(expected_string, stream.str());
}

// Verify the output string from SpatialInertia's fmt formatter.
GTEST_TEST(SpatialInertia, ToStringFmtFormatter) {
  const double mass = 2.5;
  const Vector3d com(0.1, -0.2, 0.3);
  const Vector3d m(2.0, 2.3, 2.4);         // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);        // p for products.
  UnitInertia<double> G(m(0), m(1), m(2),  /* moments of inertia */
                        p(0), p(1), p(2)); /* products of inertia */
  SpatialInertia<double> M(mass, com, G);
  std::string expected_string =
      "\n"
      " mass = 2.5\n"
      " Center of mass = [0.1  -0.2  0.3]\n"
      " Inertia about point P, I_BP =\n"
      "[    5   0.25  -0.25]\n"
      "[ 0.25   5.75    0.5]\n"
      "[-0.25    0.5      6]\n";
  EXPECT_EQ(fmt::to_string(M), expected_string);
}

// Verifies the correctness of:
// - operator+=()
// - ShiftInPlace()
GTEST_TEST(SpatialInertia, PlusEqualOperator) {
  const double L = 2.0;  // Length of the two cubes below (left and right).
  // Spatial inertia computed about the origin for a cube with sides of
  // length L centered at x = 1.0. Expressed in world frame W.
  const double mass_right = 1.5;  // Mass of the cube on the right.
  // So far the "about point" is the cube's centroid.
  // We'll shift the about point below.
  SpatialInertia<double> MRightBox_Wo_W(mass_right, Vector3d::Zero(),
                                        UnitInertia<double>::SolidCube(L));
  MRightBox_Wo_W.ShiftInPlace(-Vector3d::UnitX());
  // Check if after transformation this still is a physically valid inertia.
  EXPECT_TRUE(MRightBox_Wo_W.IsPhysicallyValid());

  // Spatial inertia computed about the origin for a cube with sides of
  // length L centered at x = -1.0. Expressed in world frame W.
  const double mass_left = 0.5;  // Mass of the cube on the left.
  // So far the "about point" is the cube's centroid.
  // We'll shift the about point below.
  SpatialInertia<double> MLeftBox_Wo_W(mass_left, Vector3d::Zero(),
                                       UnitInertia<double>::SolidCube(L));
  MLeftBox_Wo_W.ShiftInPlace(Vector3d::UnitX());
  // Check if after transformation this still is a physically valid inertia.
  EXPECT_TRUE(MLeftBox_Wo_W.IsPhysicallyValid());

  // The result of adding the spatial inertia of two bodies is the spatial
  // inertia of the combined system of the two bodies as if they were welded
  // together. For this unit tests, the bodies are two cubes side to side. One
  // is located to the left of the world's origin Wo at x = -1.0 and the second
  // one is located to the right of Wo at x = 1.0. Therefore adding these two
  // spatial inertia objects results in a combined body with the shape of a box
  // of length 2 * L and squared parallel bases of size L x L.
  // Notice that the about point Wo and the expressed-in frame W is the same as
  // in the two individual components.
  SpatialInertia<double> MBox_Wo_W(MLeftBox_Wo_W);
  MBox_Wo_W += MRightBox_Wo_W;
  EXPECT_TRUE(MBox_Wo_W.IsPhysicallyValid());

  // Check that the compound inertia corresponds to that of a larger box of
  // length 4.0.
  // We compute here in `com` the center of mass of the combined system
  // consisting of the two boxes.
  const double mass = mass_left + mass_right;
  const Vector3d com((mass_left * MLeftBox_Wo_W.get_com() +
                      mass_right * MRightBox_Wo_W.get_com()) /
                     mass);
  SpatialInertia<double> MExpected_Wo_W(
      mass, com, UnitInertia<double>::SolidBox(2 * L, L, L));
  EXPECT_TRUE(MBox_Wo_W.CopyToFullMatrix6().isApprox(
      MExpected_Wo_W.CopyToFullMatrix6(), kEpsilon));
}

// Tests the method SpatialInertia::ReExpress().
GTEST_TEST(SpatialInertia, ReExpress) {
  // Spatial inertia for a cube C computed about a point P and expressed in a
  // frame E.
  const double Lx = 0.2, Ly = 1.0, Lz = 0.5;  // Cube's lengths.
  const double mass = 1.3;                    // Cube's mass
  SpatialInertia<double> M_CP_E(  // First computed about its centroid.
      mass, Vector3d::Zero(), UnitInertia<double>::SolidBox(Lx, Ly, Lz));
  // Shift to point P placed one unit in the y direction along the y axis in
  // frame E.
  M_CP_E.ShiftInPlace(Vector3d::UnitY());

  // Place B rotated +90 degrees about W's x-axis.
  const RotationMatrixd R_WE = RotationMatrixd::MakeXRotation(M_PI_2);

  SpatialInertia<double> M_CP_W = M_CP_E.ReExpress(R_WE);

  // Checks for physically correct spatial inertia.
  EXPECT_TRUE(M_CP_W.IsPhysicallyValid());

  // The mass is invariant when re-expressing in another frame.
  EXPECT_EQ(M_CP_E.get_mass(), M_CP_W.get_mass());

  // The vector p_PCcm changes when re-expressed in another frame from
  // p_PCcm_E = [0, -1, 0] to p_PCcm_W = [0, 0, -1]
  EXPECT_TRUE(M_CP_W.get_com().isApprox(-Vector3d::UnitZ(), kEpsilon));

  Vector3d moments_E = M_CP_E.get_unit_inertia().get_moments();
  Vector3d moments_W = M_CP_W.get_unit_inertia().get_moments();
  // Since rotation is along the x-axis the first moment about x does
  // not change.
  EXPECT_NEAR(moments_W(0), moments_E(0), kEpsilon);

  // The y and z moments swap places after the rotation of 90 degrees about the
  // x axis.
  EXPECT_NEAR(moments_W(1), moments_E(2), kEpsilon);
  EXPECT_NEAR(moments_W(2), moments_E(1), kEpsilon);
}

// Unit tests the parallel axis theorem shift. The test computes the moment of
// inertia for a cylinder computed about its center of mass and shifted to a
// point at its base. The result is compared to the expected value.
GTEST_TEST(SpatialInertia, Shift) {
  // This defines the orientation of a frame B to be rotated +90 degrees about
  // the x-axis of a W frame.
  const RotationMatrixd R_WB = RotationMatrixd::MakeXRotation(M_PI_2);

  // Spatial inertia for a thin cylinder of computed about its center of
  // mass and expressed in frame W.
  const double mass = 1.2;
  const double radius = 0.05, length = 1.5;
  // First define it in frame B.
  SpatialInertia<double> M_BBcm_W(
      mass, Vector3d::Zero(),
      UnitInertia<double>::SolidCylinder(radius, length, Vector3d::UnitZ()));
  // Then re-express in frame W.
  M_BBcm_W.ReExpressInPlace(R_WB);

  // Vector from Bcm to the top of the cylinder Btop.
  Vector3d p_BcmBtop_W(0, length / 2.0, 0);

  // Computes spatial inertia about Btop, still expressed in W.
  SpatialInertia<double> M_BBtop_W = M_BBcm_W.Shift(p_BcmBtop_W);

  // Checks for physically correct spatial inertia.
  EXPECT_TRUE(M_BBtop_W.IsPhysicallyValid());

  // Shift() does not change the mass.
  EXPECT_EQ(M_BBtop_W.get_mass(), M_BBcm_W.get_mass());

  // The position vector from Bcm was zero by definition.
  // It is not zero from Btop but -p_BcmBtop_W.
  EXPECT_EQ(M_BBtop_W.get_com(), -p_BcmBtop_W);

  // Expected moment of inertia for a rod when computed about one of its ends.
  const double I_end =
      mass * (3 * radius * radius + length * length) / 12 /*About centroid.*/
      + mass * length * length / 4; /*Parallel axis theorem shift.*/
  const auto I_Xo_W = M_BBtop_W.CalcRotationalInertia();
  EXPECT_NEAR(I_Xo_W(0, 0), I_end, kEpsilon);
  EXPECT_NEAR(I_Xo_W(2, 2), I_end, kEpsilon);

  // Now check that shifting back to the COM results in the same spatial
  // inertia.
  SpatialInertia<double> M_BBcm_W_back = M_BBtop_W.Shift(-p_BcmBtop_W);
  EXPECT_TRUE(M_BBcm_W_back.CopyToFullMatrix6().isApprox(
      M_BBcm_W.CopyToFullMatrix6(), kEpsilon));
}

// Tests that it is possible to create a spatial inertia with zero mass.
GTEST_TEST(SpatialInertia, IsPhysicallyValidWithZeroMass) {
  // The current behavior of Drake is to allow a spatial inertia to have a mass
  // of zero, whether or not p_PBcm is a zero vector.
  const double mass_zero = 0.0;
  Vector3d p_PBcm(0.0, 0.0, 0.0);
  const UnitInertia<double> G = UnitInertia<double>::SolidSphere(1.0);
  const SpatialInertia<double> M0(mass_zero, p_PBcm, G);
  EXPECT_TRUE(M0.IsPhysicallyValid());

  p_PBcm = Vector3d(1.0, 2.0, 3.0);
  const SpatialInertia<double> M1(mass_zero, p_PBcm, G);
  EXPECT_TRUE(M1.IsPhysicallyValid());

  // Test when an attempt to form UnitInertia with a divide-by-zero problem.
  p_PBcm = Vector3d(0.0, 0.0, 0.0);
  const RotationalInertia<double> I(2, 3, 4);
  const std::string expected_message = ".*condition 'mass > 0' failed.";
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::MakeFromCentralInertia(mass_zero, p_PBcm, I),
      expected_message);

  p_PBcm = Vector3d(1.0, 2.0, 3.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::MakeFromCentralInertia(mass_zero, p_PBcm, I),
      expected_message);
}

// Test that it is not possible to create a spatial inertia with a negative
// or NaN mass or a bad position vector.
GTEST_TEST(SpatialInertia, IsInvalidDueToBadMassOrPositionVector) {
  Vector3<double> p_PBcm_E = Vector3<double>(0.0, 0.0, 0.0);
  const double Gxx = 2.0, Gyy = 3.0, Gzz = 4.0;
  const UnitInertia<double> G_BP_E(Gxx, Gyy, Gzz, 0, 0, 0);

  // Ensure a negative mass throws an exception.
  double mass = -1.0;
  std::string expected_message =
      "Spatial inertia fails SpatialInertia::IsPhysicallyValid\\(\\).\n"
      "mass = -1(\\.0)? is negative or not finite.\n";
  DRAKE_EXPECT_THROWS_MESSAGE(SpatialInertia(mass, p_PBcm_E, G_BP_E),
                              expected_message);

  // Ensure a non-finite mass (e.g., infinity or NaN) throws an exception.
  constexpr double kInf = std::numeric_limits<double>::infinity();
  expected_message =
      "Spatial inertia fails SpatialInertia::IsPhysicallyValid\\(\\).\n"
      "mass = inf is negative or not finite.\n";
  DRAKE_EXPECT_THROWS_MESSAGE(SpatialInertia(mass = kInf, p_PBcm_E, G_BP_E),
                              expected_message);

  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  expected_message =
      "Spatial inertia fails SpatialInertia::IsPhysicallyValid\\(\\).\n"
      "mass = nan is negative or not finite.\n";
  DRAKE_EXPECT_THROWS_MESSAGE(SpatialInertia(mass = kNaN, p_PBcm_E, G_BP_E),
                              expected_message);

  // Ensure a position vector with a non-finite element throws an exception.
  p_PBcm_E = Vector3<double>(0.0, 0.0, kInf);
  expected_message =
      "Spatial inertia fails SpatialInertia::IsPhysicallyValid\\(\\).\n"
      "Position vector \\[0(\\.0)?  0(\\.0)?  inf\\] "
      "has non-finite elements.\n";
  DRAKE_EXPECT_THROWS_MESSAGE(SpatialInertia(mass = 1.0, p_PBcm_E, G_BP_E),
                              expected_message);
}

// Test that it is not possible to create a spatial inertia with a bad
// rotational inertia negative mass.
GTEST_TEST(SpatialInertia, IsPhysicallyValidWithBadInertia) {
  const double Ixy = 0.1, Ixz = 0.2, Iyz = 0.3;
  const RotationalInertia<double> I(2, 3, 4, Ixy, Ixz, Iyz);
  RotationalInertia<double> Ibad = I.MultiplyByScalarSkipValidityCheck(-1);
  Vector3d p_PBcm = Vector3d(0.0, 0.0, 0.0);

  const std::string expected_message =
      "Spatial inertia fails SpatialInertia::IsPhysicallyValid\\(\\).\n"
      " mass = 1(\\.0)?\n"
      " Center of mass = \\[0(\\.0)?  0(\\.0)?  0(\\.0)?\\]\n"
      " Inertia about point P, I_BP =\n"
      "\\[  -2  -0.1  -0.2\\]\n"
      "\\[-0.1    -3  -0.3\\]\n"
      "\\[-0.2  -0.3    -4\\]\n"
      " Principal moments of inertia about Bcm \\(center of mass\\) =\n"
      "\\[-4.105976670111\\d+  -2.9188291125626\\d+  -1.9751942173260\\d+\\]\n";
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::MakeFromCentralInertia(1.0, p_PBcm, Ibad),
      expected_message);
}

// Tests IsPhysicallyValid() fails within the constructor since the COM given is
// inconsistently too far out for the unit inertia provided.
GTEST_TEST(SpatialInertia, IsPhysicallyValidWithCOMTooFarOut) {
  const std::string expected_message =
      "Spatial inertia fails SpatialInertia::IsPhysicallyValid\\(\\).\n"
      " mass = 1(\\.0)?\n"
      " Center of mass = \\[2(\\.0)?  0(\\.0)?  0(\\.0)?\\]\n"
      " Inertia about point P, I_BP =\n"
      "\\[0.4    0    0\\]\n"
      "\\[  0  0.4    0\\]\n"
      "\\[  0    0  0.4\\]\n"
      " Inertia about center of mass, I_BBcm =\n"
      "\\[ 0.4     0     0\\]\n"
      "\\[   0  -3.6     0\\]\n"
      "\\[   0     0  -3.6\\]\n"
      " Principal moments of inertia about Bcm \\(center of mass\\) =\n"
      "\\[-3.6  -3.6  0.4\\]\n";
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>(1.0, Vector3d(2.0, 0.0, 0.0),
                             UnitInertia<double>::SolidSphere(1.0)),
      expected_message);
}

// Tests that an informative exception message is issued if a spatial inertia
// fails IsPhysicallyValid().  This test resulted from issue #16058 in which a
// user reported that SpatialInertia::IsPhysicallyValid() failed for what seemed
// like a reasonable spatial inertia. Although the spatial inertia is invalid,
// Drake's previous error messages made it very difficult to see the problem.
// The new error message reports rotational inertia about both Bcm (body or
// composite body B's center of mass) and point P.  This makes it easy to match
// against the user's URDF/SDFormat and also reports the principal moments of
// inertia about Bcm (to help identify the "triangle inequality").
GTEST_TEST(SpatialInertia, IsPhysicallyValidThrowsNiceExceptionMessage) {
  const double mass = 0.634;
  const Vector3<double> p_PBcm(0, 0.016, -0.02);  // Center of mass.
  const double Ixx = 0.001983, Ixy = 0.000245, Ixz = 0.0000129;
  const double Iyy = 0.002103, Iyz = 0.0000015, Izz = 0.000408;

  // Create an invalid rotational inertia.
  const RotationalInertia<double> I_BBcm =
      RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
          Ixx, Iyy, Izz, Ixy, Ixz, Iyz, /* skip_validity_check = */ true);

  // Check for physically invalid rotational inertia.
  EXPECT_FALSE(I_BBcm.CouldBePhysicallyValid());

  // Shift the spatial inertia from Bcm to point P and verify that it throws
  // an exception and the exception message makes sense.
  const std::string expected_message = fmt::format(
      "Spatial inertia fails SpatialInertia::IsPhysicallyValid\\(\\).\n"
      " mass = 0.634\n"
      " Center of mass = \\[0(\\.0)?  0(\\.0)?  0(\\.0)?\\]\n"
      " Inertia about point P, I_BP =\n"
      "\\[0.001983  0.000245  1.29e-05\\]\n"
      "\\[0.000245  0.002103   1.5e-06\\]\n"
      "\\[1.29e-05   1.5e-06  0.000408\\]\n"
      " Principal moments of inertia about Bcm \\(center of mass\\) =\n"
      "\\[0.00040789\\d+  0.00179082\\d+  0.002295284\\d+\\]"
      "\n");
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>::MakeFromCentralInertia(mass, p_PBcm, I_BBcm),
      expected_message);
}

// Tests that by setting skip_validity_check = true, it is possible to create
// invalid spatial inertias with negative mass and malformed COM.
GTEST_TEST(SpatialInertia, SkipValidityCheck) {
  DRAKE_EXPECT_NO_THROW(SpatialInertia<double>(
      -1.0, Vector3d::Zero(), UnitInertia<double>::SolidSphere(1.0), true));
  DRAKE_EXPECT_NO_THROW(
      SpatialInertia<double>(1.0, Vector3d(2.0, 0.0, 0.0),
                             UnitInertia<double>::SolidSphere(1.0), true));
}

// Tests the method SpatialInertia::MakeFromCentralInertia(...).
GTEST_TEST(SpatialInertia, MakeFromCentralInertia) {
  const double mass = 2;
  const Vector3d p_BoBcm_B(3, 4, 5);
  const RotationalInertia<double> I_BBcm_B(6, 7, 8);
  const SpatialInertia<double> M_BBo_B =
      SpatialInertia<double>::MakeFromCentralInertia(mass, p_BoBcm_B, I_BBcm_B);

  // Check for physically correct spatial inertia.
  EXPECT_TRUE(M_BBo_B.IsPhysicallyValid());

  // Check spatial inertia for proper value for mass and center of mass.
  EXPECT_EQ(M_BBo_B.get_mass(), mass);
  EXPECT_EQ(M_BBo_B.get_com(), p_BoBcm_B);

  // Check spatial inertia for proper moments/products of inertia.
  // Note: The values below for Ixx, Iyy, Izz were calculated by MotionGenesis.
  const RotationalInertia<double> I_BBo_B = M_BBo_B.CalcRotationalInertia();
  const Vector3d moments = I_BBo_B.get_moments();
  const double Ixx = 88, Iyy = 75, Izz = 58;
  EXPECT_NEAR(moments(0), Ixx, kEpsilon);
  EXPECT_NEAR(moments(1), Iyy, kEpsilon);
  EXPECT_NEAR(moments(2), Izz, kEpsilon);

  // Check spatial inertia for proper moments/products of inertia.
  // Note: The values below for Ixy, Ixz, Iyz were calculated by MotionGenesis.
  const Vector3d products = I_BBo_B.get_products();
  const double Ixy = -24, Ixz = -30, Iyz = -40;
  EXPECT_NEAR(products(0), Ixy, kEpsilon);
  EXPECT_NEAR(products(1), Ixz, kEpsilon);
  EXPECT_NEAR(products(2), Iyz, kEpsilon);
}

// Verify SpatialInertia::MakeFromCentralInertia() performs the inertia triangle
// validity test _before_ shifting from about-point Bcm to an about-point Bo.
// Before June 2023, the tests below threw an exception due to the inertia
// triangle inequality test. This was happening because the inertia triangle
// inequalilty test was not occurring immediately at Bcm. Instead, significant
// digits were lost when the spatial inertia was shifted a relatively long
// distance from Bcm to Bo for storage and then back from Bo to Bcm for testing
// the inertia triangle inequality. Although the inertia triangle validity test
// has a reasonable tolerance based on the trace of the rotational inertia
// matrix (which is invariant to the inertia's expressed-in basis), the inertia
// triangle validity test can be adversely affected by adding/subtracting large
// terms, e.g., of the form mass * distance².
GTEST_TEST(SpatialInertia, IsMakeFromCentralInertiaTestValidityImmediately) {
  // Create a spatial inertia for a thin rod B about its center of mass Bcm.
  // The allowable violation for rotational inertia triangle inequality test is
  // ≈ 16 * std::numeric_limits<double>::epsilon() * 20.833 ≈ 7.4E-14.
  const double mass = 10.0, length = 5.0;
  const SpatialInertia<double> M_BBcm_B =
      SpatialInertia<double>::ThinRodWithMass(mass, length, Vector3d(0, 0, 1));
  RotationalInertia<double> I_BBcm_B = M_BBcm_B.CalcRotationalInertia();

  // Form M_BBo_B (B's spatial inertia about a point Bo, expressed in B) using
  // the mass, position from Bo to Bcm, and B's rotational inertia about Bcm.
  // Note: With mass = 10 kg and the given position vector from Bo to Bcm,
  // mass * dcm² = mass * (10² + 20² + 30²) = 10 * 1400 = 14000 kg*m².
  // This 14000 kg*m² is much larger than rod B's maximum moment of inertia
  // about Bcm of 1/12 * mass * length² ≈ 20.8333 (ratio 14000 / 28.8333 = 672).
  // Since numbers of such differing size are added to form M_BBo_B, significant
  // digits can be lost in the process. Before June 2023, the shift from Bcm to
  // Bo (for storage) and a later shift from Bo to Bcm (for triangle inequalty
  // test) caused the inertia triangle inequality to be violated by ≈ 1.1E-12.
  Vector3<double> p_BoBcm_B(10, 20, 30);
  DRAKE_EXPECT_NO_THROW(/* M_BBo_B = */
                        SpatialInertia<double>::MakeFromCentralInertia(
                            mass, p_BoBcm_B, I_BBcm_B));

  // Scale the position vector by a factor of 10.
  // Before June 2023, the following caused the inertia triangle inequality test
  // to be violated by ≈ 1.5E-10. Note the ratio 1.5E-10 / 1.1E-12 ≈ 10² = 100.
  // In other words, error scaled as ≈ mass * distance² to center of mass.
  p_BoBcm_B *= 10;
  DRAKE_EXPECT_NO_THROW(/* M_BBo_B = */
                        SpatialInertia<double>::MakeFromCentralInertia(
                            mass, p_BoBcm_B, I_BBcm_B));

  // Before June 2023, the following caused the inertia triangle inequality test
  // to be violated by ≈ 1.2E-06. Note the ratio 1.2E-06 / 1.1E-12 ≈ 1000² = 1E6
  // which reinforces that lost signficiant digits scale with distance².
  p_BoBcm_B *= 100;
  DRAKE_EXPECT_NO_THROW(/* M_BBo_B = */
                        SpatialInertia<double>::MakeFromCentralInertia(
                            mass, p_BoBcm_B, I_BBcm_B));

  // Test for a small spatial inertia with somewhat disorderly digits.
  // Before June 2023, the test below threw an exception (similarly as above).
  I_BBcm_B = RotationalInertia<double>(0.01 / M_PI, 0.01 / M_PI, 0.02 / M_PI);
  p_BoBcm_B = Vector3<double>(10, 20, 30);
  DRAKE_EXPECT_NO_THROW(/* M_BBo_B = */
                        SpatialInertia<double>::MakeFromCentralInertia(
                            mass, p_BoBcm_B, I_BBcm_B));

  // Test a spatial inertia constructor that STILL throws an exception due to
  // the inertia triangle inequality test being violated by ≈ 1.1E-12.
  // Note: This test STILL throws an exception since it does not account for
  // significant digits lost in a relatively large shift to the center of mass.
  const UnitInertia<double> G_BBcm_B = M_BBcm_B.get_unit_inertia();
  UnitInertia<double> G_BBo_B = G_BBcm_B.ShiftFromCenterOfMass(p_BoBcm_B);
  EXPECT_THROW(/* M_BBo_B = */
               SpatialInertia(mass, p_BoBcm_B, G_BBo_B), std::exception);
}

// Verifies the operator*(const SpatialVelocity&) by computing the kinetic
// energy of a cylindrical body B with spatial velocity V_WBp, where P is a
// point that is fixed in the body frame B.
// The computation involves the product of the body's spatial inertia M_Bp_W
// with its spatial velocity V_WBp: ke_WB = 0.5 * V_WBp.dot(M_Bp_W * V_WBp).
// This result is verified against a calculation invoving quantities about the
// bodies COM: ke_WB = 0.5 * mass * v_WBcm² + 0.5 * w_WBᵀ * I_Bcm_W * w_WB.
GTEST_TEST(SpatialInertia, KineticEnergy) {
  // Parameters for a cylindrical body of a given mass, length and radius.
  const double mass = 1.2;
  const double radius = 0.05, length = 1.5;

  // Axis for a cylinder arbitrarily oriented, expressed in world W.
  const Vector3<double> axis_W = Vector3<double>(1, 2, 3).normalized();
  // Create rotational inertia for the cylinder:
  const RotationalInertia<double> I_Bcm_W =
      mass * UnitInertia<double>::SolidCylinder(radius, length, axis_W);

  // Create the spatial inertia of body B about an arbitrary point P.
  const Vector3<double> p_BcmP_W(1, -2, 5);
  const SpatialInertia<double> M_BP_W =
      SpatialInertia<double>::MakeFromCentralInertia(mass, -p_BcmP_W, I_Bcm_W);

  // Say body B moves with spatial velocity V_WBcm.
  const SpatialVelocity<double> V_WBcm(Vector3<double>(1, 2, 3),
                                       Vector3<double>(4, 5, 6));

  // Point P on body B will move with spatial velocity V_WBp.
  const SpatialVelocity<double> V_WBp = V_WBcm.Shift(p_BcmP_W);

  // Compute the kinetic energy of B (in frame W) from V_WBp:
  const double ke_WB = 0.5 * V_WBp.dot(M_BP_W * V_WBp);

  // We expect the energy to be (computed from COM quantities.)
  const Vector3<double>& w_WB = V_WBcm.rotational();
  const Vector3<double>& v_WBcm = V_WBcm.translational();
  const double ke_WB_expected =
      0.5 * w_WB.dot(I_Bcm_W * w_WB) + 0.5 * mass * v_WBcm.squaredNorm();

  const double relative_tol = ke_WB_expected * (8 * kEpsilon);
  EXPECT_NEAR(ke_WB, ke_WB_expected, relative_tol);
}

GTEST_TEST(SpatialInertia, MultiplyByEigenMatrix) {
  // Make an arbitrary spatial inertia.
  const SpatialInertia<double> M = MakeArbitrarySpatialInertia();

  // Make an arbitrary set of spatial accelerations.
  Eigen::Matrix<double, 6, 4> Amatrix;
  // "view" as a column vector of size 24 so that we can use setLinSpaced.
  Eigen::Map<Vector<double, 24>>(Amatrix.data()).setLinSpaced(1, 24);

  // Compute the result in matrix form:
  Eigen::Matrix<double, 6, 4> Fmatrix = M * Amatrix;

  // Verify against the computation performed with spatial accelerations.
  Eigen::Matrix<double, 6, 4> Fmatrix_expected;
  for (int i = 0; i < Amatrix.cols(); ++i) {
    const SpatialAcceleration<double> A(Amatrix.col(i));
    const SpatialForce<double> F = M * A;
    Fmatrix_expected.col(i) = F.get_coeffs();
  }
  EXPECT_TRUE(Fmatrix.isApprox(Fmatrix_expected, kEpsilon));
}

GTEST_TEST(SpatialInertia, SymbolicNan) {
  using T = symbolic::Expression;
  using symbolic::Variable;

  const Variable mass{"m"};
  const Vector3<T> p_PScm_E = symbolic::MakeVectorContinuousVariable(3, "p");
  const UnitInertia<T> G_SP_E{Variable{"Ixx"}, Variable{"Iyy"},
                              Variable{"Izz"}, Variable{"Ixy"},
                              Variable{"Ixz"}, Variable{"Iyz"}};
  const SpatialInertia<T> I{mass, p_PScm_E, G_SP_E};
  ASSERT_EQ(
      I.IsNaN().to_string(),
      "(isnan(m) or isnan(p(0)) or isnan(p(1)) or isnan(p(2)) or isnan(Ixx) or"
      " isnan(Iyy) or isnan(Izz) or isnan(Ixy) or isnan(Ixz) or isnan(Iyz))");
}

// Verifies we can still call IsPhysicallyValid() when T = symbolic::Expression
// and get a result whenever the expression represents a constant.
GTEST_TEST(SpatialInertia, SymbolicConstant) {
  using T = symbolic::Expression;

  // Make a "constant" symbolic expression of a spatial inertia.
  const T mass(2.5);
  const Vector3<T> com(0.1, -0.2, 0.3);
  const Vector3<T> m(2.0, 2.3, 2.4);   // m for moments.
  const Vector3<T> p(0.1, -0.1, 0.2);  // p for products.
  UnitInertia<T> G(m(0), m(1), m(2),   /* moments of inertia */
                   p(0), p(1), p(2));  /* products of inertia */
  SpatialInertia<T> M(mass, com, G);

  // The expression can still be evaluated since all terms are constants.
  ASSERT_TRUE(M.IsPhysicallyValid());
}

// Ensure that MakeFromCentralInertia works for Expression.  This test
// represents a minimal reproduction from a user issue.
GTEST_TEST(SpatialInertia, SymbolicMakeFromCentralInertia) {
  using T = symbolic::Expression;

  symbolic::Variable m("m");
  VectorX<T> com = symbolic::MakeVectorVariable(3, "com");
  VectorX<T> I = symbolic::MakeVectorVariable(3, "I");
  RotationalInertia<T> I_SScm_E(I[0], I[1], I[2]);

  SpatialInertia<T>::MakeFromCentralInertia(m, com, I_SScm_E);
}

// The composition of spatial inertias for two massless bodies is not well
// defined. However we do support this operation in Drake in the limit to zero
// mass when the two bodies have equal mass. In this case, the center of mass
// position vector and the unit inertia of the resulting composite body equals
// the arithmetic average of the center of mass and unit inertia respectively of
// the original two massless bodies.
GTEST_TEST(SpatialInertia, PlusEqualOperatorForTwoMasslessBodies) {
  // To create physically valid unit inertias about an arbitrary point P, we
  // shift from the center of mass a valid unit inertia about the center of
  // mass.

  // Massless body 1.
  const Vector3<double> p_PB1cm_E(1.0, 2.0, 3.0);
  const UnitInertia<double> G_B1Bcm_E =
      UnitInertia<double>::TriaxiallySymmetric(1.0);
  const UnitInertia<double> G_B1P_E =
      G_B1Bcm_E.ShiftFromCenterOfMass(-p_PB1cm_E);
  const SpatialInertia<double> M_B1P_E(0.0, p_PB1cm_E, G_B1P_E);

  // Massless body 2.
  const Vector3<double> p_PB2cm_E(3.0, 2.0, 1.0);
  const UnitInertia<double> G_B2Bcm_E =
      UnitInertia<double>::TriaxiallySymmetric(3.0);
  const UnitInertia<double> G_B2P_E =
      G_B2Bcm_E.ShiftFromCenterOfMass(-p_PB2cm_E);
  const SpatialInertia<double> M_B2P_E(0.0, p_PB2cm_E, G_B2P_E);

  // Compose the spatial inertias of the two massless bodies.
  const SpatialInertia<double> M_BcP_E = SpatialInertia<double>(M_B1P_E) +=
      M_B2P_E;

  // The result should still be the spatial inertia for a massless body.
  EXPECT_EQ(M_BcP_E.get_mass(), 0.0);

  // Verify we get the arithmetic mean.
  const Vector3<double> p_PScm_E =
      0.5 * (M_B1P_E.get_com() + M_B2P_E.get_com());
  const Matrix3<double> G_SP_E =
      0.5 * (M_B1P_E.get_unit_inertia().CopyToFullMatrix3() +
             M_B2P_E.get_unit_inertia().CopyToFullMatrix3());
  EXPECT_EQ(M_BcP_E.get_com(), p_PScm_E);
  EXPECT_EQ(M_BcP_E.get_unit_inertia().CopyToFullMatrix3(), G_SP_E);
}

// Test the SpatialInertia function that determines an inertia-equivalent shape.
GTEST_TEST(SpatialInertia, CalcPrincipalHalfLengthsAndPoseForEquivalentShape) {
  // Consider a body B whose shape (e.g., a solid or hollow ellipsoid or box) is
  // defined by semi-diameters (half-lengths) a, b, c.
  const double a = 5.0, b = 4.0, c = 3.0;  // To facilitate testing, a > b > c.
  const double density = 2.34;
  constexpr double kTolerance = 64 * std::numeric_limits<double>::epsilon();
  const drake::math::RotationMatrix R_identity =
      drake::math::RotationMatrix<double>::Identity();

  // Form the spatial inertia M_BBcm_B for a solid ellipsoid B. Verify the
  // function under test reproduces semi-diameters lmax = a, lmed = b, lmin = c.
  // Verify principal directions Ax, Ay, Az (R_BA is an identity matrix).
  // Verify p_BcmAo_B is zero (since Ao should be located at Bcm).
  SpatialInertia<double> M_BBcm_B =
      SpatialInertia<double>::SolidEllipsoidWithDensity(density, a, b, c);
  auto [abc, X_BA] =
      M_BBcm_B.CalcPrincipalSemiDiametersAndPoseForSolidEllipsoid();
  EXPECT_TRUE(CompareMatrices(Vector3<double>(a, b, c), abc, kTolerance));
  EXPECT_TRUE(X_BA.rotation().IsExactlyEqualTo(R_identity));
  EXPECT_TRUE(X_BA.translation() == Vector3<double>::Zero());

  // Form the spatial inertia M_BBcm_B for a solid box B. Verify the function
  // under test reproduces half-lengths lmax = a, lmed = b, lmin = c.
  // Verify principal directions Ax, Ay, Az (R_BA is an identity matrix).
  // Verify p_BcmAo_B is zero (since Ao should be located at Bcm).
  M_BBcm_B =
      SpatialInertia<double>::SolidBoxWithDensity(density, 2 * a, 2 * b, 2 * c);
  std::tie(abc, X_BA) = M_BBcm_B.CalcPrincipalHalfLengthsAndPoseForSolidBox();
  EXPECT_TRUE(CompareMatrices(Vector3<double>(a, b, c), abc, kTolerance));
  EXPECT_TRUE(X_BA.rotation().IsExactlyEqualTo(R_identity));
  EXPECT_TRUE(X_BA.translation() == Vector3<double>::Zero());

  // For the previous solid box spatial inertia M_BBcm_B, verify the theoretical
  // minimum bounding box is smaller than the actual solid box by a factor which
  // scales via inertia_shape_factor. See @ref spatial_inertia_equivalent_shapes
  // "Spatial inertia equivalent shapes" for details. Since the
  // inertia_shape_factor for a solid box is 1.0 / 3.0 whereas the
  // inertia_shape_factor for the minimum bounding box is 1.0, ensure the
  // minimum bounding box has ½-lengths lmax = a/√3, lmed = b/√3, lmin = c/√3.
  // Verify principal directions Ax, Ay, Az (R_BA is an identity matrix).
  // Verify p_BcmAo_B is zero (since Ao should be located at Bcm).
  std::tie(abc, X_BA) =
      M_BBcm_B.CalcPrincipalHalfLengthsAndPoseForMinimumBoundingBox();
  Vector3<double> min_abc = std::sqrt(1.0 / 3.0) * Vector3<double>(a, b, c);
  EXPECT_TRUE(CompareMatrices(min_abc, abc, kTolerance));
  EXPECT_TRUE(X_BA.rotation().IsExactlyEqualTo(R_identity));
  EXPECT_TRUE(X_BA.translation() == Vector3<double>::Zero());

  // Verify CalcMinimumPhysicalLength() calculates the length of the
  // space-diagonal of the minimum bounding box.
  double space_diagonal = M_BBcm_B.CalcMinimumPhysicalLength();
  EXPECT_NEAR(space_diagonal, (2 * abc).norm(), kTolerance);

  // Translate the solid box B and ensure half-lengths and principal axes are
  // unchanged, whereas the position vector returned in X_BA has changed.
  const Vector3<double> p_BoBcm_B(1.2, 3.4, 5.6);
  SpatialInertia<double> M_BBo_B = M_BBcm_B.Shift(-p_BoBcm_B);
  std::tie(abc, X_BA) = M_BBo_B.CalcPrincipalHalfLengthsAndPoseForSolidBox();
  EXPECT_TRUE(CompareMatrices(Vector3<double>(a, b, c), abc, kTolerance));
  EXPECT_TRUE(X_BA.rotation().IsExactlyEqualTo(R_identity));
  EXPECT_TRUE(X_BA.translation() == p_BoBcm_B);

  // Repeat the previous test for the minimum bounding box B.
  // Verify the answer is not changed by translation.
  std::tie(abc, X_BA) =
      M_BBo_B.CalcPrincipalHalfLengthsAndPoseForMinimumBoundingBox();
  EXPECT_TRUE(CompareMatrices(min_abc, abc, kTolerance));
  EXPECT_TRUE(X_BA.rotation().IsExactlyEqualTo(R_identity));
  EXPECT_TRUE(X_BA.translation() == p_BoBcm_B);

  // Verify CalcMinimumPhysicalLength() calculates the length of the
  // space-diagonal of the minimum bounding box (unchanged by translation).
  space_diagonal = M_BBo_B.CalcMinimumPhysicalLength();
  EXPECT_NEAR(space_diagonal, (2 * abc).norm(), kTolerance);

  // Rotate the solid box B so R_BE is not the identity matrix. Verify ½-lengths
  // are unchanged and principal axes directions Ax Ay Az are parallel to
  // Bx, By, Bz (although Bx ≠ Ex, By ≠ Ey).
  // Note: This tests a rotational inertia with non-zero products of inertia.
  const drake::math::RotationMatrix<double> R_BE =
      drake::math::RotationMatrix<double>::MakeZRotation(M_PI / 6.0);
  SpatialInertia<double> M_BBo_E = M_BBo_B.ReExpress(R_BE);
  const Vector3<double> G_products = M_BBo_E.get_unit_inertia().get_products();
  EXPECT_GE(std::abs(G_products[1]), 0.1);  // Sufficiently non-zero.
  std::tie(abc, X_BA) = M_BBo_E.CalcPrincipalHalfLengthsAndPoseForSolidBox();
  EXPECT_TRUE(CompareMatrices(Vector3<double>(a, b, c), abc, kTolerance));

  // The orthogonal unit length eigenvectors Ax_B, Ay_B, Az_B stored in the
  // columns of R_BA are parallel to the principal axes (lines). Since lines
  // do not have a fully-qualified direction (they lack sense), all we can check
  // is whether these principal axes (represented by Ax_B, Ay_B, Az_B) are
  // parallel to the right-handed unit vectors Ex_B, Ey_B, Ez_B stored in the
  // columns of R_BE and whether they form a right-handed set.
  drake::math::RotationMatrix<double> R_BA = X_BA.rotation();
  Vector3<double> Ax_B = R_BA.col(0), Ex_B = R_BE.col(0);
  Vector3<double> Ay_B = R_BA.col(1), Ey_B = R_BE.col(1);
  Vector3<double> Az_B = R_BA.col(2), Ez_B = R_BE.col(2);
  EXPECT_NEAR(std::abs(Az_B(2)), 1.0, kTolerance);  // Az = [0 0 1] or [0 0 -1]
  EXPECT_NEAR(std::abs(Ax_B.dot(Ex_B)), 1.0, kTolerance);  // Ax parallel to Ex.
  EXPECT_NEAR(std::abs(Ay_B.dot(Ey_B)), 1.0, kTolerance);  // Ay parallel to Ey.
  EXPECT_NEAR(std::abs(Az_B.dot(Ez_B)), 1.0, kTolerance);  // Az parallel to Ez.
  EXPECT_NEAR(Ax_B.cross(Ay_B).dot(Az_B), 1.0, kTolerance);  // Right-handed.

  // Repeat the previous test for the minimum bounding box B.
  std::tie(abc, X_BA) =
      M_BBo_E.CalcPrincipalHalfLengthsAndPoseForMinimumBoundingBox();
  EXPECT_TRUE(CompareMatrices(min_abc, abc, kTolerance));
  R_BA = X_BA.rotation();
  Ax_B = R_BA.col(0);
  Ay_B = R_BA.col(1);
  Az_B = R_BA.col(2);
  EXPECT_NEAR(std::abs(Az_B(2)), 1.0, kTolerance);  // Az = [0 0 1] or [0 0 -1]
  EXPECT_NEAR(std::abs(Ax_B.dot(Ex_B)), 1.0, kTolerance);  // Ax parallel to Ex.
  EXPECT_NEAR(std::abs(Ay_B.dot(Ey_B)), 1.0, kTolerance);  // Ay parallel to Ey.
  EXPECT_NEAR(std::abs(Az_B.dot(Ez_B)), 1.0, kTolerance);  // Az parallel to Ez.
  EXPECT_NEAR(Ax_B.cross(Ay_B).dot(Az_B), 1.0, kTolerance);  // Right-handed.

  // Verify CalcMinimumPhysicalLength() calculates the length of the
  // space-diagonal of the minimum bounding box.
  space_diagonal = M_BBo_E.CalcMinimumPhysicalLength();
  EXPECT_NEAR(space_diagonal, (2 * abc).norm(), kTolerance);
}

GTEST_TEST(SpatialInertia, VerifyMinimumBoundingBoxLengths) {
  // Create a 360 meter massless rod B with 1.0 kg particles at its distal ends.
  // Ensure CalcPrincipalHalfLengthsAndPoseForMinimumBoundingBox() actually
  // produces a minimum bounding box with ½-lengths a = 180 m, b = 0, c = 0.
  constexpr double kTolerance = 64 * std::numeric_limits<double>::epsilon();
  const double mass = 1.0;
  double a = 180, b = 0, c = 0;  // rod's ½-length is 180 meters.
  auto M_BBcm_B = SpatialInertia<double>::NaN();
  M_BBcm_B = SpatialInertia<double>::PointMass(mass, Vector3d(a, 0, 0));
  M_BBcm_B += SpatialInertia<double>::PointMass(mass, Vector3d(-a, 0, 0));
  auto [abc, X_BA] =
      M_BBcm_B.CalcPrincipalHalfLengthsAndPoseForMinimumBoundingBox();
  EXPECT_TRUE(CompareMatrices(Vector3<double>(a, b, c), abc, kTolerance));
  const drake::math::RotationMatrix R_identity =
      drake::math::RotationMatrix<double>::Identity();
  EXPECT_TRUE(X_BA.rotation().IsExactlyEqualTo(R_identity));
  EXPECT_TRUE(X_BA.translation() == Vector3<double>::Zero());

  // Verify CalcMinimumPhysicalLength() returns the length of the space-diagonal
  // of the minimum bounding line-segment (i.e., rod's length = 2 * a = 360 m).
  double space_diagonal = M_BBcm_B.CalcMinimumPhysicalLength();
  EXPECT_NEAR(space_diagonal, 2 * a, kTolerance);

  // Create 400 m x 300 m rectangle B with 1.0 kg particles at its 4 vertices.
  // Ensure CalcPrincipalHalfLengthsAndPoseForMinimumBoundingBox() actually
  // produces a minimum bounding box with ½-lengths a = 200 m, b = 150 m, c = 0.
  a = 200, b = 150, c = 0;  // rectangle sides' ½-lengths in meters.
  M_BBcm_B = SpatialInertia<double>::PointMass(mass, Vector3d(a, b, 0));
  M_BBcm_B += SpatialInertia<double>::PointMass(mass, Vector3d(a, -b, 0));
  M_BBcm_B += SpatialInertia<double>::PointMass(mass, Vector3d(-a, b, 0));
  M_BBcm_B += SpatialInertia<double>::PointMass(mass, Vector3d(-a, -b, 0));
  std::tie(abc, X_BA) =
      M_BBcm_B.CalcPrincipalHalfLengthsAndPoseForMinimumBoundingBox();
  EXPECT_TRUE(CompareMatrices(Vector3<double>(a, b, c), abc, kTolerance));
  EXPECT_TRUE(X_BA.rotation().IsExactlyEqualTo(R_identity));
  EXPECT_TRUE(
      CompareMatrices(X_BA.translation(), Vector3<double>::Zero(), kTolerance));

  // Verify CalcMinimumPhysicalLength() returns the length of the face-diagonal
  // of the minimum bounding rectangle (i.e., √[(2*a)² + (2*b)²] = 500).
  space_diagonal = M_BBcm_B.CalcMinimumPhysicalLength();
  EXPECT_NEAR(space_diagonal, (2 * abc).norm(), kTolerance);

  // Create 360 m x 120 m x 90 m box with 1.0 kg particles at its 8 vertices.
  // Ensure CalcPrincipalHalfLengthsAndPoseForMinimumBoundingBox() actually
  // produces a minimum bounding box with ½-lengths a = 130 m, b = 60 m, c = 45.
  a = 180, b = 60, c = 45;  // box sides' ½-lengths in meters.
  M_BBcm_B = SpatialInertia<double>::PointMass(mass, Vector3d(a, b, c));
  M_BBcm_B += SpatialInertia<double>::PointMass(mass, Vector3d(a, b, -c));
  M_BBcm_B += SpatialInertia<double>::PointMass(mass, Vector3d(a, -b, c));
  M_BBcm_B += SpatialInertia<double>::PointMass(mass, Vector3d(a, -b, -c));
  M_BBcm_B += SpatialInertia<double>::PointMass(mass, Vector3d(-a, b, c));
  M_BBcm_B += SpatialInertia<double>::PointMass(mass, Vector3d(-a, b, -c));
  M_BBcm_B += SpatialInertia<double>::PointMass(mass, Vector3d(-a, -b, c));
  M_BBcm_B += SpatialInertia<double>::PointMass(mass, Vector3d(-a, -b, -c));
  std::tie(abc, X_BA) =
      M_BBcm_B.CalcPrincipalHalfLengthsAndPoseForMinimumBoundingBox();
  EXPECT_TRUE(CompareMatrices(Vector3<double>(a, b, c), abc, kTolerance));
  EXPECT_TRUE(X_BA.rotation().IsExactlyEqualTo(R_identity));
  EXPECT_TRUE(
      CompareMatrices(X_BA.translation(), Vector3<double>::Zero(), kTolerance));

  // Verify CalcMinimumPhysicalLength() returns the length of the space-diagonal
  // of the minimum bounding box (i.e., √[(2*a)² + (2*b)² + (2*c)²] = 390 m).
  space_diagonal = M_BBcm_B.CalcMinimumPhysicalLength();
  EXPECT_NEAR(space_diagonal, (2 * abc).norm(), kTolerance);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
