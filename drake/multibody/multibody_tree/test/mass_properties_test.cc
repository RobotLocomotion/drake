#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"
#include "drake/multibody/multibody_tree/unit_inertia.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::NumTraits;
using Eigen::Vector3d;
using std::sort;

// Test default constructor which leaves all entries initialized to NaN.
GTEST_TEST(RotationalInertia, DefaultConstructor) {
  RotationalInertia<double> I;
  // Verify the underlying Eigen matrix.
  EXPECT_TRUE(I.get_matrix().array().isNaN().all());
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
  const Vector3d m(1.0, 1.3, 2.4);  // m for moments.
  RotationalInertia<double> I(m(0), m(1), m(2));
  Vector3d moments_expected = m;
  Vector3d products_expected = Vector3d::Zero();
  EXPECT_EQ(I.get_moments(), moments_expected);
  EXPECT_EQ(I.get_products(), products_expected);
}

// Test constructor for a general rotational inertia matrix with non-zero
// off-diagonal elements for which the six entires need to be specified.
GTEST_TEST(RotationalInertia, GeneralConstructor) {
  const Vector3d m(1.0, 1.3, 2.4);  // m for moments.
  const Vector3d p(0.1, 0.3, 1.4);  // m for products.
  RotationalInertia<double> I(m(0), m(1), m(2), /* moments of inertia */
                              p(0), p(1), p(2));/* products of inertia */
  Vector3d moments_expected = m;
  Vector3d products_expected = p;
  EXPECT_EQ(I.get_moments(), moments_expected);
  EXPECT_EQ(I.get_products(), products_expected);
}

// Test access by (i, j) indexes.
GTEST_TEST(RotationalInertia, AccessByIndexes) {
  const Vector3d m(1.0, 1.3, 2.4);  // m for moments.
  const Vector3d p(0.1, 0.3, 1.4);  // m for products.
  RotationalInertia<double> I(m(0), m(1), m(2), /* moments of inertia */
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

  // Test mutable access.
  // This should have the effect of setting both (2, 0) and (0, 2) even when
  // only one element in memory is being accessed.
  I(2,0) = -1.0;
  EXPECT_EQ(I(0, 0), m(0));
  EXPECT_EQ(I(1, 1), m(1));
  EXPECT_EQ(I(2, 2), m(2));
  EXPECT_EQ(I(0, 1), p(0));
  EXPECT_EQ(I(0, 2), -1.0);
  EXPECT_EQ(I(1, 2), p(2));
  EXPECT_EQ(I(1, 0), p(0));
  EXPECT_EQ(I(2, 0), -1.0);
  EXPECT_EQ(I(2, 1), p(2));
}

// Tests that even when the underlying dense Eigen representation holds NaN
// entries for unused entries, the RotationalInertia behaves as a symmetric
// matrix.
GTEST_TEST(RotationalInertia, Symmetry) {
  const Vector3d m(1.0, 1.3, 2.4);  // m for moments.
  const Vector3d p(0.1, 0.3, 1.4);  // m for products.
  RotationalInertia<double> I(m(0), m(1), m(2), /* moments of inertia */
                              p(0), p(1), p(2));/* products of inertia */

  // Test that the underlying Eigen representation effectively has NaN entries.
  EXPECT_TRUE(I.get_matrix().array().isNaN().any());

  // Tests however that the copy to a full Matrix3 object is well defined and
  // leads to a symmetric matrix.
  Matrix3d Imatrix = I.CopyToFullMatrix3();
  EXPECT_FALSE(Imatrix.array().isNaN().any()); // no entry is NaN.
  EXPECT_EQ(Imatrix(0, 0), m(0));
  EXPECT_EQ(Imatrix(1, 1), m(1));
  EXPECT_EQ(Imatrix(2, 2), m(2));
  EXPECT_EQ(Imatrix(0, 1), p(0));
  EXPECT_EQ(Imatrix(0, 2), p(1));
  EXPECT_EQ(Imatrix(1, 2), p(2));
  EXPECT_EQ(Imatrix(1, 0), p(0));
  EXPECT_EQ(Imatrix(2, 0), p(1));
  EXPECT_EQ(Imatrix(2, 1), p(2));

  // Test that the return from get_symmetric_matrix_view() can be copied to a
  // full matrix with all valide entries.
  Matrix3<double> MatView = I.get_symmetric_matrix_view();
  EXPECT_EQ(MatView, Imatrix);
}

// Test we can take a rotational inertia expressed in a frame R and express it
// in another frame F.
GTEST_TEST(RotationalInertia, ReExpressInAnotherFrame) {
  const double radius = 0.1;
  const double length = 1.0;
  // Rod frame R located at the rod's geometric center, oriented along its
  // principal axes and z-axis along the rod's axial direction.
  // Inertia computed about Ro and expressed in R.
  RotationalInertia<double> I_Ro_R =
      UnitInertia<double>::SolidRod(radius, length);
  // Momentum about its axis aligned with R's z-axis.
  const double Irr = I_Ro_R(2, 2);
  // Moment of inertia about an axis perpendicular to the rod's axis.
  const double Iperp = I_Ro_R(0, 0);

  // Rotation of +90 degrees about x.
  Matrix3<double> R_FR =
      AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();

  // Re-express in frame F using the above rotation.
  RotationalInertia<double> I_Ro_F = I_Ro_R.ReExpress(R_FR);

  // Verify that now R's z-axis is oriented along F's y-axis.
  EXPECT_NEAR(I_Ro_F(0, 0), Iperp, Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(I_Ro_F(1, 1), Irr, Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(I_Ro_F(2, 2), Iperp, Eigen::NumTraits<double>::epsilon());

  // While at it, check if after transformation this still is a physically
  // valid inertia.
  EXPECT_TRUE(I_Ro_F.IsPhysicallyValid());
}

GTEST_TEST(RotationalInertia, PrincipalMomentsOfInertia) {
  const double L1 = 3.0;
  const double L2 = 1.0;
  const double L3 = 5.0;

  // Rotational inertia computed about the center of mass of the cube.
  const RotationalInertia<double>& I_Bc_W =
      UnitInertia<double>::SolidBox(L1, L2, L3);

  // Define a new frame Q by rotating +20 degrees about x and then z.
  const double angle = 20 * M_PI / 180.0;
  Matrix3<double> R_WQ =
      (AngleAxisd(angle, Vector3d::UnitZ()) *
       AngleAxisd(angle, Vector3d::UnitX())).toRotationMatrix();

  // Compute the cube's spatial inertia in this frame Q.
  // This results in a rotational inertia with all entries being non-zero, i.e
  // far away from being diagonal or diagonalizable in any trivial way.
  RotationalInertia<double> I_Bc_Q = I_Bc_W.ReExpress(R_WQ);

  // Compute the principal moments.
  Vector3d principal_moments;
  EXPECT_TRUE(I_Bc_Q.CalcPrincipalMomentsOfInertia(
      &principal_moments));
  PRINT_VARn(principal_moments);

  // The expected moments are those originally computed in I_Bc_W, though the
  // return from RotationalInertia::CalcPrincipalMomentsOfInertia() is sorted
  // in ascending order. Therefore reorder to perform comparison.
  Vector3d expected_principal_moments = I_Bc_W.get_moments();
  std::sort(expected_principal_moments.data(),
            expected_principal_moments.data() +
                expected_principal_moments.size());
  PRINT_VAR(expected_principal_moments.transpose());
  EXPECT_TRUE(expected_principal_moments.isApprox(
      principal_moments, NumTraits<double>::epsilon()));
}

// Add a simple test for the RotationalInertia+= operator
#if 0
GTEST_TEST(SpatialInertia, PlusEqualOperator) {
  const double L = 2.0;
  // Rod frame R located at the rod's geometric center and oriented along its
  // principal axes.
  // Inertia computed about Ro and expressed in R.

  // Spatial inertia computed about the origin for a cube with sides of
  // length 2.0 centered at x = 1.0. Expressed in world frame.
  const double mass_right = 1.5;
  SpatialInertia<double> MRightBox_Wo_W(
      mass_right,
      Vector3d::Zero(),
      UnitInertia<double>::SolidCube(L));
  MRightBox_Wo_W.ShiftInPlace(-Vector3d::UnitX());
  // Check if after transformation this still is a physically valid inertia.
  EXPECT_TRUE(MRightBox_Wo_W.IsPhysicallyValid());

  PRINT_VARn(MRightBox_Wo_W);

  // Spatial inertia computed about the origin for a cube with sides of
  // length 2.0 centered at x = -1.0. Expressed in world frame.
  const double mass_left = 0.5;
  SpatialInertia<double> MLeftBox_Wo_W(
      mass_left,
      Vector3d::Zero(),
      UnitInertia<double>::SolidCube(L));
  MLeftBox_Wo_W.ShiftInPlace(Vector3d::UnitX());
  // Check if after transformation this still is a physically valid inertia.
  EXPECT_TRUE(MLeftBox_Wo_W.IsPhysicallyValid());

  PRINT_VARn(MLeftBox_Wo_W);

  // Spatial inertia of a prism with a squared transverse area of size
  // 2.0 x 2.0 and length 4.0.
  // This is computed by adding the above spatial inertias.
  // Notice that the origina and the expressed-in frame is the same as in the
  // two individual components.
  SpatialInertia<double> MPrism_Wo_W(MLeftBox_Wo_W);
  MPrism_Wo_W += MRightBox_Wo_W;
  EXPECT_TRUE(MPrism_Wo_W.IsPhysicallyValid());

  PRINT_VARn(MPrism_Wo_W);

  // Check that the compound inertia corresponds to that of a larger box of
  // length 4.0.
  const double mass = mass_left + mass_right;
  const Vector3d com(
      (mass_left * MLeftBox_Wo_W.get_com() +
       mass_right * MRightBox_Wo_W.get_com()) / mass);
  SpatialInertia<double> MExpected_Wo_W(
      mass,
      com,
      UnitInertia<double>::SolidBox(2 * L, L, L));
  // Check if after transformation this still is a physically valid inertia.
  EXPECT_TRUE(MExpected_Wo_W.IsPhysicallyValid());

  PRINT_VARn(MExpected_Wo_W);

  EXPECT_TRUE(MPrism_Wo_W.IsApprox(MExpected_Wo_W));
}
#endif

}
}  // math
}  // multibody
}  // drake