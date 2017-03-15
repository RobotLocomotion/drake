#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"
#include "drake/multibody/multibody_tree/unit_inertia.h"

#include <limits>

#include <algorithm>
#include <iostream>
#include <sstream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {
namespace math {
namespace {

using drake::math::VectorToSkewSymmetric;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::NumTraits;
using Eigen::Vector3d;
using std::sort;

// Test default constructor which leaves entries initialized to NaN for a
// quick detection of un-initialized values.
GTEST_TEST(SpatialInertia, DefaultConstructor) {
  SpatialInertia<double> I;
  ASSERT_TRUE(I.IsNaN());
}

// Test the construction from the mass, center of mass, and unit inertia of a
// body. Also tests:
//   - Getters.
//   - CopyToFullMatrix6().
//   - SetNan()
GTEST_TEST(SpatialInertia, ConstructionFromMasComAndUnitInertia) {
  const double mass = 2.5;
  const Vector3d com(0.1, -0.2, 0.3);
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);  // p for products.
  UnitInertia<double> G(m(0), m(1), m(2), /* moments of inertia */
                        p(0), p(1), p(2));/* products of inertia */
  SpatialInertia<double> M(mass, com, G);
  ASSERT_TRUE(M.IsPhysicallyValid());
  
  ASSERT_EQ(M.get_mass(), mass);
  ASSERT_EQ(M.get_com(), com);
  ASSERT_TRUE(M.get_rotational_inertia().IsApprox(
      mass * G, std::numeric_limits<double>::epsilon()));

  Matrix6<double> Mmatrix = M.CopyToFullMatrix6();
  Matrix6<double> expected_matrix;
  expected_matrix.block<3,3>(0,0) = mass * G.CopyToFullMatrix3();
  expected_matrix.block<3,3>(3,3) = mass * Matrix3d::Identity();
  expected_matrix.block<3,3>(0,3) = mass * VectorToSkewSymmetric(com);
  expected_matrix.block<3,3>(3,0) = expected_matrix.block<3,3>(0,3).transpose();

  EXPECT_TRUE(Mmatrix.isApprox(expected_matrix, NumTraits<double>::epsilon()));

  EXPECT_FALSE(M.IsNaN());
  M.SetNaN();
  EXPECT_TRUE(M.IsNaN());
}

// Test the shift operator to write into a stream.
GTEST_TEST(SpatialInertia, ShiftOperator) {
  const double mass = 2.5;
  const Vector3d com(0.1, -0.2, 0.3);
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);  // p for products.
  UnitInertia<double> G(m(0), m(1), m(2), /* moments of inertia */
                        p(0), p(1), p(2));/* products of inertia */
  SpatialInertia<double> M(mass, com, G);

  std::stringstream stream;
  stream << std::fixed << std::setprecision(4) << M;
  std::string expected_string =
      " mass = 2.5000\n"
      " com = [ 0.1000 -0.2000  0.3000]^T\n"
      " I = \n"
      "[ 5.0000,  0.2500, -0.2500]\n"
      "[ 0.2500,  5.7500,  0.5000]\n"
      "[-0.2500,  0.5000,  6.0000]\n";
  EXPECT_EQ(expected_string, stream.str());
}

// Tests comparison to a given precision.
GTEST_TEST(SpatialInertia, IsApprox) {
  const double mass = 2.5;
  const Vector3d com(0.1, -0.2, 0.3);
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);  // p for products.
  UnitInertia<double> G(m(0), m(1), m(2), /* moments of inertia */
                        p(0), p(1), p(2));/* products of inertia */
  SpatialInertia<double> M(mass, com, G);

  // Creates a spatial inertia that is approximately equal to M, but not equal.
  const double precision = 1.0e-10;
  SpatialInertia<double> other(
      (1.0 + precision) * mass,
      (1.0 + precision) * com,
      UnitInertia<double>((1.0 + precision) * G));
  EXPECT_TRUE(M.IsApprox(other, 2.6 * precision));
  EXPECT_FALSE(M.IsApprox(other, 2.5 * precision));
}


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

#if 0
GTEST_TEST(SpatialInertia, ReExpress) {
  // Spatial inertia for a cube of unit mass computed about its center of mass
  // and expressed in its principal axes frame.
  const double Lx = 0.2, Ly = 1.0, Lz = 0.5;  // Box's lengths.
  SpatialInertia<double> M_Bo_B(
      1.0, Vector3d::UnitX(), UnitInertia<double>::SolidBox(Lx, Ly, Lz));

  // Replace this pring by the respective getters and check values.
  PRINT_VARn(M_Bo_B);

  // Place B rotated +90 degrees about W's x-axis.
  Matrix3<double> R_WB =
      AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();
  PRINT_VARn(R_WB);

  SpatialInertia<double> M_Bo_W = M_Bo_B.ReExpress(R_WB);

  // Checks for physically correct spatial inertia.
  EXPECT_TRUE(M_Bo_W.IsPhysicallyValid());

  // Replace this print by the respective checks of the new computed values.
  // What was in y now is in z.
  PRINT_VARn(M_Bo_W);
}
#endif

#if 0
GTEST_TEST(SpatialInertia, Shift) {
  // Place B rotated +90 degrees about W's x-axis.
  Matrix3<double> R_WB =
      AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();

  // Spatial inertia for a thin rod of unit mass computed about its center of
  // mass and expressed in the world frame W.
  const double mass = 1.2;
  const double radius = 0.05, length = 1.0;
  SpatialInertia<double> M_Bo_W(
      mass, Vector3d::Zero(),
      UnitInertia<double>::SolidRod(radius, length));
  M_Bo_W.ReExpressInPlace(R_WB);

  // Replace this print by the respective getters and check values.
  PRINT_VARn(M_Bo_W);

  // Vector from Bo, in this case Bo = Bc, to the top of the box.
  Vector3d p_BoXo_W(0, 0.5, 0);

  PRINT_VARn(p_BoXo_W.transpose());

  // Computes spatial inertia about Xo, still expressed in W.
  SpatialInertia<double> M_Xo_W = M_Bo_W.Shift(p_BoXo_W);

  // Replace this print by the respective checks of the new computed values.
  // What was in y now is in z.
  PRINT_VARn(M_Xo_W);

  // Checks for physically correct spatial inertia.
  EXPECT_TRUE(M_Xo_W.IsPhysicallyValid());

  // Expected moment of inertia for a rod when computed about one of its ends.
  const double I_end = mass * length / 3.0;
  const auto& I_Xo_W = M_Xo_W.get_rotational_inertia();
  EXPECT_NEAR(I_Xo_W(0,0), I_end, Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(I_Xo_W(2,2), I_end, Eigen::NumTraits<double>::epsilon());
}

// This tests the implementation of two product operators:
// 1. SpatialVector operator*(const SpatialVector& V) const;
// 2. SpatialVelocityJacobian operator*(const SpatialVelocityJacobian& J) const;
//
// The first operator allows to multiply a SpatialInertia with a SpatialVector
// from the right.
// The second operator allows to multiply a SpatialInertia with a
// SpatialVelocityJacobian. A SpatialVelocityJacobian can be viewed as a matrix
// where each column is a SpatialVector.
GTEST_TEST(SpatialInertia, ProductWithSpatialVectors) {
  const double mass = 2.5;
  const double radius = 0.1;
  const double length = 1.0;

  // Spatial inertia of a rod along the z-axis computed about center of mass.
  SpatialInertia<double> M_Bc_W(
      mass,
      Vector3d::Zero(),
      UnitInertia<double>::SolidRod(radius, length));

  // Spatial inertia about the top end of the bar.
  SpatialInertia<double> M_Eo_W =
      M_Bc_W.Shift(length / 2 * Vector3d::UnitZ());
  EXPECT_TRUE(M_Eo_W.IsPhysicallyValid());

  // Assume a given linear velocity.
  const Vector3d v_WB(-1, 2, 8);

  // Assume a given angular velocity.
  const Vector3d w_WB(1, 2, 3);

  // Spatial velocity of its top end frame.
  SpatialVector<double> V_WB(w_WB, v_WB);

  // Product of a SpatialInertia times a SpatialVector
  SpatialVector<double> F_WB = M_Bc_W * V_WB;

  SpatialVector<double> F_expected(
      Vector3d(5.0 / 24.0, 5.0 / 12.0, 0.0375),
      Vector3d(-2.5, 5.0, 20.0));
  EXPECT_TRUE(F_WB.IsApprox(F_expected, Eigen::NumTraits<double>::epsilon()));

  // Jacobian with three columns but a maximum up to six columns.
  SpatialVelocityJacobianUpTo6<double> H(3);
  H.col(0) = V_WB;
  H.col(1) = 2. * V_WB;
  H.col(2) = 3. * V_WB;
  EXPECT_EQ(H.cols(), 3);  // Verifying the constructor did the right thing.

  // Product of a SpatialInertia times a SpatialVelocityJacobian.
  SpatialVelocityJacobianUpTo6<double> MxH = M_Bc_W * H;
  EXPECT_EQ(MxH.rows(), 6);  // Trivial. This is always true.
  EXPECT_EQ(MxH.cols(), H.cols());  // Depends on the size of H.
  EXPECT_TRUE(MxH.col(0).IsApprox(F_WB, Eigen::NumTraits<double>::epsilon()));
  EXPECT_TRUE(MxH.col(1).IsApprox(
      2.0 * F_WB, Eigen::NumTraits<double>::epsilon()));
  EXPECT_TRUE(MxH.col(2).IsApprox(
      3.0 * F_WB, Eigen::NumTraits<double>::epsilon()));
}

GTEST_TEST(SpatialInertia, ComputeKineticEnergy) {
  const double mass = 2.5;
  const double radius = 0.1;
  const double length = 1.0;

  // Spatial inertia of a rod along the z-axis computed about center of mass.
  SpatialInertia<double> M_Bc_W(
      mass,
      Vector3d::Zero(),
      UnitInertia<double>::SolidRod(radius, length));

  // Spatial inertia about the top end of the bar.
  SpatialInertia<double> M_Eo_W =
      M_Bc_W.Shift(length / 2 * Vector3d::UnitZ());
  EXPECT_TRUE(M_Eo_W.IsPhysicallyValid());

  // Assume a given angular velocity.
  const double w0 = 2.5;  // Magnitude of the angular velocity.
  const Vector3d w_WBc = -w0 * Vector3d::UnitY();

  // Spatial velocity of its top end frame.
  SpatialVector<double> V_WE(w_WBc, Vector3d::Zero());

  // Spatial velocity of its center of mass.
  SpatialVector<double> V_WB =
      /* phi_BE */
      ShiftOperator<double>(M_Eo_W.get_com()).transpose() * V_WE;

  PRINT_VARn(V_WE);
  PRINT_VARn(V_WB);
  PRINT_VARn(M_Bc_W);

  // Computes kinetic energy using spatial operations:
  // Using spatial algebra about the center of mass:
  double K_com = 0.5 * V_WB.dot(M_Bc_W * V_WB);

  PRINT_VARn(M_Bc_W * V_WB);

  // Using spatial algebra about the rod's end:
  double K_end = 0.5 * V_WE.dot(M_Eo_W * V_WE);

  PRINT_VAR(K_com);
  PRINT_VAR(K_end);

  // Expected kinetic energy of a rod rotation about its end.
  const double Iend = mass * length * length / 3.0;
  const double Kexpected = 0.5 * Iend * w0 * w0;
  PRINT_VAR(Kexpected);

  EXPECT_NEAR(Kexpected, K_com, 4 * NumTraits<double>::epsilon());
  EXPECT_NEAR(Kexpected, K_end, 4 * NumTraits<double>::epsilon());

}
#endif


}
}  // math
}  // multibody
}  // drake