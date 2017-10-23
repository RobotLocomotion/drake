#include "drake/multibody/multibody_tree/spatial_inertia.h"

#include <limits>
#include <sstream>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"
#include "drake/multibody/multibody_tree/unit_inertia.h"

namespace drake {
namespace multibody {
namespace {

using drake::math::VectorToSkewSymmetric;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using std::numeric_limits;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Test default constructor which leaves entries initialized to NaN for a
// quick detection of uninitialized values.
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
// inertia with zero gradients. Since we are using a dynamic size
// AutoDiffScalar type, this results in gradient vectors with zero size.
GTEST_TEST(SpatialInertia, CastToAutoDiff) {
  const double mass_double = 2.5;
  const Vector3d com_double(0.1, -0.2, 0.3);
  const Vector3d m_double(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p_double(0.1, -0.1, 0.2);  // p for products.
  const UnitInertia<double> G_double(
      m_double(0), m_double(1), m_double(2), /* moments of inertia */
      p_double(0), p_double(1), p_double(2));/* products of inertia */
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
  auto G_value = math::autoDiffToValueMatrix(G_autodiff_matrix);
  G_value.resize(3, 3);
  EXPECT_TRUE(G_value.isApprox(G_double.CopyToFullMatrix3(), kEpsilon));
  MatrixXd G_gradient = math::autoDiffToGradientMatrix(G_autodiff_matrix);
  ASSERT_EQ(G_gradient.size(), 0);

  // Values and gradients of the unit inertia.
  const auto& com_autodiff = M_autodiff.get_com();
  const Vector3d com_value = math::autoDiffToValueMatrix(com_autodiff);
  EXPECT_TRUE(com_value.isApprox(com_double, kEpsilon));
  MatrixXd com_gradient = math::autoDiffToGradientMatrix(com_autodiff);
  ASSERT_EQ(com_gradient.size(), 0);
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
      " com = [ 0.1000 -0.2000  0.3000]ᵀ\n"
      " I = \n"
      "[ 5.0000,  0.2500, -0.2500]\n"
      "[ 0.2500,  5.7500,  0.5000]\n"
      "[-0.2500,  0.5000,  6.0000]\n";
  EXPECT_EQ(expected_string, stream.str());
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
  SpatialInertia<double> MRightBox_Wo_W(
      mass_right,
      Vector3d::Zero(),
      UnitInertia<double>::SolidCube(L));
  MRightBox_Wo_W.ShiftInPlace(-Vector3d::UnitX());
  // Check if after transformation this still is a physically valid inertia.
  EXPECT_TRUE(MRightBox_Wo_W.IsPhysicallyValid());

  // Spatial inertia computed about the origin for a cube with sides of
  // length L centered at x = -1.0. Expressed in world frame W.
  const double mass_left = 0.5;  // Mass of the cube on the left.
  // So far the "about point" is the cube's centroid.
  // We'll shift the about point below.
  SpatialInertia<double> MLeftBox_Wo_W(
      mass_left,
      Vector3d::Zero(),
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
  const Vector3d com(
      (mass_left * MLeftBox_Wo_W.get_com() +
       mass_right * MRightBox_Wo_W.get_com()) / mass);
  SpatialInertia<double> MExpected_Wo_W(
      mass,
      com,
      UnitInertia<double>::SolidBox(2 * L, L, L));
  EXPECT_TRUE(MBox_Wo_W.CopyToFullMatrix6().isApprox(
      MExpected_Wo_W.CopyToFullMatrix6(), kEpsilon));
}

// Tests the method SpatialInertia::ReExpress().
GTEST_TEST(SpatialInertia, ReExpress) {
  // Spatial inertia for a cube C computed about a point P and expressed in a
  // frame E.
  const double Lx = 0.2, Ly = 1.0, Lz = 0.5;  // Cube's lengths.
  const double mass = 1.3;  // Cube's mass
  SpatialInertia<double> M_CP_E(  // First computed about its centroid.
      mass, Vector3d::Zero(), UnitInertia<double>::SolidBox(Lx, Ly, Lz));
  // Shift to point P placed one unit in the y direction along the y axis in
  // frame E.
  M_CP_E.ShiftInPlace(Vector3d::UnitY());

  // Place B rotated +90 degrees about W's x-axis.
  Matrix3<double> R_WE =
      AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();

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
  Matrix3<double> R_WB =
      AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();

  // Spatial inertia for a thin cylinder of computed about its center of
  // mass and expressed in frame W.
  const double mass = 1.2;
  const double radius = 0.05, length = 1.5;
  // First define it in frame B.
  SpatialInertia<double> M_BBcm_W(
      mass, Vector3d::Zero(),
      UnitInertia<double>::SolidCylinder(radius, length));
  // Then re-express in frame W.
  M_BBcm_W.ReExpressInPlace(R_WB);

  // Vector from Bcm to the the top of the cylinder Btop.
  Vector3d p_BcmBtop_W(0, length/2.0, 0);

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
      mass * (3 * radius * radius + length * length) / 12  /*About centroid.*/
      + mass * length * length / 4;  /*Parallel axis theorem shift.*/
  const auto I_Xo_W = M_BBtop_W.CalcRotationalInertia();
  EXPECT_NEAR(I_Xo_W(0, 0), I_end, kEpsilon);
  EXPECT_NEAR(I_Xo_W(2, 2), I_end, kEpsilon);

  // Now check that shifting back to the COM results in the same spatial
  // inertia.
  SpatialInertia<double> M_BBcm_W_back = M_BBtop_W.Shift(-p_BcmBtop_W);
  EXPECT_TRUE(M_BBcm_W_back.CopyToFullMatrix6().isApprox(
      M_BBcm_W.CopyToFullMatrix6(), kEpsilon));
}

// Tests that it is not possible to create a spatial inertia with negative mass
// since IsPhysicallyValid() will fail in the constructor.
GTEST_TEST(SpatialInertia, IsPhysicallyValidWithNegativeMass) {
  try {
    SpatialInertia<double> M(
        -1.0, Vector3d::Zero(),
        UnitInertia<double>::SolidSphere(1.0));
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    std::string expected_msg =
        "The resulting spatial inertia is not physically valid. "
        "See SpatialInertia::IsPhysicallyValid()";
    EXPECT_EQ(e.what(), expected_msg);
  }
}

// Tests IsPhysicallyValid() fails within the constructor since the COM given is
// inconsistently too far out for the unit inertia provided.
GTEST_TEST(SpatialInertia, IsPhysicallyValidWithCOMTooFarOut) {
  try {
    SpatialInertia<double> M(
        1.0, Vector3d(2.0, 0.0, 0.0),
        UnitInertia<double>::SolidSphere(1.0));
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    std::string expected_msg =
        "The resulting spatial inertia is not physically valid. "
        "See SpatialInertia::IsPhysicallyValid()";
    EXPECT_EQ(e.what(), expected_msg);
  }
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
  const Vector3d moments  = I_BBo_B.get_moments();
  const double Ixx = 88, Iyy = 75,  Izz = 58;
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

}  // namespace
}  // namespace multibody
}  // namespace drake
