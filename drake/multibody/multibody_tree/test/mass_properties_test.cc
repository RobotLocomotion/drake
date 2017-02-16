#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"

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
// Also test the implementation of IsNaN().
GTEST_TEST(RotationalInertia, DefaultConstructor) {
  RotationalInertia<double> I;
  // Verify the underlying Eigen matrix.
  EXPECT_TRUE(I.get_matrix().array().isNaN().all());
  EXPECT_TRUE(I.IsNaN());
}

// Test construction and assignment from any Eigen matrix expression.
GTEST_TEST(RotationalInertia, ConstructorFromEigenExpression) {
  Eigen::RowVector3d row(1.0, 2.0, 0.5);
  Eigen::Matrix<double, 4, 3> M;
  M << row, 2.0 * row, 0.5 *row, 0.1 * row;

  // Construction from a Matrix3.
  Matrix3d ma = M.block<3, 3>(0, 0);
  RotationalInertia<double> Ia(ma);
  EXPECT_EQ(Ia.CopyToFullMatrix3(), ma);

  // Assign a Matrix3.
  Matrix3d mm = M.block<3, 3>(1, 0);
  mm = (mm + mm.transpose()).eval();  // make it symmetric.
  Ia = mm;
  EXPECT_EQ(Ia.CopyToFullMatrix3(), mm);

  // Construction from an Eigen block, a more interesting Eigen expression.
  RotationalInertia<double> Ib(M.block<3, 3>(0, 0));
  EXPECT_EQ(Ib.CopyToFullMatrix3(), ma);

  // Assign an Eigen expression.
  Ib = M.block<3, 3>(1, 0) + M.block<3, 3>(1, 0).transpose();
  EXPECT_EQ(Ib.CopyToFullMatrix3(), mm);
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
// Also test SetZero() and SetNaN methods.
GTEST_TEST(RotationalInertia, GeneralConstructor) {
  const Vector3d m(1.0, 1.3, 2.4);  // m for moments.
  const Vector3d p(0.1, 0.3, 1.4);  // m for products.
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

// Test access by (i, j) indexes.
GTEST_TEST(RotationalInertia, AccessByIndexes) {
  const Vector3d m(1.0, 1.3, 2.4);  // m for moments.
  const Vector3d p(0.1, 0.3, 1.4);  // p for products.
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
  I(2, 0) = -1.0;
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
  const Vector3d p(0.1, 0.3, 1.4);  // p for products.
  RotationalInertia<double> I(m(0), m(1), m(2), /* moments of inertia */
                              p(0), p(1), p(2));/* products of inertia */

  // Test that the underlying Eigen representation effectively has NaN entries.
  EXPECT_TRUE(I.get_matrix().array().isNaN().any());

  // Tests however that the copy to a full Matrix3 object is well defined and
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

  // Test that the return from get_symmetric_matrix_view() can be copied to a
  // full matrix with all valide entries.
  Matrix3<double> MatView = I.get_symmetric_matrix_view();
  EXPECT_EQ(MatView, Imatrix);
}

// Test we can take a rotational inertia expressed in a frame R and express it
// in another frame F.
GTEST_TEST(RotationalInertia, ReExpressInAnotherFrame) {
  // Rod frame R located at the rod's geometric center, oriented along its
  // principal axes and z-axis along the rod's axial direction.
  // Inertia computed about Ro and expressed in R.
  const double radius = 0.1;
  const double length = 1.0;

  // Momentum about its axis aligned with R's z-axis.
  const double Irr = radius * radius / 2.0;
  // Moment of inertia about an axis perpendicular to the rod's axis.
  const double Iperp = length * length / 12.0;

  RotationalInertia<double> I_Ro_R(Iperp, Iperp, Irr);

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

// Test the method RotationalInertia::CalcPrincipalMomentsOfInertia() that
// computes the principal moments of inertia of a general rotational inertia
// by solving an eigenvalue problem.
GTEST_TEST(RotationalInertia, PrincipalMomentsOfInertia) {
  const double Lx = 3.0;
  const double Ly = 1.0;
  const double Lz = 5.0;

  // Rotational inertia of a box computed about its center of mass.
  const double Lx2 = Lx * Lx, Ly2 = Ly * Ly, Lz2 = Lz * Lz;
  RotationalInertia<double> I_Bc_W(
      (Ly2 + Lz2) / 12.0,
      (Lx2 + Lz2) / 12.0,
      (Lx2 + Ly2) / 12.0);

  // Define a new frame Q by rotating +20 degrees about x and another +20
  // degrees about z.
  const double angle = 20 * M_PI / 180.0;
  Matrix3<double> R_WQ =
      (AngleAxisd(angle, Vector3d::UnitZ()) *
       AngleAxisd(angle, Vector3d::UnitX())).toRotationMatrix();

  // Compute the cube's spatial inertia in this frame Q.
  // This results in a rotational inertia with all entries being non-zero, i.e
  // far away from being diagonal or diagonalizable in any trivial way.
  RotationalInertia<double> I_Bc_Q = I_Bc_W.ReExpress(R_WQ);

  // Verify that indeed this inertia in frame Q contains non-zero diagonal
  // elements.
  EXPECT_TRUE((I_Bc_Q.CopyToFullMatrix3().array().abs() > 0.1).all());

  // Compute the principal moments of I_Bc_Q.
  Vector3d principal_moments;
  // This method returns true on success.
  EXPECT_TRUE(I_Bc_Q.CalcPrincipalMomentsOfInertia(
      &principal_moments));

  // The expected moments are those originally computed in I_Bc_W, though the
  // return from RotationalInertia::CalcPrincipalMomentsOfInertia() is sorted
  // in ascending order. Therefore reorder before performing the comparison.
  Vector3d expected_principal_moments = I_Bc_W.get_moments();
  std::sort(expected_principal_moments.data(),
            expected_principal_moments.data() +
                expected_principal_moments.size());

  // Verify against the expected value.
  EXPECT_TRUE(expected_principal_moments.isApprox(
      principal_moments, NumTraits<double>::epsilon()));
}

// Test the correctness of multiplication with a scalar from the left.
GTEST_TEST(RotationalInertia, MultiplicationWithScalarFromTheLeft) {
  const Vector3d m(1.0, 1.3, 2.4);  // m for moments.
  const Vector3d p(0.1, 0.3, 1.4);  // p for products.
  RotationalInertia<double> I(m(0), m(1), m(2), /* moments of inertia */
                              p(0), p(1), p(2));/* products of inertia */
  const double scalar = 3.0;
  RotationalInertia<double> sxI = scalar * I;
  EXPECT_EQ(sxI.get_moments(), scalar * m);
  EXPECT_EQ(sxI.get_products(), scalar * p);
}

// Test the correctness of operator+=().
GTEST_TEST(RotationalInertia, OperatorPlusEqual) {
  const Vector3d m(1.0, 1.3, 2.4);  // m for moments.
  const Vector3d p(0.1, 0.3, 1.4);  // p for products.
  RotationalInertia<double> Ia(m(0), m(1), m(2), /* moments of inertia */
                               p(0), p(1), p(2));/* products of inertia */
  // A second inertia.
  RotationalInertia<double> Ib = 2.0 * Ia;

  // Use of operator+=() results in: Ib = Ib + Ia.
  Ib += Ia;

  EXPECT_EQ(Ib.get_moments(), 3.0 * m);
  EXPECT_EQ(Ib.get_products(), 3.0 * p);
}

}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
