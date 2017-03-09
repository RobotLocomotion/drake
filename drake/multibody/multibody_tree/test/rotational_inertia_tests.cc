#include "gtest/gtest.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"

#include <iomanip>
#include <sstream>
#include <string>

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::NumTraits;
using Eigen::Vector3d;
using std::sort;

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

// Test we can take a rotational inertia expressed in a frame R and express it
// in another frame F.
GTEST_TEST(RotationalInertia, ReExpressInAnotherFrame) {
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
  EXPECT_NEAR(I_Ro_F(0, 0), Iperp, Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(I_Ro_F(1, 1), Irr, Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(I_Ro_F(2, 2), Iperp, Eigen::NumTraits<double>::epsilon());

  // While at it, check if after transformation this still is a physically
  // valid inertia.
  EXPECT_TRUE(I_Ro_F.CouldBePhysicallyValid());
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
      wcross_expected, Eigen::NumTraits<double>::epsilon()));

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

  EXPECT_TRUE(Idot_W.isApprox(
      Idot_W_expected, Eigen::NumTraits<double>::epsilon()));
}

}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
