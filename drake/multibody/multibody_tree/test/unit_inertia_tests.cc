#include "drake/multibody/multibody_tree/unit_inertia.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::NumTraits;
using Eigen::Vector3d;

// Test default constructor which leaves entries initialized to NaN for a
// quick detection of un-initialized values.
GTEST_TEST(UnitInertia, DefaultConstructor) {
  UnitInertia<double> I;
  ASSERT_TRUE(I.IsNaN());
}

// Test constructor for a diagonal unit inertia with all elements equal.
GTEST_TEST(UnitInertia, DiagonalInertiaConstructor) {
  const double I0 = 3.14;
  UnitInertia<double> I(I0);
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
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);  // p for products.
  UnitInertia<double> I(m(0), m(1), m(2), /* moments of inertia */
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

// Test constructor from a RotationalInertia.
GTEST_TEST(UnitInertia, ConstructorFromRotationalInertia) {
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
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
  Matrix3<double> R_FR =
      AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();

  // Re-express in frame F using the above rotation.
  const UnitInertia<double> G_Ro_F = G_Ro_R.ReExpress(R_FR);

  // Verify that now R's z-axis is oriented along F's y-axis.
  EXPECT_NEAR(G_Ro_F(0, 0), Iperp, Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(G_Ro_F(1, 1), Irr, Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(G_Ro_F(2, 2), Iperp, Eigen::NumTraits<double>::epsilon());
  EXPECT_TRUE(
      G_Ro_F.get_products().isZero(Eigen::NumTraits<double>::epsilon()));

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
  EXPECT_TRUE(uxuxv.isApprox(G * v, Eigen::NumTraits<double>::epsilon()));
}

// Tests the static method to obtain the unit inertia of a solid sphere.
GTEST_TEST(UnitInertia, SolidSphere) {
  const double radius = 3.5;
  const double sphere_I = 4.9;
  const UnitInertia<double> G_expected(sphere_I);
  UnitInertia<double> G = UnitInertia<double>::SolidSphere(radius);
  EXPECT_TRUE(G_expected.get_moments() == G.get_moments());
  EXPECT_TRUE(G_expected.get_products() == G.get_products());
}

// Tests the static method to obtain the unit inertia of a hollow sphere.
GTEST_TEST(UnitInertia, HollowSphere) {
  const double radius = 3.5;
  const double sphere_I = 2.0 *radius * radius / 3.0;
  const UnitInertia<double> G_expected(sphere_I);
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
  EXPECT_TRUE(G.IsApprox(G_expected));
}

// Tests the static method to obtain the unit inertia of a solid cube.
GTEST_TEST(UnitInertia, SolidCube) {
  const double L = 1.5;
  const double I = L * L / 6.0;
  const UnitInertia<double> G_expected(I);
  UnitInertia<double> G = UnitInertia<double>::SolidCube(L);
  EXPECT_TRUE(G.IsApprox(G_expected));
}

// Tests the static method to obtain the unit inertia of a solid cylinder.
GTEST_TEST(UnitInertia, SolidCylinder) {
  const double r = 2.5;
  const double L = 1.5;
  const double I_perp = (3.0 * r * r + L * L) / 12.0;
  const double I_axial = r * r / 2.0;
  const UnitInertia<double> G_expected(I_perp, I_perp, I_axial);
  UnitInertia<double> G = UnitInertia<double>::SolidCylinder(r, L);
  EXPECT_TRUE(G.IsApprox(G_expected));
}

// Tests the static method to obtain the unit inertia of a solid cylinder
// computed about a point at the center of its base.
GTEST_TEST(UnitInertia, SolidCylinderAboutEnd) {
  const double r = 2.5;
  const double L = 1.5;
  const double I_perp = (3.0 * r * r + L * L) / 12.0 + L * L /4.0;
  const double I_axial = r * r / 2.0;
  const UnitInertia<double> G_expected(I_perp, I_perp, I_axial);
  UnitInertia<double> G = UnitInertia<double>::SolidCylinderAboutEnd(r, L);
  EXPECT_TRUE(G.IsApprox(G_expected));
}

// Tests the methods ShiftFromCentroidInPlace() and ShiftFromCentroid().
GTEST_TEST(UnitInertia, ShiftFromCentroidInPlace) {
  const double r = 2.5;
  const double L = 1.5;
  const UnitInertia<double> G_expected =
      UnitInertia<double>::SolidCylinderAboutEnd(r, L);
  UnitInertia<double> G = UnitInertia<double>::SolidCylinder(r, L);
  EXPECT_FALSE(G.IsApprox(G_expected));  // Not equal yet.
  G.ShiftFromCentroidInPlace({0.0, 0.0, L / 2.0});
  EXPECT_TRUE(G.IsApprox(G_expected));  // Equal after shifting in place.
  EXPECT_TRUE(G.CouldBePhysicallyValid());

  // Create a new object.
  UnitInertia<double> G2 =
      UnitInertia<double>::
      SolidCylinder(r, L).ShiftFromCentroid({0.0, 0.0, L / 2.0});
  EXPECT_TRUE(G2.IsApprox(G_expected));
  EXPECT_TRUE(G2.CouldBePhysicallyValid());
}

// Tests that we can instantiate a unit inertia with AutoDiffScalar and
// we can perform some basic operations with it.
// As an example, we define the unit inertia G_B of a body B. The
// orientation of this body in the world frame W is given by the time dependent
// rotation R_WB = Rz(theta(t)) about the z-axis with angle theta(t).
// The time derivative of theta(t) is the angular velocity wz.
// We then re-express the inertia of B in the world frame and verify the value
// of its time derivative with the expected result.
GTEST_TEST(UnitInertia, AutoDiff) {
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

  // Construct a unit inertia in the frame of a body B.
  double Ix(1.0), Iy(2.0), Iz(3.0);
  UnitInertia<ADScalar> G_B(Ix, Iy, Iz);

  // Assume B has a pose rotated +20 degrees about z with respect to the
  // world frame W. The body rotates with angular velocity wz in the z-axis.
  const double angle_value = 20 * M_PI / 180.0;
  const double wz = 1.0;  // Angular velocity in the z-axis.

  ADScalar angle = angle_value;
  angle.derivatives()[0] = wz;
  Matrix3<ADScalar> R_WB =
      (AngleAxis<ADScalar>(angle, Vector3d::UnitZ())).toRotationMatrix();

  // Split the unit inertia into two Matrix3d; one with the values and
  // another one with the time derivatives.
  Matrix3<double> Rvalue_WB, Rdot_WB;
  extract_derivatives(R_WB, Rvalue_WB, Rdot_WB);

  // The time derivative of the rotation matrix should be:
  //  Rdot = w× * R, with w the angular velocity.
  // Therefore we have w× = Rdot * R.transpose().
  Matrix3<double> wcross = Rdot_WB * Rvalue_WB.transpose();
  Matrix3<double> wcross_expected;
  wcross_expected << 0.0,  -wz, 0.0,
                      wz,  0.0, 0.0,
                     0.0,  0.0, 0.0;
  EXPECT_TRUE(wcross.isApprox(
      wcross_expected, Eigen::NumTraits<double>::epsilon()));

  // Re-express inertia into another frame.
  const UnitInertia<ADScalar> I_W = G_B.ReExpress(R_WB);

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

  EXPECT_TRUE(Idot_W.isApprox(
      Idot_W_expected, Eigen::NumTraits<double>::epsilon()));
}

// The code below is in support of the goal to use a unit-test to confirm that
// disallowed operators have not been introduced. This uses SFINAE template
// trickery to introduce compile-time code that can be evaluated at run-time.
// If the operator is introduced, the "failing" version of the method will be
// instantiated and invoked. Otherwise, only the passing version will be
// instantiated. There is a variation of this block of code for each disallowed
// operator.
//
// This template generates an l-value at compile time for the templates below.
template <class T>
struct GenerateLValue { T& get_thing(); };

// This overload gets chosen if the *= double would compile.
template <typename T,
    typename = decltype(GenerateLValue<T>().get_thing() *= 1.)>
bool has_times_equal_helper(int) { return true; }

// This overload gets chosen if the above can't compile.
// It is made to take any other argument but the above is a better match to an
// int argument if it got compiled, and therefore gets selected for a class with
// an operator*=() defined.
template <typename T>
bool has_times_equal_helper(...) { return false; }

// This method returns true at runtime if type T has an operator*=().
template <typename T>
bool has_times_equal() { return has_times_equal_helper<T>(1); }

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
template <typename T,
    typename = decltype(GenerateLValue<T>().get_thing() /= 1.)>
bool has_divide_equal_helper(int) { return true; }
template <typename T>
bool has_divide_equal_helper(...) { return false; }
template <typename T>
bool has_divide_equal() { return has_divide_equal_helper<T>(1); }

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
template <typename T,
    typename = decltype(GenerateLValue<T>().get_thing() += T())>
bool has_plus_equal_helper(int) { return true; }
template <typename T>
bool has_plus_equal_helper(...) { return false; }
template <typename T>
bool has_plus_equal() { return has_plus_equal_helper<T>(1); }

// Tests that operator+=() is indeed not available for a UnitInertia while it is
// available for a general RotationalInertia.
GTEST_TEST(UnitInertia, PlusEqualAnInertia) {
  // While we can add a RotationalInertia to a RotationalInertia....
  EXPECT_TRUE(has_plus_equal<RotationalInertia<double>>());

  // ... we cannot perform the same operation on a UnitInertia.
  EXPECT_FALSE(has_plus_equal<UnitInertia<double>>());
}

}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
