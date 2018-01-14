#include "drake/multibody/multibody_tree/articulated_body_inertia.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"
#include "drake/multibody/multibody_tree/unit_inertia.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using Eigen::AngleAxisd;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Test default constructor which leaves entries initialized to NaN for a
// quick detection of uninitialized state. Also tests CopyToFullMatrix6().
GTEST_TEST(ArticulatedBodyInertia, DefaultConstructor) {
  ArticulatedBodyInertia<double> P;
  const Matrix6<double> P_matrix = P.CopyToFullMatrix6();
  ASSERT_TRUE(std::all_of(
      P_matrix.data(),
      P_matrix.data() + 36,
      [](double x) { return std::isnan(x); }
  ));
}

// Construct a non-trivial articulated body inertia from a spatial inertia,
// testing both construction from matrix and construction from spatial inertia.
GTEST_TEST(ArticulatedBodyInertia, ConstructionNonTrivial) {
  const double mass = 2.5;
  const Vector3d com(0.1, -0.2, 0.3);
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);  // p for products.
  const UnitInertia<double> G(m(0), m(1), m(2),  /* moments of inertia */
                              p(0), p(1), p(2)); /* products of inertia */
  const SpatialInertia<double> M(mass, com, G);

  // Construct from matrix directly.
  const Matrix6<double> M_matrix = M.CopyToFullMatrix6();
  const ArticulatedBodyInertia<double> P_direct(M_matrix);
  EXPECT_TRUE(P_direct.CopyToFullMatrix6().isApprox(M_matrix, kEpsilon));

  // Construct from spatial inertia (indirectly).
  const ArticulatedBodyInertia<double> P_indirect(M);
  EXPECT_TRUE(P_direct.CopyToFullMatrix6().isApprox(M_matrix, kEpsilon));
}

// Test the shift method by comparing to spatial inertia's shift method.
GTEST_TEST(ArticulatedBodyInertia, Shift) {
  const double mass = 2.5;
  const Vector3d com(0.1, -0.2, 0.3);
  const Vector3d m(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p(0.1, -0.1, 0.2);  // p for products.
  const UnitInertia<double> G(m(0), m(1), m(2),  /* moments of inertia */
                              p(0), p(1), p(2)); /* products of inertia */
  const SpatialInertia<double> M(mass, com, G);
  const ArticulatedBodyInertia<double> P(M);

  // Shift the spatial inertia and articulated body inertia one unit in the x
  // direction along the x axis.
  const SpatialInertia<double> M_shifted = M.Shift(Vector3d::UnitX());
  const ArticulatedBodyInertia<double> P_shifted = P.Shift(Vector3d::UnitX());

  // Shifting should result in the same outcome.
  EXPECT_TRUE(P_shifted.CopyToFullMatrix6()
                  .isApprox(M_shifted.CopyToFullMatrix6(), kEpsilon));

  // Shift the articulated body inertia back and ensure that it is close to the
  // original articulated body inertia.
  EXPECT_TRUE(P_shifted.Shift(-Vector3d::UnitX()).CopyToFullMatrix6()
                  .isApprox(P.CopyToFullMatrix6(), kEpsilon));
}

// Test the re-express method by comparing to spatial inertia's re-express
// method.
GTEST_TEST(ArticulatedBodyInertia, ReExpress) {
  // Spatial inertia for a cube C computed about a point P and expressed in a
  // frame E.
  const double Lx = 0.2, Ly = 1.0, Lz = 0.5;  // Cube's lengths.
  const double mass = 1.3;  // Cube's mass
  SpatialInertia<double> M_CP_E(  // First computed about its centroid.
      mass, Vector3d::Zero(), UnitInertia<double>::SolidBox(Lx, Ly, Lz));

  // Articulated body inertia from spatial inertia.
  ArticulatedBodyInertia<double> P_CP_E(M_CP_E);

  // Shift to point P placed one unit in the y direction along the y axis in
  // frame E.
  M_CP_E.ShiftInPlace(Vector3d::UnitY());
  P_CP_E.ShiftInPlace(Vector3d::UnitY());

  // Place B rotated +90 degrees about W's x-axis.
  Matrix3<double> R_WE =
      AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();

  SpatialInertia<double> M_CP_W = M_CP_E.ReExpress(R_WE);
  ArticulatedBodyInertia<double> P_CP_W = P_CP_E.ReExpress(R_WE);

  // Re-expressing should result in the same outcome.
  EXPECT_TRUE(P_CP_W.CopyToFullMatrix6()
                  .isApprox(M_CP_W.CopyToFullMatrix6(), kEpsilon));
}

// Test the plus equal operator for adding articulated body inertias.
GTEST_TEST(ArticulatedBodyInertia, PlusEqualOperator) {
  // Spatial inertia for a cube C computed about a point P and expressed in a
  // frame E.
  double Lx = 0.2, Ly = 1.0, Lz = 0.5;  // Cube's lengths.
  double mass = 1.3;  // Cube's mass
  SpatialInertia<double> M_CP_E(  // First computed about its centroid.
      mass, Vector3d::Zero(), UnitInertia<double>::SolidBox(Lx, Ly, Lz));

  // Spatial inertia for a cube D computed about a point P and expressed in a
  // frame E.
  Lx = 1.1, Ly = 0.2, Lz = 0.9;  // Cube's lengths.
  mass = 0.2;  // Cube's mass
  SpatialInertia<double> M_DP_E(  // First computed about its centroid.
      mass, Vector3d::Zero(), UnitInertia<double>::SolidBox(Lx, Ly, Lz));

  // Articulated body inertias from spatial inertias.
  ArticulatedBodyInertia<double> P_CP_E(M_CP_E);
  ArticulatedBodyInertia<double> P_DP_E(M_DP_E);

  // Create new inertias which represent the composite body S consisting of
  // cubes C and D.
  SpatialInertia<double> M_SP_E(M_CP_E);
  ArticulatedBodyInertia<double> P_SP_E(P_CP_E);

  // Add the inertia of cube D to cube C.
  M_SP_E += M_DP_E;
  P_SP_E += P_DP_E;

  // Ensure both inertias are the same.
  EXPECT_TRUE(P_SP_E.CopyToFullMatrix6()
                  .isApprox(M_SP_E.CopyToFullMatrix6(), kEpsilon));
}

// Test the times equal for operator for multiplying on the left and right
// by arbitrary Eigen matrices of valid sizes.
GTEST_TEST(ArticulatedBodyInertia, TimesEqualOperator) {
  // Spatial inertia for a cube C.
  double Lx = 1.0, Ly = 1.0, Lz = 1.0;  // Cube's lengths.
  double mass = 1.0;  // Cube's mass
  const SpatialInertia<double> M(  // First computed about its centroid.
      mass, Vector3d::Zero(), UnitInertia<double>::SolidBox(Lx, Ly, Lz));

  // Articulated body inertia from spatial inertia.
  const ArticulatedBodyInertia<double> P(M);

  // Extract matrix for comparison.
  const Matrix6<double> P_matrix = P.CopyToFullMatrix6();

  // Verify that multiplying by the identity matrix on the left and right does
  // not change P.
  EXPECT_TRUE(P_matrix.isApprox(Matrix6<double>::Identity() * P, kEpsilon));
  EXPECT_TRUE(P_matrix.isApprox(P * Matrix6<double>::Identity(), kEpsilon));

  // Create a column vector representing the across mobilizer Jacobian for
  // a revolute joint around the x-axis.
  Vector6<double> H;
  H << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Ensure that the results of multiplying by H transpose and H on the left and
  // right are the same as operating directly on the the matrix.
  EXPECT_TRUE((H.transpose() * P_matrix * H)
                  .isApprox(H.transpose() * P * H, kEpsilon));
}

}  // namespace
}  // namespace multibody
}  // namespace drake