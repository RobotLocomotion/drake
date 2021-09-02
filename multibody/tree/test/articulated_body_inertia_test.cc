#include "drake/multibody/tree/articulated_body_inertia.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/unit_inertia.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Vector3d;
using Eigen::MatrixXd;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Test default constructor which leaves entries initialized to NaN for a
// quick detection of uninitialized state. Also tests CopyToFullMatrix6().
GTEST_TEST(ArticulatedBodyInertia, DefaultConstructor) {
  ArticulatedBodyInertia<double> P;
  const Matrix6<double> P_matrix = P.CopyToFullMatrix6();
  ASSERT_TRUE(std::all_of(
      P_matrix.data(),
      P_matrix.data() + 36,
      [](double x) { return std::isnan(x); }));
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

  // Construct from spatial inertia.
  const ArticulatedBodyInertia<double> P(M);
  EXPECT_TRUE(P.CopyToFullMatrix6().isApprox(M.CopyToFullMatrix6(), kEpsilon));

  // Construct from matrix.
  const ArticulatedBodyInertia<double> P2(M.CopyToFullMatrix6());
  EXPECT_TRUE(P2.CopyToFullMatrix6().isApprox(M.CopyToFullMatrix6(), kEpsilon));
}

// Tests that we can correctly cast a ArticulatedBodyInertia<double> to a
// ArticulatedBodyInertia<AutoDiffXd>.
// The cast from a ArticulatedBodyInertia<double>, a constant, results in an
// articulated body inertia with zero gradients.
GTEST_TEST(ArticulatedBodyInertia, CastToAutoDiff) {
  const double mass_double = 2.5;
  const Vector3d com_double(0.1, -0.2, 0.3);
  const Vector3d m_double(2.0,  2.3, 2.4);  // m for moments.
  const Vector3d p_double(0.1, -0.1, 0.2);  // p for products.
  const UnitInertia<double> G_double(
      m_double(0), m_double(1), m_double(2), /* moments of inertia */
      p_double(0), p_double(1), p_double(2));/* products of inertia */
  const SpatialInertia<double> M_double(mass_double, com_double, G_double);
  ASSERT_TRUE(M_double.IsPhysicallyValid());

  // Construct articulated body inertia from spatial inertia.
  const ArticulatedBodyInertia<double> P_double(M_double);

  // Cast from double to AutoDiffXd.
  ArticulatedBodyInertia<AutoDiffXd> P_autodiff = P_double.cast<AutoDiffXd>();

  // Verify value and gradient of P_autodiff.
  // Since there are no independent variables in this case the size of all
  // gradients must be zero.

  // Value and gradient of the matrix.
  const auto& matrix_autodiff = P_autodiff.CopyToFullMatrix6();
  auto matrix_value = math::ExtractValue(matrix_autodiff);
  EXPECT_TRUE(matrix_value.isApprox(P_double.CopyToFullMatrix6(), kEpsilon));
  MatrixXd matrix_gradient = math::ExtractGradient(matrix_autodiff);
  ASSERT_EQ(matrix_gradient.size(), 0);
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

  // Shift the spatial inertia and articulated body inertia along an arbitrary
  // vector.
  const Vector3d shift_vector = Vector3d(2, 3, 4);
  const SpatialInertia<double> M_shifted = M.Shift(shift_vector);
  const ArticulatedBodyInertia<double> P_shifted = P.Shift(shift_vector);

  // Shifting should result in the same outcome.
  EXPECT_TRUE(P_shifted.CopyToFullMatrix6().isApprox(
      M_shifted.CopyToFullMatrix6(), kEpsilon));

  // Shift the articulated body inertia back and ensure that it is close to the
  // original articulated body inertia.
  EXPECT_TRUE(P_shifted.Shift(-shift_vector).CopyToFullMatrix6().isApprox(
      P.CopyToFullMatrix6(), kEpsilon));
}

// Test the plus equal operator for adding articulated body inertias.
GTEST_TEST(ArticulatedBodyInertia, PlusEqualOperator) {
  // Spatial inertia for an arbitrary object C.
  double mass_C = 5.3;
  Vector3d p_PCcm_E = Vector3d(0.001, 0.1, 0.02);
  UnitInertia<double> G_CP_E =
      UnitInertia<double>(2.0, 2.3, 2.4, -0.1, 0.2, 0.1);
  SpatialInertia<double> M_CP_E(mass_C, p_PCcm_E, G_CP_E);

  // Spatial inertia for an arbitrary object D.
  double mass_D = 9.7;
  Vector3d p_PDcm_E = Vector3d(0.03, 0.07, 0.01);
  UnitInertia<double> G_DP_E =
      UnitInertia<double>(3.1, 2.5, 3.4, 0.1, -0.2, -0.1);
  SpatialInertia<double> M_DP_E(mass_D, p_PDcm_E, G_DP_E);

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
  EXPECT_TRUE(P_SP_E.CopyToFullMatrix6().isApprox(
      M_SP_E.CopyToFullMatrix6(), kEpsilon));
}

// Test the times equal for operator for multiplying on the left and right
// by arbitrary Eigen matrices of valid sizes.
GTEST_TEST(ArticulatedBodyInertia, TimesOperator) {
  // Spatial inertia for a cube C.
  double Lx = 1.0, Ly = 1.0, Lz = 1.0;  // Cube's lengths.
  double mass = 1.0;  // Cube's mass
  const SpatialInertia<double> M(  // First computed about its centroid.
      mass, Vector3d::Zero(), UnitInertia<double>::SolidBox(Lx, Ly, Lz));

  // Articulated body inertia from spatial inertia.
  const ArticulatedBodyInertia<double> P(M);

  // Extract matrix for comparison.
  const Matrix6<double> P_matrix = P.CopyToFullMatrix6();

  // Verify that multiplying by a matrix on the left and right works as
  // expected.
  Matrix6<double> H1;
  H1 << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6,
        1.1, 1.2, 1.3, 1.4, 1.5, 1.6,
        1.7, 1.8, 1.9, 2.1, 2.2, 2.3,
        -2.4, -2.5, -2.6, -2.7, -2.8, -2.9,
        -3.1, -3.2, -3.3, -3.4, -3.5, -3.6,
        -3.7, -3.8, -3.9, -4.1, -4.2, -4.3;
  EXPECT_TRUE((H1 * P_matrix).isApprox(H1 * P, kEpsilon));
  EXPECT_TRUE((P_matrix * H1).isApprox(P * H1, kEpsilon));

  // Create a column vector representing the across mobilizer Jacobian for
  // a complex joint.
  Vector6<double> H2;
  H2 << 1.2, -2.3, 3.4, 0.0, -5.0, 2.1;

  // Ensure that the results of multiplying by H transpose and H on the left and
  // right are the same as operating directly on the matrix.
  EXPECT_TRUE((H2.transpose() * P_matrix * H2).isApprox(
      H2.transpose() * P * H2, kEpsilon));
}

// Tests support (though limited) for symbolic::Expression.
GTEST_TEST(ArticulatedBodyInertia, Symbolic) {
  // IsPhysicallyValid() supported for numeric types.
  ArticulatedBodyInertia<double> Pd;
  DRAKE_EXPECT_NO_THROW(Pd.IsPhysicallyValid());

  // IsPhysicallyValid() not supported for non-numeric types.
  ArticulatedBodyInertia<symbolic::Expression> Ps;
  EXPECT_ANY_THROW(Ps.IsPhysicallyValid());

  // Invariant checks are a no-op for non-numeric types, allowing us to create
  // symbolic ABIs also in Debug builds. Therefore this test passes successfully
  // even though we supply an invalid matrix (in this case a negative definite
  // matrix.)
  // The same test however is expected to throw when T = double in Debug builds.
  DRAKE_EXPECT_NO_THROW(ArticulatedBodyInertia<symbolic::Expression> Ds(
      -Matrix6<symbolic::Expression>::Identity()));
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      ArticulatedBodyInertia<double> Ds(-Matrix6<double>::Identity()),
      std::runtime_error,
      "The resulting articulated body inertia is not physically "
      "valid.[\\s\\S]*");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
