#include "drake/multibody/contact_solvers/sap/sap_weld_constraint.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/sap/expect_equal.h"
#include "drake/multibody/contact_solvers/sap/validate_constraint_gradients.h"

using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

// These Jacobian matrices have arbitrary values for testing. We specify the
// size of the matrix in the name, e.g. J32 is of size 3x2.
// clang-format off
const MatrixXd J62_W =
    (MatrixXd(6, 2) << 2, 1,
                       1, 2,
                       5, 4,
                       3, 3,
                       6, 5,
                       4, 6).finished();

const MatrixXd J64_W =
    (MatrixXd(6, 4) << 7, 1, 2, 3,
                       1, 8, 4, 5,
                       2, 4, 9, 6,
                       3, 5, 3, 1,
                       8, 2, 1, 4,
                       1, 2, 4, 3).finished();
// clang-format on

const double kEps = std::numeric_limits<double>::epsilon();
const double kInf = std::numeric_limits<double>::infinity();
const double theta = 0.1;

template <typename T = double>
typename SapWeldConstraint<T>::Kinematics MakeArbitraryKinematics(
    int num_cliques) {
  const int objectA = 12;
  RigidTransform<T> X_WP(RotationMatrix<T>::MakeZRotation(theta),
                         Vector3d(1., 2., 3.));
  Vector3<T> p_AP_W(4., 5., 6.);
  const int objectB = 5;
  RigidTransform<T> X_WQ(RotationMatrix<T>::MakeZRotation(-theta),
                         Vector3d(7., 8., 9.));
  Vector3<T> p_BQ_W(10., 11., 12.);
  const int clique0 = 3;
  const int clique1 = 12;
  auto J_AmBm_W = (num_cliques == 1) ? SapConstraintJacobian<T>(clique0, J62_W)
                                     : SapConstraintJacobian<T>(clique0, J62_W,
                                                                clique1, J64_W);
  return typename SapWeldConstraint<T>::Kinematics{
      objectA, X_WP, p_AP_W, objectB, X_WQ, p_BQ_W, J_AmBm_W};
}

void ExpectEqual(const SapWeldConstraint<double>& c1,
                 const SapWeldConstraint<double>& c2) {
  ExpectBaseIsEqual(c1, c2);
  // Holonomic constraint members.
  EXPECT_EQ(c1.parameters(), c2.parameters());
  EXPECT_EQ(c1.constraint_function(), c2.constraint_function());
  EXPECT_EQ(c1.bias(), c2.bias());
  // Weld constraint members.
  EXPECT_EQ(c1.kinematics(), c2.kinematics());
}

GTEST_TEST(SapWeldConstraint, SingleCliqueConstraint) {
  const int num_cliques = 1;
  const SapWeldConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapWeldConstraint<double> c(kinematics);

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.num_constraint_equations(), 6);
  EXPECT_EQ(c.num_cliques(), 1);
  EXPECT_EQ(c.first_clique(), c.kinematics().jacobian().clique(0));
  EXPECT_THROW(c.second_clique(), std::exception);
  EXPECT_TRUE(CompareMatrices(c.first_clique_jacobian().MakeDenseMatrix(),
                              J62_W, kEps, MatrixCompareType::relative));
  EXPECT_THROW(c.second_clique_jacobian(), std::exception);

  // Check default SapHolonomicConstraint::Parameters values are set.
  EXPECT_EQ(c.parameters().impulse_lower_limits(), -kInf * Vector6d::Ones());
  EXPECT_EQ(c.parameters().impulse_upper_limits(), kInf * Vector6d::Ones());
  EXPECT_EQ(c.parameters().stiffnesses(), kInf * Vector6d::Ones());
  EXPECT_EQ(c.parameters().relaxation_times(), Vector6d::Zero());
  // This value is hard-coded in the source. This test serves as a brake to
  // prevent the value changing without notification. Changing this value
  // would lead to a behavior change and shouldn't happen silently.
  EXPECT_EQ(c.parameters().beta(), 0.1);
  EXPECT_EQ(c.parameters().num_constraint_equations(), 6);
}

GTEST_TEST(SapWeldConstraint, TwoCliquesConstraint) {
  const int num_cliques = 2;
  const SapWeldConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapWeldConstraint<double> c(kinematics);

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.num_constraint_equations(), 6);
  EXPECT_EQ(c.num_cliques(), 2);
  EXPECT_EQ(c.first_clique(), c.kinematics().jacobian().clique(0));
  EXPECT_EQ(c.second_clique(), c.kinematics().jacobian().clique(1));
  EXPECT_TRUE(CompareMatrices(c.first_clique_jacobian().MakeDenseMatrix(),
                              J62_W, kEps, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(c.second_clique_jacobian().MakeDenseMatrix(),
                              J64_W, kEps, MatrixCompareType::relative));

  // Check default SapHolonomicConstraint::Parameters values are set.
  EXPECT_EQ(c.parameters().impulse_lower_limits(), -kInf * Vector6d::Ones());
  EXPECT_EQ(c.parameters().impulse_upper_limits(), kInf * Vector6d::Ones());
  EXPECT_EQ(c.parameters().stiffnesses(), kInf * Vector6d::Ones());
  EXPECT_EQ(c.parameters().relaxation_times(), Vector6d::Zero());
  // This value is hard-coded in the source. This test serves as a brake to
  // prevent the value changing without notification. Changing this value
  // would lead to a behavior change and shouldn't happen silently.
  EXPECT_EQ(c.parameters().beta(), 0.1);
  EXPECT_EQ(c.parameters().num_constraint_equations(), 6);
}

// This method validates analytical gradients implemented by
// SapWeldConstraint using automatic differentiation.
void ValidateProjection(const Vector6d& vc) {
  // Arbitrary kinematic values.
  const int num_cliques = 1;
  const SapWeldConstraint<AutoDiffXd>::Kinematics kin_ad =
      MakeArbitraryKinematics<AutoDiffXd>(num_cliques);

  // Instantiate constraint on AutoDiffXd for automatic differentiation.
  SapWeldConstraint<AutoDiffXd> c(kin_ad);

  // Verify cost gradients using AutoDiffXd.
  ValidateConstraintGradients(c, vc);
}

GTEST_TEST(SapWeldConstraint, Gradients) {
  // Arbitrary set of vc values.
  {
    const Vector6d vc =
        (Vector6d() << -1.2, 3.4, 5.6, -7.8, 9.1, -2.3).finished();
    ValidateProjection(vc);
  }
  {
    const Vector6d vc = (Vector6d() << 6, -5, 4, 3, -2, 1).finished();
    ValidateProjection(vc);
  }
  {
    const Vector6d vc = (Vector6d() << 0, 0.1, -0.2, 0.3, 0.4, -0.5).finished();
    ValidateProjection(vc);
  }
}

GTEST_TEST(SapWeldConstraint, SingleCliqueConstraintClone) {
  const int num_cliques = 1;
  const SapWeldConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapWeldConstraint<double> c(kinematics);

  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone = dynamic_pointer_cast<SapWeldConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(c, *clone);
}

GTEST_TEST(SapWeldConstraint, TwoCliquesConstraintClone) {
  const int num_cliques = 2;
  const SapWeldConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapWeldConstraint<double> c(kinematics);

  auto clone = dynamic_pointer_cast<SapWeldConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(c, *clone);

  // Test ToDouble.
  SapWeldConstraint<AutoDiffXd> c_ad(
      MakeArbitraryKinematics<AutoDiffXd>(num_cliques));
  auto clone_from_ad =
      dynamic_pointer_cast<SapWeldConstraint<double>>(c_ad.ToDouble());
  ASSERT_NE(clone_from_ad, nullptr);
  ExpectEqual(c, *clone_from_ad);
}

GTEST_TEST(SapWeldConstraint, AccumulateSpatialImpulses) {
  // Make a weld constraint with an arbitrary kinematics state.
  const int num_cliques = 1;
  const SapWeldConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapWeldConstraint<double> c(kinematics);

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.object(0), c.kinematics().objectA());
  EXPECT_EQ(c.object(1), c.kinematics().objectB());

  // Arbitrary value of the impulse.
  const Vector6d gamma =
      (Vector6d() << -1.2, 3.4, 5.6, -7.8, 9.1, -2.3).finished();

  // Expected spatial impulse on B.
  const SpatialForce<double> gamma_BBm_W(gamma.template head<3>(),
                                         gamma.template tail<3>());
  const Vector3d p_BoBm_W = kinematics.p_BQ_W() - 0.5 * kinematics.p_PoQo_W();
  const SpatialForce<double> gamma_BBo_W = gamma_BBm_W.Shift(-p_BoBm_W);

  // Expected spatial impulse on A.
  const SpatialForce<double> gamma_AAm_W(-gamma.template head<3>(),
                                         -gamma.template tail<3>());
  const Vector3d p_AoAm_W = kinematics.p_AP_W() + 0.5 * kinematics.p_PoQo_W();
  SpatialForce<double> gamma_AAo_W = gamma_AAm_W.Shift(-p_AoAm_W);

  const SpatialForce<double> gamma0_BBo_W(Vector3d(1., 2., 3),
                                          Vector3d(4., 5., 6));
  SpatialForce<double> gamma_accumulated =
      gamma0_BBo_W;  // Initialize to non-zero value.
  const SpatialForce<double> gamma_BBo_W_expected = gamma0_BBo_W + gamma_BBo_W;
  c.AccumulateSpatialImpulses(1, gamma, &gamma_accumulated);
  EXPECT_TRUE(CompareMatrices(
      gamma_accumulated.get_coeffs(), gamma_BBo_W_expected.get_coeffs(),
      std::numeric_limits<double>::epsilon(), MatrixCompareType::relative));

  const SpatialForce<double> gamma0_AAo_W(Vector3d(1., 2., 3),
                                          Vector3d(4., 5., 6));
  gamma_accumulated = gamma0_AAo_W;  // Initialize to non-zero value.
  const SpatialForce<double> gamma_AAo_W_expected = gamma0_AAo_W + gamma_AAo_W;
  c.AccumulateSpatialImpulses(0, gamma, &gamma_accumulated);
  EXPECT_TRUE(CompareMatrices(
      gamma_accumulated.get_coeffs(), gamma_AAo_W_expected.get_coeffs(),
      std::numeric_limits<double>::epsilon(), MatrixCompareType::relative));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
