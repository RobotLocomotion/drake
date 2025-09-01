#include "drake/multibody/contact_solvers/sap/sap_coupler_constraint.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/sap/expect_equal.h"
#include "drake/multibody/contact_solvers/sap/validate_constraint_gradients.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

template <typename T = double>
typename SapCouplerConstraint<T>::Kinematics MakeArbitraryKinematics(
    int num_cliques) {
  const int clique0 = 4;
  const int clique_dof0 = 2;
  const int clique_nv0 = 5;
  const T q0 = 0.5;
  const int clique1 = (num_cliques == 1 ? clique0 : 9);
  const int clique_dof1 = 3;
  const int clique_nv1 = (num_cliques == 1 ? clique_nv0 : 7);
  const T q1 = 2.3;
  const T gear_ratio = 1.2;
  const T offset = 0.7;

  return typename SapCouplerConstraint<T>::Kinematics{
      clique0,     clique_dof0, clique_nv0, q0,         clique1,
      clique_dof1, clique_nv1,  q1,         gear_ratio, offset};
}

void ExpectEqual(const SapCouplerConstraint<double>& c1,
                 const SapCouplerConstraint<double>& c2) {
  ExpectBaseIsEqual(c1, c2);
  EXPECT_EQ(c1.kinematics(), c2.kinematics());
}

GTEST_TEST(SapCouplerConstraint, SingleCliqueConstraint) {
  const int num_cliques = 1;
  const SapCouplerConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapCouplerConstraint<double> c(kinematics);

  EXPECT_EQ(c.num_objects(), 0);
  EXPECT_EQ(c.num_constraint_equations(), 1);
  EXPECT_EQ(c.num_cliques(), 1);
  EXPECT_EQ(c.first_clique(), kinematics.clique0);
  EXPECT_EQ(c.num_velocities(0), kinematics.clique_nv0);
  EXPECT_THROW(c.second_clique(), std::exception);

  const MatrixX<double> J0 = c.first_clique_jacobian().MakeDenseMatrix();

  // The Jacobian should contain only two entries corresponding to the
  // constraints two dofs.
  EXPECT_EQ(J0(kinematics.clique_dof0), 1.0);
  EXPECT_EQ(J0(kinematics.clique_dof1), -kinematics.gear_ratio);
  EXPECT_EQ(J0.sum(), 1.0 - kinematics.gear_ratio);

  EXPECT_THROW(c.second_clique_jacobian(), std::exception);
}

GTEST_TEST(SapCouplerConstraint, TwoCliquesConstraint) {
  const int num_cliques = 2;
  const SapCouplerConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapCouplerConstraint<double> c(kinematics);

  EXPECT_EQ(c.num_objects(), 0);
  EXPECT_EQ(c.num_constraint_equations(), 1);
  EXPECT_EQ(c.num_cliques(), 2);
  EXPECT_EQ(c.first_clique(), kinematics.clique0);
  EXPECT_EQ(c.second_clique(), kinematics.clique1);
  EXPECT_EQ(c.num_velocities(0), kinematics.clique_nv0);
  EXPECT_EQ(c.num_velocities(1), kinematics.clique_nv1);

  const MatrixX<double> J0 = c.first_clique_jacobian().MakeDenseMatrix();
  const MatrixX<double> J1 = c.second_clique_jacobian().MakeDenseMatrix();

  // J0 and J1 should each contain exactly one entry corresponding to dof 0 or
  // dof 1, respectively.
  EXPECT_EQ(J0(kinematics.clique_dof0), 1.0);
  EXPECT_EQ(J0.sum(), 1.0);
  EXPECT_EQ(J1(kinematics.clique_dof1), -kinematics.gear_ratio);
  EXPECT_EQ(J1.sum(), -kinematics.gear_ratio);
}

// This method validates analytical gradients implemented by
// SapCouplerConstraint using automatic differentiation.
void ValidateProjection(const Vector1d& vc) {
  // Arbitrary kinematic values.
  const int num_cliques = 1;
  const SapCouplerConstraint<AutoDiffXd>::Kinematics kin_ad =
      MakeArbitraryKinematics<AutoDiffXd>(num_cliques);

  // Instantiate constraint on AutoDiffXd for automatic differentiation.
  SapCouplerConstraint<AutoDiffXd> c(kin_ad);

  // Verify cost gradients using AutoDiffXd.
  ValidateConstraintGradients(c, vc);
}

GTEST_TEST(SapCouplerConstraint, Gradients) {
  // Arbitrary set of vc values.
  {
    const Vector1d vc(-1.4);
    ValidateProjection(vc);
  }
  {
    const Vector1d vc(2.3);
    ValidateProjection(vc);
  }
  {
    const Vector1d vc(6.2);
    ValidateProjection(vc);
  }
}

GTEST_TEST(SapCouplerConstraint, SingleCliqueConstraintClone) {
  const int num_cliques = 1;
  const SapCouplerConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapCouplerConstraint<double> c(kinematics);

  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone = dynamic_pointer_cast<SapCouplerConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(c, *clone);

  // Test ToDouble.
  SapCouplerConstraint<AutoDiffXd> c_ad(
      MakeArbitraryKinematics<AutoDiffXd>(num_cliques));
  auto clone_from_ad =
      dynamic_pointer_cast<SapCouplerConstraint<double>>(c_ad.ToDouble());
  ASSERT_NE(clone_from_ad, nullptr);
  ExpectEqual(c, *clone_from_ad);
}

GTEST_TEST(SapCouplerConstraint, TwoCliquesConstraintClone) {
  const int num_cliques = 2;
  const SapCouplerConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapCouplerConstraint<double> c(kinematics);

  auto clone = dynamic_pointer_cast<SapCouplerConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(c, *clone);

  // Test ToDouble.
  SapCouplerConstraint<AutoDiffXd> c_ad(
      MakeArbitraryKinematics<AutoDiffXd>(num_cliques));
  auto clone_from_ad =
      dynamic_pointer_cast<SapCouplerConstraint<double>>(c_ad.ToDouble());
  ASSERT_NE(clone_from_ad, nullptr);
  ExpectEqual(c, *clone_from_ad);
}

GTEST_TEST(SapCouplerConstraint, SingleCliqueAccumulateGeneralizedImpulses) {
  // Make a coupler constraint with an arbitrary kinamtics state, irrelevant for
  // this test but needed at construction.
  const int num_cliques = 1;
  const SapCouplerConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapCouplerConstraint<double> c(kinematics);

  // Arbitrary value of the impulse.
  const Vector1d gamma(1.2345);

  // Expected generalized impulse on clique 0.
  VectorXd tau_clique0 = VectorXd::Zero(kinematics.clique_nv0);
  tau_clique0(kinematics.clique_dof0) = gamma(0);

  // Expected generalized impulse on clique 1.
  VectorXd tau_clique1 = VectorXd::Zero(kinematics.clique_nv1);
  tau_clique1(kinematics.clique_dof1) = -kinematics.gear_ratio * gamma(0);

  const VectorXd tau_0 =
      VectorXd::LinSpaced(kinematics.clique_nv0, 1, kinematics.clique_nv0);

  VectorXd tau_accumulated = tau_0;  // Initialize to non-zero value.
  const VectorXd tau_expected = tau_0 + tau_clique0 + tau_clique1;
  c.AccumulateGeneralizedImpulses(0, gamma, &tau_accumulated);
  EXPECT_TRUE(CompareMatrices(tau_accumulated, tau_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));

  // Calling on clique one when num_cliques() == 1 should throw an exception.
  EXPECT_THROW(c.AccumulateGeneralizedImpulses(1, gamma, &tau_accumulated),
               std::exception);
}

GTEST_TEST(SapCouplerConstraint, TwoCliquesAccumulateGeneralizedImpulses) {
  // Make a coupler constraint with an arbitrary kinamtics state, irrelevant for
  // this test but needed at construction.
  const int num_cliques = 2;
  const SapCouplerConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapCouplerConstraint<double> c(kinematics);

  // Arbitrary value of the impulse.
  const Vector1d gamma(1.2345);

  // Expected generalized impulse on clique 0.
  VectorXd tau_clique0 = VectorXd::Zero(kinematics.clique_nv0);
  tau_clique0(kinematics.clique_dof0) = gamma(0);

  // Expected generalized impulse on clique 1.
  VectorXd tau_clique1 = VectorXd::Zero(kinematics.clique_nv1);
  tau_clique1(kinematics.clique_dof1) = -kinematics.gear_ratio * gamma(0);

  const VectorXd tau_0 =
      VectorXd::LinSpaced(kinematics.clique_nv0, 1, kinematics.clique_nv0);

  VectorXd tau_accumulated = tau_0;  // Initialize to non-zero value.
  VectorXd tau_expected = tau_0 + tau_clique0;
  c.AccumulateGeneralizedImpulses(0, gamma, &tau_accumulated);
  EXPECT_TRUE(CompareMatrices(tau_accumulated, tau_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));

  const VectorXd tau_1 =
      VectorXd::LinSpaced(kinematics.clique_nv1, 1, kinematics.clique_nv1);

  tau_accumulated = tau_1;  // Initialize to non-zero value.
  tau_expected = tau_1 + tau_clique1;
  c.AccumulateGeneralizedImpulses(1, gamma, &tau_accumulated);
  EXPECT_TRUE(CompareMatrices(tau_accumulated, tau_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
