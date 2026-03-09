#include "drake/multibody/contact_solvers/sap/sap_distance_constraint.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/sap/expect_equal.h"
#include "drake/multibody/contact_solvers/sap/validate_constraint_gradients.h"

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
const MatrixXd J32 =
    (MatrixXd(3, 2) << 2, 1,
                       1, 2,
                       1, 2).finished();

const MatrixXd J34 =
    (MatrixXd(3, 4) << 7, 1, 2, 3,
                       1, 8, 4, 5,
                       2, 4, 9, 6).finished();
// clang-format on

template <typename T = double>
typename SapDistanceConstraint<T>::ComplianceParameters
MakeArbitraryParameters() {
  const T stiffness = 1.0e5;
  const T damping = 1.0e3;
  return typename SapDistanceConstraint<T>::ComplianceParameters{stiffness,
                                                                 damping};
}

template <typename T = double>
typename SapDistanceConstraint<T>::Kinematics MakeArbitraryKinematics(
    int num_cliques) {
  const int objectA = 12;
  Vector3<T> p_WP(1., 2., 3.);
  Vector3<T> p_AP_W(4., 5., 6.);
  const int objectB = 5;
  Vector3<T> p_WQ(7., 8., 9.);
  Vector3<T> p_BQ_W(10., 11., 12.);
  const int clique0 = 3;
  const int clique1 = 12;
  const T length = 0.35;
  auto J_PQ_W = (num_cliques == 1)
                    ? SapConstraintJacobian<T>(clique0, J32)
                    : SapConstraintJacobian<T>(clique0, J32, clique1, J34);
  return typename SapDistanceConstraint<T>::Kinematics{
      objectA, p_WP, p_AP_W, objectB, p_WQ, p_BQ_W, length, J_PQ_W};
}

void ExpectEqual(const SapDistanceConstraint<double>& c1,
                 const SapDistanceConstraint<double>& c2) {
  ExpectBaseIsEqual(c1, c2);
  EXPECT_EQ(c1.kinematics(), c2.kinematics());
  EXPECT_EQ(c1.compliance_parameters(), c2.compliance_parameters());
}

GTEST_TEST(SapDistanceConstraint, SingleCliqueConstraint) {
  const int num_cliques = 1;
  const SapDistanceConstraint<double>::ComplianceParameters parameters =
      MakeArbitraryParameters();
  const SapDistanceConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapDistanceConstraint<double> c(kinematics, parameters);

  const Vector3d p = kinematics.p_WQ() - kinematics.p_WP();
  const Vector3d p_hat = p.normalized();

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.num_constraint_equations(), 1);
  EXPECT_EQ(c.num_cliques(), 1);
  EXPECT_EQ(c.first_clique(), kinematics.jacobian().clique(0));
  EXPECT_THROW(c.second_clique(), std::exception);
  EXPECT_EQ(c.first_clique_jacobian().MakeDenseMatrix(),
            p_hat.transpose() * J32);
  EXPECT_THROW(c.second_clique_jacobian(), std::exception);
  EXPECT_EQ(c.length(), kinematics.length());
  EXPECT_EQ(c.compliance_parameters().stiffness(), parameters.stiffness());
  EXPECT_EQ(c.compliance_parameters().damping(), parameters.damping());
}

GTEST_TEST(SapDistanceConstraint, TwoCliquesConstraint) {
  const int num_cliques = 2;
  const SapDistanceConstraint<double>::ComplianceParameters parameters =
      MakeArbitraryParameters();
  const SapDistanceConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapDistanceConstraint<double> c(kinematics, parameters);

  const Vector3d p = kinematics.p_WQ() - kinematics.p_WP();
  const Vector3d p_hat = p.normalized();

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.num_constraint_equations(), 1);
  EXPECT_EQ(c.num_cliques(), 2);
  EXPECT_EQ(c.first_clique(), kinematics.jacobian().clique(0));
  EXPECT_EQ(c.second_clique(), kinematics.jacobian().clique(1));
  EXPECT_EQ(c.first_clique_jacobian().MakeDenseMatrix(),
            p_hat.transpose() * J32);
  EXPECT_EQ(c.second_clique_jacobian().MakeDenseMatrix(),
            p_hat.transpose() * J34);
  EXPECT_EQ(c.length(), kinematics.length());
  EXPECT_EQ(c.compliance_parameters().stiffness(), parameters.stiffness());
  EXPECT_EQ(c.compliance_parameters().damping(), parameters.damping());
}

// This method validates analytical gradients implemented by
// SapDistanceConstraint using automatic differentiation.
void ValidateProjection(
    const SapDistanceConstraint<double>::ComplianceParameters& p,
    const Vector1d& vc) {
  SapDistanceConstraint<AutoDiffXd>::ComplianceParameters p_ad(p.stiffness(),
                                                               p.damping());

  // Arbitrary kinematic values.
  const int num_cliques = 1;
  const SapDistanceConstraint<AutoDiffXd>::Kinematics kin_ad =
      MakeArbitraryKinematics<AutoDiffXd>(num_cliques);

  // Instantiate constraint on AutoDiffXd for automatic differentiation.
  SapDistanceConstraint<AutoDiffXd> c(kin_ad, p_ad);

  // Verify cost gradients using AutoDiffXd.
  ValidateConstraintGradients(c, vc);
}

GTEST_TEST(SapDistanceConstraint, Gradients) {
  // An arbitrary set of parameters.
  SapDistanceConstraint<double>::ComplianceParameters p(1.0e3, 1.0e2);

  // Arbitrary set of vc values.
  {
    const Vector1d vc(-1.4);
    ValidateProjection(p, vc);
  }
  {
    const Vector1d vc(2.3);
    ValidateProjection(p, vc);
  }
  {
    const Vector1d vc(6.2);
    ValidateProjection(p, vc);
  }
}

GTEST_TEST(SapDistanceConstraint, SingleCliqueConstraintClone) {
  const int num_cliques = 1;
  const SapDistanceConstraint<double>::ComplianceParameters parameters =
      MakeArbitraryParameters();
  const SapDistanceConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapDistanceConstraint<double> c(kinematics, parameters);

  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone = dynamic_pointer_cast<SapDistanceConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(c, *clone);

  // Test ToDouble.
  SapDistanceConstraint<AutoDiffXd> c_ad(
      MakeArbitraryKinematics<AutoDiffXd>(num_cliques),
      MakeArbitraryParameters<AutoDiffXd>());
  auto clone_from_ad =
      dynamic_pointer_cast<SapDistanceConstraint<double>>(c_ad.ToDouble());
  ASSERT_NE(clone_from_ad, nullptr);
  ExpectEqual(c, *clone_from_ad);
}

GTEST_TEST(SapDistanceConstraint, TwoCliquesConstraintClone) {
  const int num_cliques = 2;
  const SapDistanceConstraint<double>::ComplianceParameters parameters =
      MakeArbitraryParameters();
  const SapDistanceConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapDistanceConstraint<double> c(kinematics, parameters);

  auto clone = dynamic_pointer_cast<SapDistanceConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(c, *clone);

  // Test ToDouble.
  SapDistanceConstraint<AutoDiffXd> c_ad(
      MakeArbitraryKinematics<AutoDiffXd>(num_cliques),
      MakeArbitraryParameters<AutoDiffXd>());
  auto clone_from_ad =
      dynamic_pointer_cast<SapDistanceConstraint<double>>(c_ad.ToDouble());
  ASSERT_NE(clone_from_ad, nullptr);
  ExpectEqual(c, *clone_from_ad);
}

GTEST_TEST(SapDistanceConstraint, AccumulateSpatialImpulses) {
  // Make a distance constraint with an arbitrary kinematics state and set of
  // parameters, irrelevant for this test but needed at construction.
  const int num_cliques = 1;
  const SapDistanceConstraint<double>::ComplianceParameters parameters =
      MakeArbitraryParameters();
  const SapDistanceConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapDistanceConstraint<double> c(kinematics, parameters);

  // Vector from P to Q, and its normalized versor.
  const Vector3d p_PQ_W = kinematics.p_WQ() - kinematics.p_WP();
  const Vector3d p_hat_W = p_PQ_W.normalized();

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.object(0), kinematics.objectA());
  EXPECT_EQ(c.object(1), kinematics.objectB());

  // Arbitrary value of the impulse.
  const Vector1d gamma(1.2345);

  // Expected spatial impulse on B.
  const Vector3d f_B_W = gamma(0) * p_hat_W;
  const Vector3d t_Bo_W = kinematics.p_BQ_W().cross(f_B_W);
  const SpatialForce<double> F_Bo_W(t_Bo_W, f_B_W);

  // Expected spatial impulse on A.
  const Vector3d f_A_W = -f_B_W;
  const Vector3d t_Ao_W = kinematics.p_AP_W().cross(f_A_W);
  const SpatialForce<double> F_Ao_W(t_Ao_W, f_A_W);

  const SpatialForce<double> F0(Vector3d(1., 2., 3), Vector3d(4., 5., 6));
  SpatialForce<double> Faccumulated = F0;  // Initialize to non-zero value.
  SpatialForce<double> F_Bo_W_expected = F0 + F_Bo_W;
  c.AccumulateSpatialImpulses(1, gamma, &Faccumulated);
  EXPECT_TRUE(CompareMatrices(
      Faccumulated.get_coeffs(), F_Bo_W_expected.get_coeffs(),
      std::numeric_limits<double>::epsilon(), MatrixCompareType::relative));

  Faccumulated = F0;  // Initialize to non-zero value.
  SpatialForce<double> F_Ao_W_expected = F0 + F_Ao_W;
  c.AccumulateSpatialImpulses(0, gamma, &Faccumulated);
  EXPECT_TRUE(CompareMatrices(
      Faccumulated.get_coeffs(), F_Ao_W_expected.get_coeffs(),
      std::numeric_limits<double>::epsilon(), MatrixCompareType::relative));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
