#include "drake/multibody/contact_solvers/sap/sap_ball_constraint.h"

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
typename SapBallConstraint<T>::Kinematics MakeArbitraryKinematics(
    int num_cliques) {
  const int objectA = 12;
  Vector3<T> p_WP(1., 2., 3.);
  Vector3<T> p_AP_W(4., 5., 6.);
  const int objectB = 5;
  Vector3<T> p_WQ(7., 8., 9.);
  Vector3<T> p_BQ_W(10., 11., 12.);
  const int clique0 = 3;
  const int clique1 = 12;
  auto J_PQ_W = (num_cliques == 1)
                    ? SapConstraintJacobian<T>(clique0, J32)
                    : SapConstraintJacobian<T>(clique0, J32, clique1, J34);
  return typename SapBallConstraint<T>::Kinematics{
      objectA, p_WP, p_AP_W, objectB, p_WQ, p_BQ_W, J_PQ_W};
}

void ExpectEqual(const SapBallConstraint<double>& c1,
                 const SapBallConstraint<double>& c2) {
  ExpectBaseIsEqual(c1, c2);
  EXPECT_EQ(c1.kinematics(), c2.kinematics());
}

GTEST_TEST(SapBallConstraint, SingleCliqueConstraint) {
  const int num_cliques = 1;
  const SapBallConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapBallConstraint<double> c(kinematics);

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.num_constraint_equations(), 3);
  EXPECT_EQ(c.num_cliques(), 1);
  EXPECT_EQ(c.first_clique(), kinematics.jacobian().clique(0));
  EXPECT_THROW(c.second_clique(), std::exception);
  EXPECT_EQ(c.first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_THROW(c.second_clique_jacobian(), std::exception);
}

GTEST_TEST(SapBallConstraint, TwoCliquesConstraint) {
  const int num_cliques = 2;
  const SapBallConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapBallConstraint<double> c(kinematics);

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.num_constraint_equations(), 3);
  EXPECT_EQ(c.num_cliques(), 2);
  EXPECT_EQ(c.first_clique(), kinematics.jacobian().clique(0));
  EXPECT_EQ(c.second_clique(), kinematics.jacobian().clique(1));
  EXPECT_EQ(c.first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_EQ(c.second_clique_jacobian().MakeDenseMatrix(), J34);
}

// This method validates analytical gradients implemented by
// SapBallConstraint using automatic differentiation.
void ValidateProjection(const Vector3d& vc) {
  // Arbitrary kinematic values.
  const int num_cliques = 1;
  const SapBallConstraint<AutoDiffXd>::Kinematics kin_ad =
      MakeArbitraryKinematics<AutoDiffXd>(num_cliques);

  // Instantiate constraint on AutoDiffXd for automatic differentiation.
  SapBallConstraint<AutoDiffXd> c(kin_ad);

  // Verify cost gradients using AutoDiffXd.
  ValidateConstraintGradients(c, vc);
}

GTEST_TEST(SapBallConstraint, Gradients) {
  // Arbitrary set of vc values.
  {
    const Vector3d vc(-1.4, 9.3, -4.5);
    ValidateProjection(vc);
  }
  {
    const Vector3d vc(2.3, -1.7, 3.4);
    ValidateProjection(vc);
  }
  {
    const Vector3d vc(6.2, 0.5, -4.9);
    ValidateProjection(vc);
  }
}

GTEST_TEST(SapBallConstraint, SingleCliqueConstraintClone) {
  const int num_cliques = 1;
  const SapBallConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapBallConstraint<double> c(kinematics);

  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone = dynamic_pointer_cast<SapBallConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(c, *clone);

  // Test ToDouble.
  SapBallConstraint<AutoDiffXd> c_ad(
      MakeArbitraryKinematics<AutoDiffXd>(num_cliques));
  auto clone_from_ad =
      dynamic_pointer_cast<SapBallConstraint<double>>(c_ad.ToDouble());
  ASSERT_NE(clone_from_ad, nullptr);
  ExpectEqual(c, *clone_from_ad);
}

GTEST_TEST(SapBallConstraint, TwoCliquesConstraintClone) {
  const int num_cliques = 2;
  const SapBallConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapBallConstraint<double> c(kinematics);

  auto clone = dynamic_pointer_cast<SapBallConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(c, *clone);

  // Test ToDouble.
  SapBallConstraint<AutoDiffXd> c_ad(
      MakeArbitraryKinematics<AutoDiffXd>(num_cliques));
  auto clone_from_ad =
      dynamic_pointer_cast<SapBallConstraint<double>>(c_ad.ToDouble());
  ASSERT_NE(clone_from_ad, nullptr);
  ExpectEqual(c, *clone_from_ad);
}

GTEST_TEST(SapBallConstraint, AccumulateSpatialImpulses) {
  // Make a ball constraint with an arbitrary kinematics state, irrelevant for
  // this test but needed at construction.
  const int num_cliques = 1;
  const SapBallConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapBallConstraint<double> c(kinematics);

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.object(0), kinematics.objectA());
  EXPECT_EQ(c.object(1), kinematics.objectB());

  // Arbitrary value of the impulse.
  const Vector3d gamma(1.2, 3.4, 5.6);

  // Expected spatial impulse on B.
  const SpatialForce<double> F_Bo_W =
      (SpatialForce<double>(Vector3d::Zero(), gamma))
          .Shift(-kinematics.p_BQ_W());

  // Expected spatial impulse on A.
  const SpatialForce<double> F_Ao_W =
      (SpatialForce<double>(Vector3d::Zero(), -gamma))
          .Shift(-kinematics.p_AP_W());

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
