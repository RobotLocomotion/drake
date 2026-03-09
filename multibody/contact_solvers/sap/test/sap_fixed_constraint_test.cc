#include "drake/multibody/contact_solvers/sap/sap_fixed_constraint.h"

#include <limits>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/sap/validate_constraint_gradients.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

// These Jacobian matrices have arbitrary values for testing. We specify the
// size of the matrix in the name, e.g. J62 is of size 3x2.
// clang-format off
const MatrixXd J62 =
    (MatrixXd(6, 2) << 2, 1,
                       1, 2,
                       3, 4,
                       2, 4,
                       3, 6,
                       1, 2).finished();

const MatrixXd J66 =
    (MatrixXd(6, 6) << 7, 1, 2, 3, 1, 2,
                       1, 8, 4, 5, 8, 4,
                       1, 4, 5, 5, 0, 4,
                       2, 2, 4, 5, 2, 1,
                       3, 9, 7, 6, 8, 9,
                       2, 4, 9, 6, 4, 9).finished();
// clang-format on

template <typename T = double>
FixedConstraintKinematics<T> MakeArbitraryKinematics(int num_cliques) {
  const int objectA = 12;
  const VectorX<T> p_APs_W =
      (VectorXd(6) << 4.0, 5.0, 6.0, 4.1, 5.1, 6.1).finished();
  const int objectB = 5;
  const VectorX<T> p_BQs_W =
      (VectorXd(6) << 10.0, 11.0, 12.0, 12.0, 13.0, 15.0).finished();
  const VectorX<T> p_PQs_W =
      (VectorXd(6) << 1.0, 2.0, 3.0, 1.1, 2.2, 3.3).finished();
  const int clique0 = 3;
  const int clique1 = 12;
  auto J_PQ_W = (num_cliques == 1)
                    ? SapConstraintJacobian<T>(clique0, J66)
                    : SapConstraintJacobian<T>(clique0, J66, clique1, J62);
  return (num_cliques == 1)
             ? FixedConstraintKinematics<T>{objectA, p_APs_W, p_PQs_W, J_PQ_W}
             : FixedConstraintKinematics<T>{objectA, p_APs_W, objectB,
                                            p_BQs_W, p_PQs_W, J_PQ_W};
}

GTEST_TEST(SapFixedConstraint, SingleCliqueConstraint) {
  const int num_cliques = 1;
  const FixedConstraintKinematics<double> kinematics =
      MakeArbitraryKinematics(num_cliques);
  const SapFixedConstraint<double> c(kinematics);

  EXPECT_EQ(c.num_objects(), 1);
  EXPECT_EQ(c.num_constraint_equations(), 6);
  EXPECT_EQ(c.num_constrained_point_pairs(), 2);
  EXPECT_EQ(c.num_cliques(), 1);
  EXPECT_EQ(c.first_clique(), kinematics.J.clique(0));
  EXPECT_THROW(c.second_clique(), std::exception);
  EXPECT_EQ(c.first_clique_jacobian().MakeDenseMatrix(), J66);
  EXPECT_THROW(c.second_clique_jacobian(), std::exception);
}

GTEST_TEST(SapFixedConstraint, TwoCliquesConstraint) {
  const int num_cliques = 2;
  const FixedConstraintKinematics<double> kinematics =
      MakeArbitraryKinematics(num_cliques);
  const SapFixedConstraint<double> c(kinematics);

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.num_constraint_equations(), 6);
  EXPECT_EQ(c.num_constrained_point_pairs(), 2);
  EXPECT_EQ(c.num_cliques(), 2);
  EXPECT_EQ(c.first_clique(), kinematics.J.clique(0));
  EXPECT_EQ(c.second_clique(), kinematics.J.clique(1));
  EXPECT_EQ(c.first_clique_jacobian().MakeDenseMatrix(), J66);
  EXPECT_EQ(c.second_clique_jacobian().MakeDenseMatrix(), J62);
}

// This method validates analytical gradients implemented by
// SapFixedConstraint using automatic differentiation.
void ValidateProjection(const VectorXd& vc) {
  // Arbitrary kinematic values.
  const int num_cliques = 1;
  FixedConstraintKinematics<AutoDiffXd> kin_ad =
      MakeArbitraryKinematics<AutoDiffXd>(num_cliques);

  // Instantiate constraint on AutoDiffXd for automatic differentiation.
  SapFixedConstraint<AutoDiffXd> c(std::move(kin_ad));

  // Verify cost gradients using AutoDiffXd.
  ValidateConstraintGradients(c, vc);
}

GTEST_TEST(SapFixedConstraint, Gradients) {
  // Arbitrary set of vc values.
  {
    const VectorXd vc =
        (VectorXd(6) << 10.0, 11.0, 12.0, 12.0, 13.0, 15.0).finished();
    ValidateProjection(vc);
  }
  {
    const VectorXd vc =
        (VectorXd(6) << 2.3, -1.7, 3.4, 9.3, -4.5, 2.3).finished();
    ValidateProjection(vc);
  }
  {
    const VectorXd vc =
        (VectorXd(6) << 6.2, 0.5, -4.9, 0.0, 0.0, 0.0).finished();
    ValidateProjection(vc);
  }
}

GTEST_TEST(SapFixedConstraint, SingleCliqueConstraintClone) {
  const int num_cliques = 1;
  const FixedConstraintKinematics<double> kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapFixedConstraint<double> c(kinematics);

  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone = dynamic_pointer_cast<SapFixedConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->num_objects(), 1);
  EXPECT_EQ(clone->num_constraint_equations(), 6);
  EXPECT_EQ(c.num_constrained_point_pairs(), 2);
  EXPECT_EQ(clone->num_cliques(), 1);
  EXPECT_EQ(clone->first_clique(), kinematics.J.clique(0));
  EXPECT_THROW(clone->second_clique(), std::exception);
  EXPECT_EQ(clone->first_clique_jacobian().MakeDenseMatrix(), J66);
  EXPECT_THROW(clone->second_clique_jacobian(), std::exception);

  DRAKE_EXPECT_THROWS_MESSAGE(
      c.ToDouble(),
      "SapFixedConstraint: Scalar conversion to double not supported.");
}

GTEST_TEST(SapFixedConstraint, TwoCliquesConstraintClone) {
  const int num_cliques = 2;
  const FixedConstraintKinematics<double> kinematics =
      MakeArbitraryKinematics(num_cliques);
  const SapFixedConstraint<double> c(kinematics);

  auto clone = dynamic_pointer_cast<SapFixedConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->num_objects(), 2);
  EXPECT_EQ(clone->num_constraint_equations(), 6);
  EXPECT_EQ(c.num_constrained_point_pairs(), 2);
  EXPECT_EQ(clone->num_cliques(), 2);
  EXPECT_EQ(clone->first_clique(), kinematics.J.clique(0));
  EXPECT_EQ(clone->second_clique(), kinematics.J.clique(1));
  EXPECT_EQ(clone->first_clique_jacobian().MakeDenseMatrix(), J66);
  EXPECT_EQ(clone->second_clique_jacobian().MakeDenseMatrix(), J62);

  DRAKE_EXPECT_THROWS_MESSAGE(
      c.ToDouble(),
      "SapFixedConstraint: Scalar conversion to double not supported.");
}

GTEST_TEST(SapFixedConstraint, AccumulateGeneralizedImpulses) {
  const int num_cliques = 2;
  const FixedConstraintKinematics<double> kinematics =
      MakeArbitraryKinematics(num_cliques);
  const SapFixedConstraint<double> c(kinematics);

  // Arbitrary impulse vector.
  const VectorXd gamma =
      (VectorXd(6) << -0.25, 0.01, 0.03, -0.5, 0.04, -0.5).finished();

  // Arbitrary nonzero taus to be accumulated into.
  VectorXd tau0 = (VectorXd(6) << -0.23, 0.21, 1, 2, 3, 4).finished();
  VectorXd tau1 = (VectorXd(2) << -0.23, 0.21).finished();

  // We expect AccumulateGeneralizedImpulses to be an no-op -- nothing should be
  // accumulated.
  const VectorXd tau0_expected = tau0;
  const VectorXd tau1_expected = tau1;

  c.AccumulateGeneralizedImpulses(0, gamma, &tau0);
  c.AccumulateGeneralizedImpulses(1, gamma, &tau1);

  EXPECT_EQ(tau0, tau0_expected);
  EXPECT_EQ(tau1, tau1_expected);
}

/* In this test, we set a special kinematics to verify the applied spatial
   impulse with a hand calculated reference impulse.

                  +z
                   |
                   |            P1 (0.5, 0, 0.5)
                   |
                   |
                   |                             (1, 0, 0)
    -x-----------Ao+----P0------------Q0--------Bo---------+x
           (0,0,0) | (0.25, 0, 0)   (0.75, 0, 0)
                   |
                   |
                   |            Q1 (0.5, 0, -0.5)
                   |
                  -z
*/
GTEST_TEST(SapFixedConstraint, AccumulateSpatialImpulses) {
  const int objectA = 0;
  const VectorXd p_APs_W =
      (VectorXd(6) << 0.25, 0.0, 0.0, 0.5, 0.0, 0.5).finished();
  const int objectB = 1;
  const VectorXd p_BQs_W =
      (VectorXd(6) << -0.25, 0.0, 0.0, -0.5, 0.0, -0.5).finished();
  const VectorXd p_PQs_W =
      (VectorXd(6) << 0.5, 0.0, 0.0, 0.0, 0.0, -1.0).finished();
  const int clique0 = 0;
  const int clique1 = 1;
  // Compatibly-sized Jacobian with arbitrary values as it doesn't affect the
  // test result.
  SapConstraintJacobian<double> J_PQ_W(clique0, J66, clique1, J62);
  FixedConstraintKinematics<double> kinematics{objectA, p_APs_W, objectB,
                                               p_BQs_W, p_PQs_W, J_PQ_W};
  SapFixedConstraint<double> c(kinematics);

  // Impulse pulling P0 and Q0 together.
  const Vector3d gamma0(-1, 0, 0);
  // Impulse pulling P1 and Q1 together.
  const Vector3d gamma1(0, 0, 1);
  const VectorXd gamma = (VectorXd(6) << gamma0, gamma1).finished();

  // Expected spatial impulse on B.
  const SpatialForce<double> F_Bo_W =
      SpatialForce<double>(Vector3d::Zero(), gamma0)
          .Shift(Vector3d(0.25, 0, 0)) +
      SpatialForce<double>(Vector3d::Zero(), gamma1)
          .Shift(Vector3d(0.5, 0, 0.5));

  // Expected spatial impulse on A.
  const SpatialForce<double> F_Ao_W =
      SpatialForce<double>(Vector3d::Zero(), -gamma0)
          .Shift(Vector3d(-0.25, 0, 0)) +
      SpatialForce<double>(Vector3d::Zero(), -gamma1)
          .Shift(Vector3d(-0.5, 0, -0.5));

  const SpatialForce<double> F0(Vector3d(1, 2, 3), Vector3d(4, 5, 6));
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
