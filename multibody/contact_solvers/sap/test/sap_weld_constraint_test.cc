#include "drake/multibody/contact_solvers/sap/sap_weld_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rotation_matrix.h"
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
namespace kcov339_avoidance_magic {
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
const double theta = 0.1;

template <typename T = double>
typename SapWeldConstraint<T>::Kinematics MakeArbitraryKinematics(
    int num_cliques) {
  const int objectA = 12;
  RigidTransform<T> X_AFp(RotationMatrix<T>::MakeZRotation(theta),
                          Vector3d(1., 2., 3.));
  RigidTransform<T> X_WA(RotationMatrix<T>::MakeXRotation(theta),
                         Vector3d(4., 5., 6.));
  const int objectB = 5;
  RigidTransform<T> X_BFq(RotationMatrix<T>::MakeZRotation(-theta),
                          Vector3d(7., 8., 9.));
  RigidTransform<T> X_WB(RotationMatrix<T>::MakeXRotation(-theta),
                         Vector3d(10., 11., 12.));
  const int clique0 = 3;
  const int clique1 = 12;
  auto Jv_FpFq_W = (num_cliques == 1) ? SapConstraintJacobian<T>(clique0, J62_W)
                                      : SapConstraintJacobian<T>(
                                            clique0, J62_W, clique1, J64_W);
  return typename SapWeldConstraint<T>::Kinematics{
      objectA, X_AFp, X_WA, objectB, X_BFq, X_WB, Jv_FpFq_W};
}

GTEST_TEST(SapWeldConstraint, Kinematics) {}

GTEST_TEST(SapWeldConstraint, SingleCliqueConstraint) {
  const int num_cliques = 1;
  const double relaxation_time = 0.01;
  const SapWeldConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapWeldConstraint<double> c(kinematics, relaxation_time);

  const MatrixX<double>& R_NW =
      c.kinematics().X_WN().rotation().matrix().transpose();
  MatrixX<double> J62_N(J62_W.rows(), J62_W.cols());
  J62_N << R_NW * J62_W.template topRows<3>(),
      R_NW * J62_W.template bottomRows<3>();

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.num_constraint_equations(), 6);
  EXPECT_EQ(c.num_cliques(), 1);
  EXPECT_EQ(c.first_clique(), c.kinematics().jacobian().clique(0));
  EXPECT_THROW(c.second_clique(), std::exception);
  EXPECT_TRUE(CompareMatrices(c.first_clique_jacobian().MakeDenseMatrix(),
                              J62_N, kEps, MatrixCompareType::relative));
  EXPECT_THROW(c.second_clique_jacobian(), std::exception);
}

GTEST_TEST(SapWeldConstraint, TwoCliquesConstraint) {
  const int num_cliques = 2;
  const double relaxation_time = 0.01;
  const SapWeldConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapWeldConstraint<double> c(kinematics, relaxation_time);

  const MatrixX<double>& R_NW =
      c.kinematics().X_WN().rotation().matrix().transpose();
  MatrixX<double> J62_N(J62_W.rows(), J62_W.cols());
  J62_N << R_NW * J62_W.template topRows<3>(),
      R_NW * J62_W.template bottomRows<3>();
  MatrixX<double> J64_N(J64_W.rows(), J64_W.cols());
  J64_N << R_NW * J64_W.template topRows<3>(),
      R_NW * J64_W.template bottomRows<3>();

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.num_constraint_equations(), 6);
  EXPECT_EQ(c.num_cliques(), 2);
  EXPECT_EQ(c.first_clique(), c.kinematics().jacobian().clique(0));
  EXPECT_EQ(c.second_clique(), c.kinematics().jacobian().clique(1));

  // c's Jacobian is expressed in intermediate frame N, but we expect R_NW = I
  // by construction, so J(0) == J62_W and J(1) == J64_W.
  EXPECT_TRUE(CompareMatrices(c.first_clique_jacobian().MakeDenseMatrix(),
                              J62_N, kEps, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(c.second_clique_jacobian().MakeDenseMatrix(),
                              J64_N, kEps, MatrixCompareType::relative));
}

// This method validates analytical gradients implemented by
// SapWeldConstraint using automatic differentiation.
void ValidateProjection(const Vector6d& vc) {
  const double relaxation_time = 0.01;

  // Arbitrary kinematic values.
  const int num_cliques = 1;
  const SapWeldConstraint<AutoDiffXd>::Kinematics kin_ad =
      MakeArbitraryKinematics<AutoDiffXd>(num_cliques);

  // Instantiate constraint on AutoDiffXd for automatic differentiation.
  SapWeldConstraint<AutoDiffXd> c(kin_ad, relaxation_time);

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
  const double relaxation_time = 0.01;
  const SapWeldConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapWeldConstraint<double> c(kinematics, relaxation_time);

  const MatrixX<double>& R_NW =
      c.kinematics().X_WN().rotation().matrix().transpose();
  MatrixX<double> J62_N(J62_W.rows(), J62_W.cols());
  J62_N << R_NW * J62_W.template topRows<3>(),
      R_NW * J62_W.template bottomRows<3>();

  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone = dynamic_pointer_cast<SapWeldConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->num_objects(), 2);
  EXPECT_EQ(clone->num_constraint_equations(), 6);
  EXPECT_EQ(clone->num_cliques(), 1);
  EXPECT_EQ(clone->first_clique(), c.kinematics().jacobian().clique(0));
  EXPECT_THROW(clone->second_clique(), std::exception);
  // c's Jacobian is expressed in intermediate frame N, but we expect R_NW = I
  // by construction, so J(0) == J62_W.
  EXPECT_TRUE(CompareMatrices(clone->first_clique_jacobian().MakeDenseMatrix(),
                              J62_N, kEps, MatrixCompareType::relative));
  EXPECT_THROW(clone->second_clique_jacobian(), std::exception);
}

GTEST_TEST(SapWeldConstraint, TwoCliquesConstraintClone) {
  const int num_cliques = 2;
  const double relaxation_time = 0.01;
  const SapWeldConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapWeldConstraint<double> c(kinematics, relaxation_time);

  const MatrixX<double>& R_NW =
      c.kinematics().X_WN().rotation().matrix().transpose();
  MatrixX<double> J62_N(J62_W.rows(), J62_W.cols());
  J62_N << R_NW * J62_W.template topRows<3>(),
      R_NW * J62_W.template bottomRows<3>();
  MatrixX<double> J64_N(J64_W.rows(), J64_W.cols());
  J64_N << R_NW * J64_W.template topRows<3>(),
      R_NW * J64_W.template bottomRows<3>();

  auto clone = dynamic_pointer_cast<SapWeldConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->num_objects(), 2);
  EXPECT_EQ(clone->num_constraint_equations(), 6);
  EXPECT_EQ(clone->num_cliques(), 2);
  EXPECT_EQ(clone->first_clique(), c.kinematics().jacobian().clique(0));
  EXPECT_EQ(clone->second_clique(), c.kinematics().jacobian().clique(1));
  // c's Jacobian is expressed in intermediate frame N, but we expect R_NW = I
  // by construction, so J(0) == J62_W and J(1) == J64_W.
  EXPECT_TRUE(CompareMatrices(clone->first_clique_jacobian().MakeDenseMatrix(),
                              J62_N, kEps, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(clone->second_clique_jacobian().MakeDenseMatrix(),
                              J64_N, kEps, MatrixCompareType::relative));
}

GTEST_TEST(SapWeldConstraint, AccumulateSpatialImpulses) {
  // Make a weld constraint with an arbitrary kinematics state.
  const int num_cliques = 1;
  const double relaxation_time = 0.01;
  const SapWeldConstraint<double>::Kinematics kinematics =
      MakeArbitraryKinematics(num_cliques);
  SapWeldConstraint<double> c(kinematics, relaxation_time);

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.object(0), c.kinematics().objectA());
  EXPECT_EQ(c.object(1), c.kinematics().objectB());

  // Arbitrary value of the impulse.
  const Vector6d gamma =
      (Vector6d() << -1.2, 3.4, 5.6, -7.8, 9.1, -2.3).finished();

  const SpatialForce<double> F_No_N(gamma.template segment<3>(0),
                                    gamma.template segment<3>(3));

  // Expected spatial impulse on B.
  const SpatialForce<double> F_Bo_N =
      F_No_N.Shift(c.kinematics().X_NB().translation());
  SpatialForce<double> F_Bo_W(
      c.kinematics().X_WN().rotation() * F_Bo_N.rotational(),
      c.kinematics().X_WN().rotation() * F_Bo_N.translational());

  // Expected spatial impulse on A.
  const SpatialForce<double> F_Ao_N =
      -F_No_N.Shift(c.kinematics().X_NA().translation());
  SpatialForce<double> F_Ao_W(
      c.kinematics().X_WN().rotation() * F_Ao_N.rotational(),
      c.kinematics().X_WN().rotation() * F_Ao_N.translational());

  const SpatialForce<double> F0_Bo_W(Vector3d(1., 2., 3), Vector3d(4., 5., 6));
  SpatialForce<double> Faccumulated = F0_Bo_W;  // Initialize to non-zero value.
  const SpatialForce<double> F_Bo_W_expected = F0_Bo_W + F_Bo_W;
  c.AccumulateSpatialImpulses(1, gamma, &Faccumulated);
  EXPECT_TRUE(CompareMatrices(
      Faccumulated.get_coeffs(), F_Bo_W_expected.get_coeffs(),
      std::numeric_limits<double>::epsilon(), MatrixCompareType::relative));

  const SpatialForce<double> F0_Ao_W(Vector3d(1., 2., 3), Vector3d(4., 5., 6));
  Faccumulated = F0_Ao_W;  // Initialize to non-zero value.
  const SpatialForce<double> F_Ao_W_expected = F0_Ao_W + F_Ao_W;
  c.AccumulateSpatialImpulses(0, gamma, &Faccumulated);
  EXPECT_TRUE(CompareMatrices(
      Faccumulated.get_coeffs(), F_Ao_W_expected.get_coeffs(),
      std::numeric_limits<double>::epsilon(), MatrixCompareType::relative));
}

}  // namespace
}  // namespace kcov339_avoidance_magic
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
