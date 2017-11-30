#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/drake_kuka_iiwa_robot.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {
namespace {

using Eigen::Vector3d;
using Eigen::Matrix3d;
using drake::multibody::multibody_tree::test_utilities::SpatialKinematicsPVA;

// This test is used to verify the correctness of the method
// MultibodyTree::CalcPointsGeometricJacobianExpressedInWorld().
// The test computes the end effector Jacobian J_NG (in the Newtonian, world,
// frame N) using two methods:
// 1. Calling MultibodyTree::CalcPointsGeometricJacobianExpressedInWorld().
// 2. Using AutoDiffXd to compute the partial derivative of v_NG(q, v) with
//    respect to v.
// By comparing the two results we verify the correctness of the MultibodyTree
// implementation.
GTEST_TEST(KukaIiwaRobot, GeometricJacobian) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = 7;

  // Acceleration of gravity is not an important parameter for this purely
  // kinematic test. Therefore we arbitrarily set it to zero.
  const double kGravity = 0.0;

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  // A set of values for the joint's angles chosen mainly to avoid in-plane
  // motions.
  const double q30 = M_PI / 6, q60 = M_PI / 3;
  const double qA = q60;
  const double qB = q30;
  const double qC = q60;
  const double qD = q30;
  const double qE = q60;
  const double qF = q30;
  const double qG = q60;
  VectorX<double> q(kNumPositions);
  q << qA, qB, qC, qD, qE, qF, qG;

  // A non-zero set of values for the joint's velocities.
  const double vA = 0.1;
  const double vB = 0.2;
  const double vC = 0.3;
  const double vD = 0.4;
  const double vE = 0.5;
  const double vF = 0.6;
  const double vG = 0.7;
  VectorX<double> v(kNumPositions);
  v << vA, vB, vC, vD, vE, vF, vG;

  // This helper lambda computes v_NG(q, v), the translational velocity
  // of the end effector frame G in the world (Newtonian) frame N.
  auto end_effector_velocity = [kGravity, kNumPositions](
      const auto& q_on_T, const auto& v_on_T) {
    using T = typename std::remove_reference<decltype(q_on_T)>::type::Scalar;
    DrakeKukaIIwaRobot<T> kuka_robot(kGravity);
    // v_NG does not depend on vdot, it is a function of q and v only.
    // Therefore we arbitrarily set vdot = 0.
    const VectorX<T> vdot = VectorX<T>::Zero(kNumPositions);
    const SpatialKinematicsPVA<T> drake_kinematics =
        kuka_robot.CalcEndEffectorKinematics(q_on_T, v_on_T, vdot);
    return drake_kinematics.translational_velocity();
  };

  // Helper lambda to keep positions fixed in q so that we can take
  // partial derivatives with respect to v.
  auto end_effector_velocity_fixed_positions = [&](const auto& v_on_T) {
    using T = typename std::remove_reference<decltype(v_on_T)>::type::Scalar;
    Eigen::Matrix<T, kNumPositions, 1> q_on_T = q;
    return end_effector_velocity(q_on_T, v_on_T);
  };

  // Compute the value of the end effector's velocity using <double> values
  // of v.
  const Vector3<double> v_NG = end_effector_velocity_fixed_positions(v);

  // Initialize v_autodiff to have values v and so that it is the independent
  // variable of the problem.
  VectorX<AutoDiffXd> v_autodiff(kNumPositions);
  math::initializeAutoDiff(v, v_autodiff);

  // Compute now both, value and partial derivatives with respect to v, using
  // <AutoDiffXd> values of v on the same method call as above.
  const Vector3<AutoDiffXd> v_NG_autodiff =
      end_effector_velocity_fixed_positions(v_autodiff);
  const Vector3<double> v_NG_value = math::autoDiffToValueMatrix(v_NG_autodiff);
  const MatrixX<double> v_NG_derivs =
      math::autoDiffToGradientMatrix(v_NG_autodiff);

  // Values obtained with <AutoDiffXd> should match those computed with
  // <double>.
  EXPECT_TRUE(CompareMatrices(v_NG_value, v_NG,
                              kTolerance, MatrixCompareType::relative));

  // Some sanity checks on the expected sizes of the derivatives.
  EXPECT_EQ(v_NG_derivs.rows(), 3);
  EXPECT_EQ(v_NG_derivs.cols(), kNumPositions);

  DrakeKukaIIwaRobot<double> kuka_robot(kGravity);
  Vector3<double> p_NG;
  Matrix3X<double> J_NG(3, kNumPositions);
  // The end effector (G) Jacobian is computed by asking the Jacobian for a
  // point P with position p_GP = 0 in the G frame.
  kuka_robot.CalcPointsOnEndEffectorGeometricJacobian(
      q, Vector3<double>::Zero(), &p_NG, &J_NG);
  const SpatialKinematicsPVA<double> spatial_kinematics =
      kuka_robot.CalcEndEffectorKinematics(
          q, v, VectorX<double>(kNumPositions));
  const Vector3<double> p_NG_expected = spatial_kinematics.position_vector();

  // Verify the value of the end effector position.
  EXPECT_TRUE(CompareMatrices(p_NG, p_NG_expected,
                              kTolerance, MatrixCompareType::relative));

  // Verify the computed Jacobian matches the one obtained using automatic
  // differentiation.
  EXPECT_TRUE(CompareMatrices(J_NG, v_NG_derivs,
                              kTolerance, MatrixCompareType::relative));

  // Verify that v_NG = J_NG * v:
  const Vector3<double> J_NG_times_v = J_NG * v;
  EXPECT_TRUE(CompareMatrices(J_NG_times_v, v_NG,
                              kTolerance, MatrixCompareType::relative));
}

// Given a set of points Pi attached to the end effector frame G, this test
// computes the analytic Jacobian J_NGpi of these points using two methods:
// 1. Since for the Kuka iiwa arm v = q̇, the analytic Jacobian equals the
//    geometric Jacobian and we compute it with MultibodyTree's implementation.
// 2. We compute the analytic Jacobian by direct differentiation with respect to
//    q using AutoDiffXd.
// We then verify MultibodyTree's implementation by comparing the results from
// both methods.
GTEST_TEST(KukaIiwaRobot, AnalyticJacobian) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = 7;

  // Acceleration of gravity is not an important parameter for this purely
  // kinematic test. Therefore we arbitrarily set it to zero.
  const double kGravity = 0.0;

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  // A set of values for the joint's angles chosen mainly to avoid in-plane
  // motions.
  const double q30 = M_PI / 6, q60 = M_PI / 3;
  const double qA = q60;
  const double qB = q30;
  const double qC = q60;
  const double qD = q30;
  const double qE = q60;
  const double qF = q30;
  const double qG = q60;
  VectorX<double> q0(kNumPositions);
  q0 << qA, qB, qC, qD, qE, qF, qG;

  // A set of points Pi attached to the end effector, thus we a fixed position
  // in its frame G.
  const int kNumPoints = 2;  // The set stores 2 points.
  Matrix3X<double> p_GPi(3, kNumPoints);
  p_GPi.col(0) << 0.1, -0.05, 0.02;
  p_GPi.col(1) << 0.2, 0.3, -0.15;

  DrakeKukaIIwaRobot<double> kuka_robot(kGravity);
  Matrix3X<double> p_NGpi(3, kNumPoints);
  MatrixX<double> J_NGpi(3 * kNumPoints, kNumPositions);

  // Since for the Kuka iiwa arm v = q̇, the analytic Jacobian equals the
  // geometric Jacobian.
  kuka_robot.CalcPointsOnEndEffectorGeometricJacobian(
      q0, p_GPi, &p_NGpi, &J_NGpi);

  // Alternatively, compute the analytic Jacobian by taking the gradient of
  // the positions p_NGpi(q) with respect to the generalized positions.
  DrakeKukaIIwaRobot<AutoDiffXd> kuka_robot_autodiff(kGravity);

  // Initialize q to have values qvalue and so that it is the independent
  // variable of the problem.
  VectorX<AutoDiffXd> q_autodiff(kNumPositions);
  math::initializeAutoDiff(q0, q_autodiff);

  const Matrix3X<AutoDiffXd> p_GPi_autodiff = p_GPi;
  Matrix3X<AutoDiffXd> p_NGpi_autodiff(3, kNumPoints);
  MatrixX<AutoDiffXd> J_NGpi_autodiff(3 * kNumPoints, kNumPositions);
  kuka_robot_autodiff.CalcPointsOnEndEffectorGeometricJacobian(
      q_autodiff, p_GPi_autodiff, &p_NGpi_autodiff, &J_NGpi_autodiff);
  // Extract values and derivatives:
  const Matrix3X<double> p_NGpi_value =
      math::autoDiffToValueMatrix(p_NGpi_autodiff);
  const MatrixX<double> p_NGpi_derivs =
      math::autoDiffToGradientMatrix(p_NGpi_autodiff);

  // Some sanity checks:
  // Values obtained with <AutoDiffXd> should match those computed with
  // <double>.
  EXPECT_TRUE(CompareMatrices(p_NGpi_value, p_NGpi,
                              kTolerance, MatrixCompareType::relative));
  // Sizes of the derivatives.
  EXPECT_EQ(p_NGpi_derivs.rows(), 3 * kNumPoints);
  EXPECT_EQ(p_NGpi_derivs.cols(), kNumPositions);

  // Verify the computed Jacobian J_NGpi matches the one obtained using
  // automatic differentiation.
  // In this case analytic and geometric Jacobians are equal since v = q.
  EXPECT_TRUE(CompareMatrices(J_NGpi, p_NGpi_derivs,
                              kTolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
