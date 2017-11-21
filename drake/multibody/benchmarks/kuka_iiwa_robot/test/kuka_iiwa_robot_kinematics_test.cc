#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/jacobian.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/MG/MG_kuka_iiwa_robot.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/drake_kuka_iiwa_robot.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {
namespace {

using Eigen::Vector3d;
using Eigen::Matrix3d;
using drake::multibody::multibody_tree::test_utilities::SpatialKinematicsPVA;

// Compare Drake's MultibodyTree kinematics with MotionGenesis solution.
void TestEndEffectorKinematics(const Eigen::Ref<const VectorX<double>>& q,
                               const Eigen::Ref<const VectorX<double>>& qDt,
                               const Eigen::Ref<const VectorX<double>>& qDDt) {
  // Get Drake's end-effector kinematics information, including:
  // R_NG       | Rotation matrix relating Nx, Ny, Nz to Gx, Gy, Gz.
  // p_NoGo_N   | Go's position from No, expressed in N.
  // w_NG_N     | G's angular velocity in N, expressed in N.
  // v_NGo_N    | Go's velocity in N, expressed in N.
  // alpha_NG_N | G's angular acceleration in N, expressed in N.
  // a_NGo_N    | Go's acceleration in N, expressed in N.
  DrakeKukaIIwaRobot<double> drake_kuka_robot(0);
  const SpatialKinematicsPVA<double> drake_kinematics =
      drake_kuka_robot.CalcEndEffectorKinematics(q, qDt, qDDt);

  // Get corresponding MotionGenesis information.
  MG::MGKukaIIwaRobot<double> MG_kuka_robot(0);
  const SpatialKinematicsPVA<double> MG_kinematics =
      MG_kuka_robot.CalcEndEffectorKinematics(q, qDt, qDDt);

  // Kinematics: Compare Drake results with MotionGenesis (expected) results.
  constexpr double kEpsilon = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(drake_kinematics.rotation_matrix().isApprox(
      MG_kinematics.rotation_matrix(), kEpsilon));
  EXPECT_TRUE(drake_kinematics.position_vector().isApprox(
      MG_kinematics.position_vector(), kEpsilon));
  EXPECT_TRUE(drake_kinematics.angular_velocity().isApprox(
      MG_kinematics.angular_velocity(), kEpsilon));
  EXPECT_TRUE(drake_kinematics.translational_velocity().isApprox(
      MG_kinematics.translational_velocity(), kEpsilon));
  EXPECT_TRUE(drake_kinematics.angular_acceleration().isApprox(
      MG_kinematics.angular_acceleration(), kEpsilon));
  EXPECT_TRUE(drake_kinematics.translational_acceleration().isApprox(
      MG_kinematics.translational_acceleration(), kEpsilon));
}

// Verify methods for MultibodyTree::CalcPositionKinematicsCache(), comparing
// its computed results to a MotionGenesis solution.
GTEST_TEST(KukaIIwaRobotKinematics, KinematicsTestA) {
  // Provide values for joint angles q and their 1st and 2nd time-derivatives.
  // The values below correspond to a static configuration.
  double q30 = M_PI / 6, q60 = M_PI / 3;
  double qA = q60, qAdot = 0.0, qAddot = 0.0;
  double qB = q30, qBdot = 0.0, qBddot = 0.0;
  double qC = q60, qCdot = 0.0, qCddot = 0.0;
  double qD = q30, qDdot = 0.0, qDddot = 0.0;
  double qE = q60, qEdot = 0.0, qEddot = 0.0;
  double qF = q30, qFdot = 0.0, qFddot = 0.0;
  double qG = q60, qGdot = 0.0, qGddot = 0.0;

  // Create a state with joint angles (q) and their time-derivatives (qDt).
  Eigen::Matrix<double, 7, 1> q, qDt, qDDt;
  q << qA, qB, qC, qD, qE, qF, qG;
  qDt << qAdot, qBdot, qCdot, qDdot, qEdot, qFdot, qGdot;
  qDDt << qAddot, qBddot, qCddot, qDddot, qEddot, qFddot, qGddot;

  // Static configuration test.
  TestEndEffectorKinematics(q, qDt, qDDt);

  qAdot = 0.1;
  qBdot = 0.2;
  qCdot = 0.3;
  qDdot = 0.4;
  qEdot = 0.5;
  qFdot = 0.6;
  qGdot = 0.7;
  qDt << qAdot, qBdot, qCdot, qDdot, qEdot, qFdot, qGdot;

  // More general test includes non-zero velocity, centrifugal acceleration.
  TestEndEffectorKinematics(q, qDt, qDDt);

  qAddot = 0.7;
  qBddot = 0.6;
  qCddot = 0.5;
  qDddot = 0.4;
  qEddot = 0.3;
  qFddot = 0.2;
  qGddot = 0.1;
  qDDt << qAddot, qBddot, qCddot, qDddot, qEddot, qFddot, qGddot;

  // Most general of these three tests non-zero velocity and acceleration.
  TestEndEffectorKinematics(q, qDt, qDDt);
}

#include <iostream>
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

GTEST_TEST(KukaIIwaRobotKinematics, GeometricJacobian) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = 7;

  // Acceleration of gravity is not an important parameter for this purely
  // kinematic test. Therefore we arbitrarily set it to zero.
  const double kGravity = 0.0;

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  // A set of values for the joint's angles chosen mainly to avoid in-plane
  // motions.
  double q30 = M_PI / 6, q60 = M_PI / 3;
  double qA = q60;
  double qB = q30;
  double qC = q60;
  double qD = q30;
  double qE = q60;
  double qF = q30;
  double qG = q60;
  VectorX<double> qvalue(kNumPositions);
  qvalue << qA, qB, qC, qD, qE, qF, qG;

  // A non-zero set of values for the joint's velocities.
  double vA = 0.1;
  double vB = 0.2;
  double vC = 0.3;
  double vD = 0.4;
  double vE = 0.5;
  double vF = 0.6;
  double vG = 0.7;
  VectorX<double> vvalue(kNumPositions);
  vvalue << vA, vB, vC, vD, vE, vF, vG;

  // This helper lambda computes v_NG(q, v), the translational velocity of the
  // end effector frame G in the world (Newtonian) frame N.
  auto end_effector_velocity = [kGravity, kNumPositions](
      const auto& q, const auto& v) {
    using T = typename std::remove_reference<decltype(q)>::type::Scalar;
    DrakeKukaIIwaRobot<T> drake_kuka_robot(kGravity);
    // v_NG does not depend on vdot, it is a function of q and v only.
    // Therefore we arbitrarily set vdot = 0.
    const VectorX<T> vdot = VectorX<T>::Zero(kNumPositions);
    const SpatialKinematicsPVA<T> drake_kinematics =
        drake_kuka_robot.CalcEndEffectorKinematics(q, v, vdot);
    return drake_kinematics.translational_velocity();
  };

  // Helper lambda to keep positions fixed at q = qvalue so that we can take
  // partial derivatives with respect to v.
  auto end_effector_velocity_fixed_positions = [&](const auto& v) {
    using T = typename std::remove_reference<decltype(v)>::type::Scalar;
    Eigen::Matrix<T, kNumPositions, 1> q = qvalue;
    return end_effector_velocity(q, v);
  };

  // Compute the value of the end effector's velocity using <double> values
  // of v.
  Vector3<double> v_NG = end_effector_velocity_fixed_positions(vvalue);

  // Initialize v to have values vvalue and so that it is the independent
  // variable of the problem.
  VectorX<AutoDiffXd> v(kNumPositions);
  math::initializeAutoDiff(vvalue, v);

  // Compute now both, value and partial derivatives with respect to v, using
  // <AutoDiffXd> values of v on the same method call as above.
  Vector3<AutoDiffXd> v_NG_autodiff = end_effector_velocity_fixed_positions(v);
  Vector3<double> v_NG_value = math::autoDiffToValueMatrix(v_NG_autodiff);
  MatrixX<double> v_NG_derivs = math::autoDiffToGradientMatrix(v_NG_autodiff);

  // Values obtained with <AutoDiffXd> should match those computed with
  // <double>.
  EXPECT_TRUE(CompareMatrices(v_NG_value, v_NG,
                              kTolerance, MatrixCompareType::relative));

  // Some sanity checks on the expected sizes of the derivatives.
  EXPECT_EQ(v_NG_derivs.rows(), 3);
  EXPECT_EQ(v_NG_derivs.cols(), kNumPositions);

  DrakeKukaIIwaRobot<double> drake_kuka_robot(kGravity);
  Vector3<double> p_NG;
  Matrix3X<double> J_NG(3, kNumPositions);
  drake_kuka_robot.CalcEndEffectorGeometricJacobian(qvalue, &p_NG, &J_NG);
  const SpatialKinematicsPVA<double> spatial_kinematics =
      drake_kuka_robot.CalcEndEffectorKinematics(
          qvalue, vvalue, VectorX<double>(kNumPositions));
  const Vector3<double> p_NG_expected = spatial_kinematics.position_vector();

  // Verify the value of the end effector position.
  EXPECT_TRUE(CompareMatrices(p_NG, p_NG_expected,
                              kTolerance, MatrixCompareType::relative));

  // Verify the computed Jacobian matches the one obtained using automatic
  // differentiation.
  EXPECT_TRUE(CompareMatrices(J_NG, v_NG_derivs,
                              kTolerance, MatrixCompareType::relative));

  // Verify that v_NG = J_NG * v:
  const Vector3<double> J_NG_times_v = J_NG * vvalue;
  EXPECT_TRUE(CompareMatrices(J_NG_times_v, v_NG,
                              kTolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
