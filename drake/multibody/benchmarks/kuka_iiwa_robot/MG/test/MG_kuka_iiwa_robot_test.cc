#include "drake/multibody/benchmarks/kuka_iiwa_robot/MG/MG_kuka_iiwa_robot.h"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {
namespace {

// Function to compare Kuka iiwa robot arm end-effector (frame G)'s orientation
// position, angular velocity, velocity in World (frame N) to expected solution.
// This method's inputs are the state (q, qDt) and expected values for frame G.
// q                  |  robot's joint angles (generalized coordinates).
// qDt                |  1st-time-derivatives of joint angles (a.k.a. qdot).
// qDDt               |  2nd-time-derivatives of joint angles (a.k.a. qddot).
// R_NG_expected      |  Rotation matrix relating Nx, Ny, Nz to Gx, Gy, Gz.
// p_NoGo_N_expected  |  Go's position from No, expressed in N.
// w_NG_N_expected    |  G's angular velocity in N, expressed in N.
// v_NGo_N_expected   |  Go's velocity in N, expressed in N.
void CompareEndEffectorPositionVelocityVsExpectedSolution(
    const Eigen::Matrix<double, 7, 1>& q,
    const Eigen::Matrix<double, 7, 1>& qDt,
    const Eigen::Matrix<double, 7, 1>& qDDt,
    const Eigen::Matrix3d& R_NG_expected,
    const Eigen::Vector3d& p_No_Go_N_expected,
    const Eigen::Vector3d& w_NG_N_expected,
    const Eigen::Vector3d& v_NGo_N_expected,
    const Eigen::Vector3d& alpha_NG_N_expected,
    const Eigen::Vector3d& a_NGo_N_expected) {
  MG::MGKukaIIwaRobot<double> MG_kuka_robot;
  // R_NG       | Rotation matrix relating Nx, Ny, Nz to Gx, Gy, Gz.
  // p_NoGo_N   | Go's position from No, expressed in N.
  // w_NG_N     | G's angular velocity in N, expressed in N.
  // v_NGo_N    | Go's velocity in N, expressed in N.
  // alpha_NG_N | G's angular acceleration in N, expressed in N.
  // a_NGo_N    | Go's acceleration in N, expressed in N.
  Eigen::Matrix3d R_NG;
  Eigen::Vector3d p_No_Go_N, w_NG_N, v_NGo_N, alpha_NG_N, a_NGo_N;
  std::tie(R_NG, p_No_Go_N, w_NG_N, v_NGo_N, alpha_NG_N, a_NGo_N) =
      MG_kuka_robot.CalcEndEffectorKinematics(q, qDt, qDDt);

  // Compare actual results with expected results.
  constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  const double tolerance = 10 * kEpsilon;
  EXPECT_TRUE(R_NG.isApprox(R_NG_expected, tolerance));
  EXPECT_TRUE(p_No_Go_N.isApprox(p_No_Go_N_expected, tolerance));
  EXPECT_TRUE(w_NG_N.isApprox(w_NG_N_expected, tolerance));
  EXPECT_TRUE(v_NGo_N.isApprox(v_NGo_N_expected, tolerance));
  EXPECT_TRUE(alpha_NG_N.isApprox(alpha_NG_N_expected, tolerance));
  EXPECT_TRUE(a_NGo_N.isApprox(a_NGo_N_expected, tolerance));
}


// Test accuracy of calculations for Kuka iiwa robot arm end-effector
// orientation, position, angular velocity, and velocity for the
// situation when the Kuka arm is static and straight up.
GTEST_TEST(KukaIIwaRobot, ForwardKinematicsA) {
  const double qA = 0.0, qADt = 0.0, qADDt = 0.0;
  const double qB = 0.0, qBDt = 0.0, qBDDt = 0.0;
  const double qC = 0.0, qCDt = 0.0, qCDDt = 0.0;
  const double qD = 0.0, qDDt = 0.0, qDDDt = 0.0;
  const double qE = 0.0, qEDt = 0.0, qEDDt = 0.0;
  const double qF = 0.0, qFDt = 0.0, qFDDt = 0.0;
  const double qG = 0.0, qGDt = 0.0, qGDDt = 0.0;

  // Create a state with joint angles (q) and their time-derivatives (qDt).
  Eigen::Matrix<double, 7, 1> q, q_Dt, q_DDt;
  q << qA, qB, qC, qD, qE, qF, qG;
  q_Dt << qADt, qBDt, qCDt, qDDt, qEDt, qFDt, qGDt;
  q_DDt << qADDt, qBDDt, qCDDt, qDDDt, qEDDt, qFDDt, qGDDt;

  // MotionGenesis solution for these joint angles/time-derivatives.
  // This solution can also be calculated by-hand (very simple case).
  Eigen::Matrix3d R_NG_expected;
  Eigen::Vector3d p_No_Go_N_expected, w_NG_N_expected, v_NGo_N_expected;
  Eigen::Vector3d alpha_NG_N_expected, a_NGo_N_expected;
  R_NG_expected << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  p_No_Go_N_expected << 0, 0, 1.261;
  w_NG_N_expected << 0, 0, 0;
  v_NGo_N_expected << 0, 0, 0;
  alpha_NG_N_expected << 0, 0, 0;
  a_NGo_N_expected << 0, 0, 0;

  CompareEndEffectorPositionVelocityVsExpectedSolution(q, q_Dt, q_DDt,
                                                       R_NG_expected,
                                                       p_No_Go_N_expected,
                                                       w_NG_N_expected,
                                                       v_NGo_N_expected,
                                                       alpha_NG_N_expected,
                                                       a_NGo_N_expected);
}


// Test accuracy of calculations for Kuka iiwa robot arm end-effector
// orientation, position, angular velocity, velocity for the situation when
// the Kuka arm is static with joint angles of 30 or 60 degrees.
GTEST_TEST(KukaIIwaRobot, ForwardKinematicsB) {
  const double q30 = M_PI / 6, q60 = M_PI / 3;
  const double qA = q60, qADt = 0.0, qADDt = 0.0;
  const double qB = q30, qBDt = 0.0, qBDDt = 0.0;
  const double qC = q60, qCDt = 0.0, qCDDt = 0.0;
  const double qD = q30, qDDt = 0.0, qDDDt = 0.0;
  const double qE = q60, qEDt = 0.0, qEDDt = 0.0;
  const double qF = q30, qFDt = 0.0, qFDDt = 0.0;
  const double qG = q60, qGDt = 0.0, qGDDt = 0.0;

  // Create a state with joint angles (q) and their time-derivatives (qDt).
  Eigen::Matrix<double, 7, 1> q, q_Dt, q_DDt;
  q << qA, qB, qC, qD, qE, qF, qG;
  q_Dt << qADt, qBDt, qCDt, qDDt, qEDt, qFDt, qGDt;
  q_DDt << qADDt, qBDDt, qCDDt, qDDDt, qEDDt, qFDDt, qGDDt;

  // MotionGenesis solution for these joint angles/time-derivatives.
  Eigen::Matrix3d R_NG_expected;
  Eigen::Vector3d p_No_Go_N_expected, w_NG_N_expected, v_NGo_N_expected;
  Eigen::Vector3d alpha_NG_N_expected, a_NGo_N_expected;
  R_NG_expected << -0.5939002959880204, 0.8043869080239565, -0.015625,
      -0.8043869080239565, -0.5932991120359317, 0.03094940802395521,
      0.015625, 0.03094940802395527, 0.9993988160479114;
  p_No_Go_N_expected << 0.2970356451892219, 0.1727696964662288,
      1.154681973689345;
  w_NG_N_expected << 0, 0, 0;
  v_NGo_N_expected << 0, 0, 0;
  alpha_NG_N_expected << 0, 0, 0;
  a_NGo_N_expected << 0, 0, 0;

  CompareEndEffectorPositionVelocityVsExpectedSolution(q, q_Dt, q_DDt,
                                                       R_NG_expected,
                                                       p_No_Go_N_expected,
                                                       w_NG_N_expected,
                                                       v_NGo_N_expected,
                                                       alpha_NG_N_expected,
                                                       a_NGo_N_expected);
}

}  // namespace
}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake

