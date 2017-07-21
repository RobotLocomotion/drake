#include "drake/multibody/benchmarks/kuka_iiwa_robot/drake_kuka_iiwa_robot.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/test/MG_kuka_iiwa_robot_glue.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace benchmarks {
namespace {

using Eigen::Vector3d;
using Eigen::Matrix3d;

// Compare Drake's MultibodyTree kinematics with MotionGenesis solution.
void TestEndEffectorKinematics(const Eigen::Ref<const VectorX<double>>& q,
                               const Eigen::Ref<const VectorX<double>>& qDt,
                               const Eigen::Ref<const VectorX<double>>& qDDt) {
  // Get Drake's end-effector information, including:
  // R_NG       | Rotation matrix relating Nx, Ny, Nz to Gx, Gy, Gz.
  // p_NoGo_N   | Go's position from No, expressed in N.
  // w_NG_N     | G's angular velocity in N, expressed in N.
  // v_NGo_N    | Go's velocity in N, expressed in N.
  // alpha_NG_N | G's angular acceleration in N, expressed in N.
  // a_NGo_N    | Go's acceleration in N, expressed in N.
  kuka_iiwa_robot::DrakeKukaIIwaRobot drake_kuka_robot;
  Matrix3d R_NG;
  Vector3d p_NoGo_N, w_NG_N, v_NGo_N, alpha_NG_N, a_NGo_N;
  std::tie(R_NG, p_NoGo_N, w_NG_N, v_NGo_N, alpha_NG_N, a_NGo_N) =
      drake_kuka_robot.CalcEndEffectorKinematics(q, qDt, qDDt);

  // Get corresponding MotionGenesis information.
  kuka_iiwa_robot::MGKukaIIwaRobotGlue<double> MG_kuka_robot;
  Matrix3d R_NG_true;
  Vector3d p_NoGo_N_true, w_NG_N_true, v_NGo_N_true;
  Vector3d alpha_NG_N_true, a_NGo_N_true;
  std::tie(R_NG_true, p_NoGo_N_true, w_NG_N_true, v_NGo_N_true,
           alpha_NG_N_true, a_NGo_N_true) =
      MG_kuka_robot.CalcEndEffectorKinematics(q, qDt, qDDt);

  // Compare actual results with expected (true) results.
  constexpr double kEpsilon = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(R_NG.isApprox(R_NG_true, kEpsilon));
  EXPECT_TRUE(p_NoGo_N.isApprox(p_NoGo_N_true, kEpsilon));
  EXPECT_TRUE(w_NG_N.isApprox(w_NG_N_true, kEpsilon));
  EXPECT_TRUE(v_NGo_N.isApprox(v_NGo_N_true, kEpsilon));
  EXPECT_TRUE(alpha_NG_N.isApprox(alpha_NG_N_true, kEpsilon));
  EXPECT_TRUE(a_NGo_N.isApprox(a_NGo_N_true, kEpsilon));
}

// Verify methods for MultibodyTree::CalcPositionKinematicsCache(), comparing
// its computed results to a MotionGenesis solution.
GTEST_TEST(KukaIIwaRobotKinematics, KinematicsTestA) {
  // Provide values for joint angles q and their 1st and 2nd time-derivatives.
  // The values below correspond to a static configuration.
  double q30 = M_PI / 6, q60 = M_PI / 3;
  double qA = q60, qADt = 0.0, qADDt = 0.0;
  double qB = q30, qBDt = 0.0, qBDDt = 0.0;
  double qC = q60, qCDt = 0.0, qCDDt = 0.0;
  double qD = q30, qDDt = 0.0, qDDDt = 0.0;
  double qE = q60, qEDt = 0.0, qEDDt = 0.0;
  double qF = q30, qFDt = 0.0, qFDDt = 0.0;
  double qG = q60, qGDt = 0.0, qGDDt = 0.0;

  // Create a state with joint angles (q) and their time-derivatives (qDt).
  Eigen::Matrix<double, 7, 1> q, q_Dt, q_DDt;
  q << qA, qB, qC, qD, qE, qF, qG;
  q_Dt << qADt, qBDt, qCDt, qDDt, qEDt, qFDt, qGDt;
  q_DDt << qADDt, qBDDt, qCDDt, qDDDt, qEDDt, qFDDt, qGDDt;

  // Static configuration test.
  TestEndEffectorKinematics(q, q_Dt, q_DDt);

  qADt = 0.1;
  qBDt = 0.2;
  qCDt = 0.3;
  qDDt = 0.4;
  qEDt = 0.5;
  qFDt = 0.6;
  qGDt = 0.7;
  q_Dt << qADt, qBDt, qCDt, qDDt, qEDt, qFDt, qGDt;

  // More general test includes non-zero velocity, centrifugal acceleration.
  TestEndEffectorKinematics(q, q_Dt, q_DDt);

  qADDt = 0.7;
  qBDDt = 0.6;
  qCDDt = 0.5;
  qDDDt = 0.4;
  qEDDt = 0.3;
  qFDDt = 0.2;
  qGDDt = 0.1;
  q_DDt << qADDt, qBDDt, qCDDt, qDDDt, qEDDt, qFDDt, qGDDt;

  // Most general of these three tests non-zero velocity and acceleration.
  TestEndEffectorKinematics(q, q_Dt, q_DDt);
}

}  // namespace
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
