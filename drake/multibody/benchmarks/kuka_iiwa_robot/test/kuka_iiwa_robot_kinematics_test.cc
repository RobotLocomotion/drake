#include <gtest/gtest.h>

#include "drake/multibody/benchmarks/kuka_iiwa_robot/MG/MG_kuka_iiwa_robot.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/drake_kuka_iiwa_robot.h"

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
  kuka_iiwa_robot::MGKukaIIwaRobot<double> MG_kuka_robot;
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

}  // namespace
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
