#include "drake/multibody/benchmarks/kuka_iiwa_robot/kuka_iiwa_robot_model.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/test/MG_kuka_iiwa_robot_glue.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace benchmarks {
namespace {

using Eigen::Vector3d;

// Verify methods for MultibodyTree::CalcPositionKinematicsCache(), comparing
// its computed results to a MotionGenesis solution.
GTEST_TEST(KukaIIwaRobotKinematics, KinematicsTestA) {
  // Create Drake's Kuka iiwa robot.
  KukaIIwaRobotTestKinematics drake_kuka_robot;

  // Provide values for joint angles and joint rates.
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

  // Get Drake's end-effector pose (transform) and spatial velocity.
  // Use these to create the following:
  // R_NG       | Rotation matrix relating Nx, Ny, Nz to Gx, Gy, Gz.
  // p_NoGo_N   | Go's position from No, expressed in N.
  // w_NG_N     | G's angular velocity in N, expressed in N.
  // v_NGo_N    | Go's velocity in N, expressed in N.
  Eigen::Isometry3d X_NG;
  SpatialVelocity<double> V_NG_N;
  std::tie(X_NG, V_NG_N) =
      drake_kuka_robot.CalcEndEffectorPoseAndSpatialVelocity(q.data(),
                                                             q_Dt.data());
  const Matrix3d R_NG = X_NG.linear();
  const Vector3d p_NoGo_N = X_NG.translation();
  const Vector3d w_NG_N = V_NG_N.rotational();
  const Vector3d v_NGo_N = V_NG_N.translational();

  MGKukaIIwaRobotGlue<double> MG_kuka_robot;
  // R_NG       | Rotation matrix relating Nx, Ny, Nz to Gx, Gy, Gz.
  // p_NoGo_N   | Go's position from No, expressed in N.
  // w_NG_N     | G's angular velocity in N, expressed in N.
  // v_NGo_N    | Go's velocity in N, expressed in N.
  Eigen::Matrix3d R_NG_true;
  Eigen::Vector3d p_NoGo_N_true, w_NG_N_true, v_NGo_N_true;
  Eigen::Vector3d alpha_NG_N_true, a_NGo_N_true;
  std::tie(R_NG_true, p_NoGo_N_true, w_NG_N_true, v_NGo_N_true,
           alpha_NG_N_true, a_NGo_N_true) =
      MG_kuka_robot.CalcEndEffectorKinematics(q, q_Dt, q_DDt);

  // Compare actual results with expected (true) results.
  constexpr double kEpsilon = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(R_NG.isApprox(R_NG_true, kEpsilon));
  EXPECT_TRUE(p_NoGo_N.isApprox(p_NoGo_N_true, kEpsilon));
  EXPECT_TRUE(w_NG_N.isApprox(w_NG_N_true, kEpsilon));
  EXPECT_TRUE(v_NGo_N.isApprox(v_NGo_N_true, kEpsilon));
}


}  // namespace
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
