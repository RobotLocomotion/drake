#include "drake/multibody/benchmarks/kuka_iiwa_robot/kuka_iiwa_robot.h"

#include <cmath>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace math {
namespace {

// Test forward kinematics for joint angles.
GTEST_TEST(KukaIIwaRobot, ForwardKinematics) {
  using benchmarks::KukaIIwaRobot;
  KukaIIwaRobot<double> kukaIIwaRobot;

  const double qA = 0.0;
  const double qB = 0.0;
  const double qC = 0.0;
  const double qD = 0.0;
  const double qE = 0.0;
  const double qF = 0.0;
  const double qG = 0.0;
  const double qADt = 0.0;
  const double qBDt = 0.0;
  const double qCDt = 0.0;
  const double qDDt = 0.0;
  const double qEDt = 0.0;
  const double qFDt = 0.0;
  const double qGDt = 0.0;

  Eigen::Matrix<double, 7, 1> joint_angles, joint_anglesDt;
  joint_angles << qA, qB, qC, qD, qE, qF, qG;
  joint_anglesDt << qADt, qBDt, qCDt, qDDt, qEDt, qFDt, qGDt;

  // R_NG       | Rotation matrix relating Nx, Ny, Nz to Gx, Gy, Gz.
  // p_NoGo_N   | Go's position from No, expressed in N.
  // w_NG_N     | G's angular velocity in N, expressed in N.
  // v_NGo_N    | Go's velocity in N, expressed in N.
  Eigen::Matrix3d R_NG_exact;
  Eigen::Vector3d p_No_Go_N_exact, w_NG_N_exact, v_NGo_N_exact;
  std::tie(R_NG_exact, p_No_Go_N_exact, w_NG_N_exact, v_NGo_N_exact) =
    kukaIIwaRobot.CalcForwardKinematicsEndEffector(joint_angles,
                                                   joint_anglesDt);

  // TODO(mitiguy) Connect to Drake's calculations when ready.
  Eigen::Matrix3d R_NG_drake;
  Eigen::Vector3d p_No_Go_N_drake, w_NG_N_drake, v_NGo_N_drake;
  R_NG_drake << 1, 0, 0,  0, 1, 0,  0, 0, 1;
  p_No_Go_N_drake << 0, 0, 1.261;  // Arm in straight-up position.
  w_NG_N_drake << 0, 0, 0;
  v_NGo_N_drake << 0, 0, 0;

  // Compare drake results with exact (MotionGenesis) results.
  constexpr double epsilon = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(R_NG_drake.isApprox(R_NG_exact, 10*epsilon));
  EXPECT_TRUE(p_No_Go_N_drake.isApprox(p_No_Go_N_exact, 10*epsilon));
  EXPECT_TRUE(w_NG_N_drake.isApprox(w_NG_N_exact, 10*epsilon));
  EXPECT_TRUE(v_NGo_N_drake.isApprox(v_NGo_N_exact, 10*epsilon));
}

}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
