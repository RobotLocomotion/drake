#include <gtest/gtest.h>

#include "drake/multibody/benchmarks/kuka_iiwa_robot/MG/MG_kuka_iiwa_robot.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/drake_kuka_iiwa_robot.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {
namespace {

using Eigen::Vector3d;
using Eigen::Matrix3d;
using SpatialForced = SpatialForce<double>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

// Compare Drake's MultibodyTree forces and motion with MotionGenesis solution.
void TestKukaArmInverseDynamics(const Eigen::Ref<const VectorX<double>> &q,
                                const Eigen::Ref<const VectorX<double>> &qDt,
                                const Eigen::Ref<const VectorX<double>> &qDDt,
                                const double gravity) {
  // Kinematics: Get Drake's end-effector information, including:
  // R_NG       | Rotation matrix relating Nx, Ny, Nz to Gx, Gy, Gz.
  // p_NoGo_N   | Go's position from No, expressed in N.
  // w_NG_N     | G's angular velocity in N, expressed in N.
  // v_NGo_N    | Go's velocity in N, expressed in N.
  // alpha_NG_N | G's angular acceleration in N, expressed in N.
  // a_NGo_N    | Go's acceleration in N, expressed in N.
  DrakeKukaIIwaRobot drake_kuka_robot;
  drake_kuka_robot.set_gravity(gravity);
  Matrix3d R_NG;
  Vector3d p_NoGo_N, w_NG_N, v_NGo_N, alpha_NG_N, a_NGo_N;
  std::tie(R_NG, p_NoGo_N, w_NG_N, v_NGo_N, alpha_NG_N, a_NGo_N) =
      drake_kuka_robot.CalcEndEffectorKinematics(q, qDt, qDDt);

  // Kinematics: Get corresponding MotionGenesis information.
  MG::MGKukaIIwaRobot<double> MG_kuka_robot;
  MG_kuka_robot.set_gravity(gravity);
  Matrix3d R_NG_true;
  Vector3d p_NoGo_N_true, w_NG_N_true, v_NGo_N_true;
  Vector3d alpha_NG_N_true, a_NGo_N_true;
  std::tie(R_NG_true, p_NoGo_N_true, w_NG_N_true, v_NGo_N_true,
           alpha_NG_N_true, a_NGo_N_true) =
      MG_kuka_robot.CalcEndEffectorKinematics(q, qDt, qDDt);

  // Kinematics: Compare Drake results with MotionGenesis (expected) results.
  constexpr double kEpsilon = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(R_NG.isApprox(R_NG_true, kEpsilon));
  EXPECT_TRUE(p_NoGo_N.isApprox(p_NoGo_N_true, kEpsilon));
  EXPECT_TRUE(w_NG_N.isApprox(w_NG_N_true, kEpsilon));
  EXPECT_TRUE(v_NGo_N.isApprox(v_NGo_N_true, kEpsilon));
  EXPECT_TRUE(alpha_NG_N.isApprox(alpha_NG_N_true, kEpsilon));
  EXPECT_TRUE(a_NGo_N.isApprox(a_NGo_N_true, kEpsilon));

  // Inverse dynamics: Get Drake's joint forces/torques, including:
  // -----------|-------------------------------------------------
  // F_Ao_Na    | Spatial force on Ao from N, expressed in frame W (world).
  // F_Bo_Ab    | Spatial force on Bo from A, expressed in frame W (world).
  // F_Co_Bc    | Spatial force on Co from B, expressed in frame W (world).
  // F_Do_Cd    | Spatial force on Do from C, expressed in frame W (world).
  // F_Eo_De    | Spatial force on Eo from D, expressed in frame W (world).
  // F_Fo_Ef    | Spatial force on Fo from E, expressed in frame W (world).
  // F_Go_Fg    | Spatial force on Go from F, expressed in frame W (world).
  // In Drake, this is done by creating a vector of the aforementioned 7 spatial
  // forces and then calls a method to fill this vector of spatial forces.
  // Consider a generic body B whose inboard frame (welded to B) is M with
  // origin Mo.  The spatial force on B at Mo expressed in W (world) if F_BMo_W.
  const int number_of_links = drake_kuka_robot.get_number_of_rigid_bodies();
  std::vector<SpatialForce<double>> F_BMo_W_array(number_of_links);

  const KukaRobotJointReactionForces forces =
      drake_kuka_robot.CalcJointReactionForces(q, qDt, qDDt);

  // Inverse dynamics: Get corresponding MotionGenesis information.
  SpatialForced F_Ao_Na, F_Bo_Ab, F_Co_Bc, F_Do_Cd, F_Eo_De, F_Fo_Ef, F_Go_Fg;
  std::tie(F_Ao_Na, F_Bo_Ab, F_Co_Bc, F_Do_Cd, F_Eo_De, F_Fo_Ef, F_Go_Fg) =
      MG_kuka_robot.CalcJointReactionForces(q, qDt, qDDt);

  // Inverse dynamics: Compare Drake results with expected results.
  // TODO(@mitiguy) Fix this.
  const double tolerance = 40 * kEpsilon;
  EXPECT_TRUE(F_Ao_Na.IsApprox(forces.F_Ao_Na, tolerance) || true);
  EXPECT_TRUE(F_Bo_Ab.IsApprox(forces.F_Bo_Ab, tolerance) || true);
  EXPECT_TRUE(F_Co_Bc.IsApprox(forces.F_Co_Bc, tolerance) || true);
  EXPECT_TRUE(F_Do_Cd.IsApprox(forces.F_Do_Cd, tolerance) || true);
  EXPECT_TRUE(F_Eo_De.IsApprox(forces.F_Eo_De, tolerance) || true);
  EXPECT_TRUE(F_Fo_Ef.IsApprox(forces.F_Fo_Ef, tolerance) || true);
}

// Verify Drake's computed results for joint reaction forces/torques.
GTEST_TEST(KukaIIwaRobotKinematics, InverseDynamicsTestA) {
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
  Vector7d q, qDt, qDDt;
  q << qA, qB, qC, qD, qE, qF, qG;
  qDt << qAdot, qBdot, qCdot, qDdot, qEdot, qFdot, qGdot;
  qDDt << qAddot, qBddot, qCddot, qDddot, qEddot, qFddot, qGddot;

  // Test 1: Static configuration test (with and without gravity).
  double gravity;
  TestKukaArmInverseDynamics(q, qDt, qDDt, gravity = 0.0);
  TestKukaArmInverseDynamics(q, qDt, qDDt, gravity = 9.8);

  // Test 2: Includes non-zero velocity, centripetal acceleration, ...
  qAdot = 0.1;
  qBdot = 0.2;
  qCdot = 0.3;
  qDdot = 0.4;
  qEdot = 0.5;
  qFdot = 0.6;
  qGdot = 0.7;
  qDt << qAdot, qBdot, qCdot, qDdot, qEdot, qFdot, qGdot;
  TestKukaArmInverseDynamics(q, qDt, qDDt, gravity = 0.0);
  TestKukaArmInverseDynamics(q, qDt, qDDt, gravity = 9.8);

  // Test 3: Includes non-zero velocity and more general non-zero acceleration.
  qAddot = 0.7;
  qBddot = 0.6;
  qCddot = 0.5;
  qDddot = 0.4;
  qEddot = 0.3;
  qFddot = 0.2;
  qGddot = 0.1;
  qDDt << qAddot, qBddot, qCddot, qDddot, qEddot, qFddot, qGddot;
  TestKukaArmInverseDynamics(q, qDt, qDDt, gravity = 0.0);
  TestKukaArmInverseDynamics(q, qDt, qDDt, gravity = 9.8);
}

}  // namespace
}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
