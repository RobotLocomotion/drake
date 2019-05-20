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
using test_utilities::SpatialKinematicsPVA;
using SpatialForced = SpatialForce<double>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

// Compare Drake's MultibodyTree forces and motion with MotionGenesis solution.
void TestKukaArmInverseDynamics(const Eigen::Ref<const VectorX<double>>& q,
                                const Eigen::Ref<const VectorX<double>>& qDt,
                                const Eigen::Ref<const VectorX<double>>& qDDt,
                                const double gravity) {
  // Kinematics: Get Drake's end-effector information, including:
  // R_NG       | Rotation matrix relating Nx, Ny, Nz to Gx, Gy, Gz.
  // p_NoGo_N   | Go's position from No, expressed in N.
  // w_NG_N     | G's angular velocity in N, expressed in N.
  // v_NGo_N    | Go's velocity in N, expressed in N.
  // alpha_NG_N | G's angular acceleration in N, expressed in N.
  // a_NGo_N    | Go's acceleration in N, expressed in N.
  DrakeKukaIIwaRobot<double> drake_kuka_robot(gravity);
  const SpatialKinematicsPVA<double> drake_kinematics =
      drake_kuka_robot.CalcEndEffectorKinematics(q, qDt, qDDt);

  // Kinematics: Get corresponding MotionGenesis information.
  MG::MGKukaIIwaRobot<double> MG_kuka_robot(gravity);
  const SpatialKinematicsPVA<double> MG_kinematics =
      MG_kuka_robot.CalcEndEffectorKinematics(q, qDt, qDDt);

  // Kinematics: Compare Drake results with MotionGenesis (expected) results.
  constexpr double kEpsilon = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(drake_kinematics.rotation_matrix().matrix().isApprox(
      MG_kinematics.rotation_matrix().matrix(), kEpsilon));
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


  // Inverse dynamics: Get Drake's joint forces/torques, including:
  // --------|-------------------------------------------------
  // F_Ao_W  | Spatial force on Ao from N, expressed in frame W (world).
  // F_Bo_W  | Spatial force on Bo from A, expressed in frame W (world).
  // F_Co_W  | Spatial force on Co from B, expressed in frame W (world).
  // F_Do_W  | Spatial force on Do from C, expressed in frame W (world).
  // F_Eo_W  | Spatial force on Eo from D, expressed in frame W (world).
  // F_Fo_W  | Spatial force on Fo from E, expressed in frame W (world).
  // F_Go_W  | Spatial force on Go from F, expressed in frame W (world).
  // In Drake, this is done by creating a vector of the aforementioned 7 spatial
  // forces and then calls a method to fill this vector of spatial forces.
  // Consider a generic body B whose inboard frame (welded to B) is M with
  // origin Mo.  The spatial force on B at Mo expressed in W (world) if F_BMo_W.
  const int number_of_links = drake_kuka_robot.get_number_of_rigid_bodies();
  std::vector<SpatialForce<double>> F_BMo_W_array(number_of_links);
  const KukaRobotJointReactionForces<double> forces =
      drake_kuka_robot.CalcJointReactionForces(q, qDt, qDDt);

  // Inverse dynamics: Get corresponding MotionGenesis information.
  SpatialForced F_Ao_W, F_Bo_W, F_Co_W, F_Do_W, F_Eo_W, F_Fo_W, F_Go_W;
  std::tie(F_Ao_W, F_Bo_W, F_Co_W, F_Do_W, F_Eo_W, F_Fo_W, F_Go_W) =
      MG_kuka_robot.CalcJointReactionForcesExpressedInWorld(q, qDt, qDDt);

  // Inverse dynamics: Compare Drake results with expected results.
  const double tolerance = 40 * kEpsilon;
  EXPECT_TRUE(F_Ao_W.IsApprox(forces.F_Ao_W, tolerance));
  EXPECT_TRUE(F_Bo_W.IsApprox(forces.F_Bo_W, tolerance));
  EXPECT_TRUE(F_Co_W.IsApprox(forces.F_Co_W, tolerance));
  EXPECT_TRUE(F_Do_W.IsApprox(forces.F_Do_W, tolerance));
  EXPECT_TRUE(F_Eo_W.IsApprox(forces.F_Eo_W, tolerance));
  EXPECT_TRUE(F_Fo_W.IsApprox(forces.F_Fo_W, tolerance));
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
  const double gravityA = 0;
  const double gravityB = 9.8;
  TestKukaArmInverseDynamics(q, qDt, qDDt, gravityA);
  TestKukaArmInverseDynamics(q, qDt, qDDt, gravityB);

  // Test 2: Includes non-zero velocity, centripetal acceleration, ...
  qAdot = 0.1;
  qBdot = 0.2;
  qCdot = 0.3;
  qDdot = 0.4;
  qEdot = 0.5;
  qFdot = 0.6;
  qGdot = 0.7;
  qDt << qAdot, qBdot, qCdot, qDdot, qEdot, qFdot, qGdot;
  TestKukaArmInverseDynamics(q, qDt, qDDt, gravityA);
  TestKukaArmInverseDynamics(q, qDt, qDDt, gravityB);

  // Test 3: Includes non-zero velocity and more general non-zero acceleration.
  qAddot = 0.7;
  qBddot = 0.6;
  qCddot = 0.5;
  qDddot = 0.4;
  qEddot = 0.3;
  qFddot = 0.2;
  qGddot = 0.1;
  qDDt << qAddot, qBddot, qCddot, qDddot, qEddot, qFddot, qGddot;
  TestKukaArmInverseDynamics(q, qDt, qDDt, gravityA);
  TestKukaArmInverseDynamics(q, qDt, qDDt, gravityB);
}


}  // namespace
}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
