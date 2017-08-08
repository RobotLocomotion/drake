#pragma once

#include <cmath>
#include <tuple>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/MG/MG_kuka_iiwa_robot_auto_generated.h"
#include "drake/multibody/multibody_tree/math/spatial_force.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {
namespace MG {

using Eigen::Vector3d;
using Eigen::Matrix3d;
using SpatialForced = SpatialForce<double>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

/// This class is Drake's interface to the MotionGenesis solution for a
/// 7-DOF KUKA LBR iiwa robot (14 kg payload) which is described at:
/// https://www.kuka.com/en-de/products/robot-systems/industrial-robots/lbr-iiwa
/// Geometry, joint-types, and mass/inertia properties are contained in:
/// drake/multibody/benchmarks/kuka_iiwa_robot/kuka_iiwa_robot.urdf
///
/// There are 7 revolute motors that connect the Kuka robot's 7 rigid links,
/// herein named A, B, C, D, E, F, G.  Rigid link G is the robot's end-effector.
/// Right-handed orthogonal unit vectors Nx, Ny, Nz are fixed in N (Earth)
/// with Nz vertically upward.  The origin of frame N (Earth) is denoted No.
///
/// Earth (ground body N) is connected to the robot link A with an ideal
/// revolute motor.  The motor stator is frame Na (the motor's inboard frame)
/// which is fixed to N whereas the motor's rotor is frame/rigid link A, hence
/// frame A is the motor's outboard frame. Point Nao (origin of frame Na, welded
/// to N) is coincident with Ao (origin of frame A, welded to link A). Hence,
/// these points are at the same location -- but fixed to different objects.
/// Frame Na has orthogonal unit vectors Nax, Nay, Naz whereas link A has
/// orthogonal unit vectors Ax, Ay, Ax. The orientation of A relative to Na is
/// described by initially setting Ax = Nax, Ay = Nay, Az = Naz, and then
/// subjecting link A to a right handed rotation relative to Na about Naz = Az
/// (Naz = Az is parallel to the motor's axis of rotation).
///
/// Similarly, link A connects to link B at point Abo of frame Ab (fixed on A)
/// to origin Bo of frame B, which unit vector Abz = Bz.  Analogous connections
/// exist from B to C, C to D, ..., F to G (with motor revolute axes parallel to
/// the corresponding `z` unit vectors).  Hence F connects to the end-effector G
/// at origin Go of link G, and right-handed orthogonal unit vectors Gx, Gy, Gz
/// are fixed in G.
template<typename T>
class MGKukaIIwaRobot {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MGKukaIIwaRobot);

  /// Constructs an object that serves as Drake's interface to a Motion Genesis
  /// model of the aforementioned KUKA robot.  All model parameters are from:
  /// drake/multibody/benchmarks/kuka_iiwa_robot/kuka_iiwa_robot.urdf
  MGKukaIIwaRobot() {
    static_assert(std::is_convertible<T, double>::value,
                  "This class only supports T = double.");
  }

  /// This method calculates kinematic properties of the robot's end-effector
  /// (herein denoted as rigid body G) -- described above.
  ///
  /// @param[in] q robot's joint angles (generalized coordinates).
  /// @param[in] qDt 1st-time-derivatives of q (q̇).
  /// @param[in] qDDt 2nd-time-derivatives of q (q̈).
  ///
  /// @returns Machine-precision values as defined below.
  ///
  /// std::tuple | Description
  /// -----------|-------------------------------------------------
  /// R_NG       | Rotation matrix relating Nx, Ny, Nz to Gx, Gy, Gz.
  /// p_NoGo_N   | Go's position from No, expressed in N.
  /// w_NG_N     | G's angular velocity in N, expressed in N.
  /// v_NGo_N    | Go's velocity in N, expressed in N.
  /// alpha_NG_N | G's angular acceleration in N, expressed in N.
  /// a_NGo_N    | Go's acceleration in N, expressed in N.
  std::tuple<Matrix3d, Vector3d, Vector3d, Vector3d, Vector3d, Vector3d>
  CalcEndEffectorKinematics(const Eigen::Ref<const VectorX<T>>& q,
                            const Eigen::Ref<const VectorX<T>>& qDt,
                            const Eigen::Ref<const VectorX<T>>& qDDt) const;

  /// This method calculates joint reaction force/torques for the 7 revolute
  /// motors that connect frames Na to A, Ab to B, Bc to C, ... Fg to G.
  ///
  /// For example, there is a revolute motor between links A and B.
  /// The set Sᴮ of forces exerted on link B by the revolute motor
  /// can be replaced by an equivalent set consisting of a single force f_Bo
  /// applied to point Bo of B, together with a couple of torque t_B on link B
  /// (t_B is equal to the moment of the set Sᴮ of forces about point Bo).
  ///
  /// When the revolute motor is <b>massless</b>, one can prove that the set Sᴬ
  /// of forces exerted on link A by the revolute motor has an action/reaction
  /// effect on A.  Hence the set Sᴬ of forces on A can be replaced by an
  /// equivalent set consisting of a single force f_Abo = -f_Bo applied to the
  /// point Abo of A that is coincident with Bo, together with a couple of
  /// torque t_A = -t_B on link A (t_A is equal to the moment of the set Sᴬ of
  /// forces about point Abo).
  ///
  /// The combination of the force f_Bo and torque t_B can be stored in a
  /// spatial force defined as F_Bo = [f_Bo; t_B].  The spatial force can be
  /// expressed in whatever basis is helpful (e.g., for computational efficiency
  /// or for human-meaningful interpretation).
  ///
  /// Similarly, there is a revolute motor between links B and C.  The set of
  /// forces on C by that revolute motor is equivalent to the spatial force
  /// F_Co = [f_Co; t_C], where f_Co and t_C have analogous meanings as above.
  ///
  /// @param[in] q robot's joint angles (generalized coordinates).
  /// @param[in] qDt 1st-time-derivatives of q (q̇).
  /// @param[in] qDDt 2nd-time-derivatives of q (q̈).
  ///
  /// @returns Machine-precision values as defined below.
  ///
  /// std::tuple | Description
  /// -----------|-------------------------------------------------
  /// F_Ao_Na    | Spatial force on Ao from N, expressed in frame Na.
  /// F_Bo_Ab    | Spatial force on Bo from A, expressed in frame Ab.
  /// F_Co_Bc    | Spatial force on Co from B, expressed in frame Bc.
  /// F_Do_Cd    | Spatial force on Do from C, expressed in frame Cd.
  /// F_Eo_De    | Spatial force on Eo from D, expressed in frame De.
  /// F_Fo_Ef    | Spatial force on Fo from E, expressed in frame Ef.
  /// F_Go_Fg    | Spatial force on Go from F, expressed in frame Fg.
  std::tuple<SpatialForced, SpatialForced, SpatialForced, SpatialForced,
             SpatialForced, SpatialForced, SpatialForced>
  CalcJointReactionForces(const Eigen::Ref<const VectorX<T>>& q,
                          const Eigen::Ref<const VectorX<T>>& qDt,
                          const Eigen::Ref<const VectorX<T>>& qDDt) const;

  /// This method calculates the revolute motor torques for the 7 revolute
  /// motors that connect frames Na to A, Ab to B, Bc to C, ... Fg to G.
  ///
  /// For example, there is a z-axis revolute motor between ground N and link A.
  /// The purpose of the revolute motor is to generate a `driving torque' tAz
  /// on A, where tAz is the Az measure of the torque on A from the motor
  /// Similarly, there is a driving torque tBz on B, tCz on C, ... tGz on G.
  ///
  /// @param[in] q robot's joint angles (generalized coordinates).
  /// @param[in] qDt 1st-time-derivatives of q (q̇).
  /// @param[in] qDDt 2nd-time-derivatives of q (q̈).
  ///
  /// @returns 7x1 matrix of machine-precision values for the revolute motor
  /// driving torques tAz, tBz, tCz, tDz, tEz, tFz, tGz.
  Vector7d
  CalcRevoluteMotorZTorques(const Eigen::Ref<const VectorX<T>>& q,
                            const Eigen::Ref<const VectorX<T>>& qDt,
                            const Eigen::Ref<const VectorX<T>>& qDDt) const;

 private:
  // This method calculates all the output quantities designated by the
  // MotionGenesis auto-generated class MGKukaIIwaRobotAutoGenerated.
  //
  // @param[in] q robot's joint angles (generalized coordinates).
  // @param[in] qDt 1st-time-derivatives of q (q̇).
  // @param[in] qDDt 2nd-time-derivatives of q (q̈).
  //
  // @Note: There is no return value because all quantities are calculated
  //        and available as public members of MG_kuka_auto_generated.
  void CalcMGOutput(const Eigen::Ref<const VectorX<T>>& q,
                    const Eigen::Ref<const VectorX<T>>& qDt,
                    const Eigen::Ref<const VectorX<T>>& qDDt) const;

  // Class that holds MotionGenesis auto-generated code for Kuka robot.
  mutable MotionGenesis::MGKukaIIwaRobotAutoGenerated MG_kuka_auto_generated;
};

}  // namespace MG
}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
