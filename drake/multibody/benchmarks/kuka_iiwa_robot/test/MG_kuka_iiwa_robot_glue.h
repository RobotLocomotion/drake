#pragma once

#include <cmath>
#include <tuple>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace MG_kuka_iiwa_robot {

using Eigen::Vector3d;
using Eigen::Matrix3d;

/// This class is Drake's interface to the MotionGenesis solution for a
/// 7-DOF KUKA LBR iiwa robot (14 kg payload) which is described at:
/// https://www.kuka.com/en-de/products/robot-systems/industrial-robots/lbr-iiwa
/// Geometry, joint-types, and mass/inertia properties are contained in:
/// drake/multibody/benchmarks/kuka_iiwa_robot/kuka_iiwa_robot.urdf
template<typename T>
class MGKukaIIwaRobotGlue {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MGKukaIIwaRobotGlue);

  /// Constructs an object that serves as Drake's interface to a Motion Genesis
  /// model of the aforementioned KUKA robot.  All model parameters are from:
  /// drake/multibody/benchmarks/kuka_iiwa_robot/kuka_iiwa_robot.urdf
  MGKukaIIwaRobotGlue() {}

  /// This method calculates kinematic properties of the end-effector (herein
  /// denoted as rigid body G) of a 7-DOF KUKA LBR iiwa robot (14 kg payload).
  /// Right-handed orthogonal unit vectors Nx, Ny, Nz are fixed in N (Earth)
  /// with Nz vertically upward and right-handed orthogonal unit vectors
  /// Gx, Gy, Gz are fixed in G.  The origin of frame N (Earth) is denoted No.
  /// The origin Go of end-effector G is located at G's inboard revolute joint.
  ///
  /// @param[in] q robot's joint angles (generalized coordinates).
  /// @param[in] v 1st-time-derivatives of q (q̇).
  /// @param[in] a 2nd-time-derivatives of q (q̈).
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
  CalcEndEffectorKinematics(const Eigen::Ref<const VectorX<T>> &q,
                            const Eigen::Ref<const VectorX<T>> &v,
                            const Eigen::Ref<const VectorX<T>> &a) const;
};

}  // namespace MG_kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
