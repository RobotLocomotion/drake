// Drake interface to the MotionGenesis solution for a 7-DOF KUKA LBR iiwa robot
// (14 kg payload) which is described at:
// https://www.kuka.com/en-de/products/robot-systems/industrial-robots/lbr-iiwa
// Geometry, joint-types, and mass/inertia properties are contained in:
// drake/multibody/benchmarks/kuka_iiwa_robot/kuka_iiwa_robot.urdf
//-----------------------------------------------------------------------------
#pragma once

#include <cmath>
#include <tuple>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace benchmarks {

using Eigen::Matrix3d;
using Eigen::Vector3d;

template <typename T>
class KukaIIwaRobot {
 public:
  KukaIIwaRobot() {}
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(KukaIIwaRobot);

  /// This method calculates kinematic properties of the end-effector (herein
  /// denoted as rigid body G) of a 7-DOF KUKA LBR iiwa robot (14 kg payload).
  /// Right-handed orthogonal unit vectors Nx, Ny, Nz are fixed in N (Earth)
  /// with Nz vertically upward and right-handed orthogonal unit vectors
  /// Gx, Gy, Gz are fixed in G.  The origin of frame N (Earth) is denoted No.
  /// The origin Go of end-effector G is located at G's inboard revolute joint.
  ///
  /// @param[in] q robot's joint angles (generalized coordinates).
  /// @param[in] v time-derivatives of q (generalized speeds).
  ///
  /// @returns Machine-precision values are returned as defined below.
  ///
  /// std::tuple | Description
  /// -----------|-------------------------------------------------
  /// R_NG       | Rotation matrix relating Nx, Ny, Nz to Gx, Gy, Gz.
  /// p_NoGo_N   | Go's position from No, expressed in N.
  /// w_NG_N     | G's angular velocity in N, expressed in N.
  /// v_NGo_N    | Go's velocity in N, expressed in N.
  std::tuple<Matrix3d, Vector3d, Vector3d, Vector3d>
  CalcForwardKinematicsEndEffector(const Eigen::Ref<const VectorX<T>>& q,
                                   const Eigen::Ref<const VectorX<T>>& v) const;
};

}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
