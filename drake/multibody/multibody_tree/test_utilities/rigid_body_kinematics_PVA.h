#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/test_utilities/spatial_kinematics_PVA.h"

using drake::multibody::multibody_tree::test_utilities::SpatialKinematicsPVA;

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test_utilities {

/// Utility class containing the position, velocity, and acceleration kinematics
/// of a rigid frame B in another arbitrary rigid frame N (e.g., the world).
/// Herein, Bo and No denote generic points (e.g., the origins) of rigid frames
/// B and N, respectively.  Right-handed sets of orthogonal unit vectors
/// Bx, By, Bz and Nx, Ny, Nz are fixed in rigid frames B and N, respectively.
/// -----------|----------------------------------------------------------------
/// R_NB       | Rotation matrix relating Nx, Ny, Nz to Bx, By, Bz.
/// p_NoBo_N   | Position vector from No to Bo, expressed in frame N.
/// w_NB_N     | B's angular velocity in N, expressed in N.
/// v_NBo_N    | Bo's velocity in N, expressed in N.
/// alpha_NB_N | B's angular acceleration in N, expressed in N.
/// a_NBo_N    | Bo's acceleration in N, expressed in N.
template <typename T>
class RigidBodyKinematicsPVA {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RigidBodyKinematicsPVA)

  /// Constructor that expands information from a SpatialKinematicsPVA.
  /// @param[in] kinematics spatial kinematics representation of this object,
  /// i.e., pose, spatial velocity, and spatial acceleration.
  explicit RigidBodyKinematicsPVA(const SpatialKinematicsPVA<T>& kinematics) {
    R_NB_ = kinematics.X_NB_.linear();
    p_NoBo_N_ = kinematics.X_NB_.translation();
    w_NB_N_ = kinematics.V_NBo_N_.rotational();
    v_NBo_N_ = kinematics.V_NBo_N_.translational();
    alpha_NB_N_ = kinematics.A_NBo_N_.rotational();
    a_NBo_N_ = kinematics.A_NBo_N_.translational();
  }

  Matrix3<T> R_NB_;
  Vector3<T> p_NoBo_N_;
  Vector3<T> w_NB_N_;
  Vector3<T> v_NBo_N_;
  Vector3<T> alpha_NB_N_;
  Vector3<T> a_NBo_N_;
};

}  // namespace test_utilities
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
