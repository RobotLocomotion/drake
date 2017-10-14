#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_acceleration.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test_utilities {

/// Utility class containing the spatial position, velocity, and acceleration
/// of a rigid frame B in another arbitrary rigid frame N (e.g., the world).
/// Herein, Bo and No denote generic points (e.g., the origins) of rigid frames
/// B and N, respectively.  Right-handed sets of orthogonal unit vectors
/// Bx, By, Bz and Nx, Ny, Nz are fixed in rigid frames B and N, respectively.
/// --------|-------------------------------------------------------------------
/// X_NB    | Isometry3 containing R_NB (rotation matrix relating Nx, Ny, Nz to
///         | Bx, By, Bz) and p_NoBo_N (position from No to Bo, expressed in N).
/// V_NBo_N | Spatial velocity that contains w_NB_N (B's angular velocity in N,
///         | expressed in N) and v_NBo_N (Bo's velocity in N, expressed in N).
/// A_NBo_N | Spatial acceleration with alpha_NB_N (B's angular acceleration in
///         | N, expressed in N) and a_NBo_N (Bo's acceleration in N, expressed
///         | in N).
template <typename T>
class SpatialKinematicsPVA {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialKinematicsPVA)

  /// Constructor that populates the members of this class.
  /// @param[in] X kinematics spatial kinematics representation of this object,
  /// i.e., pose, spatial velocity, and spatial acceleration.
  SpatialKinematicsPVA(const Isometry3<T>& X_NB,
                       const SpatialVelocity<T>& V_NBo_N,
                       const SpatialAcceleration<T>& A_NBo_N) {
    X_NB_ = X_NB;
    V_NBo_N_ = V_NBo_N;
    A_NBo_N_ = A_NBo_N;
  }

  Isometry3<T> X_NB_;
  SpatialVelocity<T> V_NBo_N_;
  SpatialAcceleration<T> A_NBo_N_;
};

}  // namespace test_utilities
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
