#pragma once

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_algebra.h"

namespace drake {
namespace multibody {
namespace test_utilities {

/// Utility class containing the transform and spatial velocity/acceleration
/// of an arbitrary frame B in another arbitrary frame N (e.g., the world).
/// Herein, Bo and No denote generic points (e.g., the origins) of frames B and
/// N, respectively.  Right-handed sets of orthogonal unit vectors Bx, By, Bz
/// and Nx, Ny, Nz are fixed in frames B and N, respectively.
template <typename T>
class SpatialKinematicsPVA {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpatialKinematicsPVA)

  /// Constructor that populates the members of this class.
  /// @param[in] X_NB transform relating the orientation/position of N to B.
  /// @param[in] V_NBo_N B's spatial velocity in N, expressed in N.
  /// @param[in] A_NBo_N B's spatial acceleration in N, expressed in N.
  SpatialKinematicsPVA(const math::RigidTransform<T>& X_NB,
                       const SpatialVelocity<T>& V_NBo_N,
                       const SpatialAcceleration<T>& A_NBo_N) :
                       X_NB_(X_NB), V_NBo_N_(V_NBo_N), A_NBo_N_(A_NBo_N) {}

  /// Constructor that populates the members of this class.
  /// @param[in] R_NB 3x3 rotation matrix relating Nx, Ny, Nz to Bx, By, Bz.
  /// @param[in] p_NoBo_N position vector from No to Bo, expressed in N.
  /// @param[in] w_NB_N B's angular velocity in N, expressed in N.
  /// @param[in] v_NBo_N Bo's translational velocity in N, expressed in N.
  /// @param[in] alpha_NB_N B's angular acceleration in N, expressed in N.
  /// @param[in] a_NBo_N Bo's translational acceleration in N, expressed in N.
  SpatialKinematicsPVA(const math::RotationMatrix<T>& R_NB,
                       const Vector3<T>& p_NoBo_N,
                       const Vector3<T>& w_NB_N,
                       const Vector3<T>& v_NBo_N,
                       const Vector3<T>& alpha_NB_N,
                       const Vector3<T>& a_NBo_N) :
                       V_NBo_N_(w_NB_N, v_NBo_N),
                       A_NBo_N_(alpha_NB_N, a_NBo_N) {
    X_NB_.set_rotation(R_NB);
    X_NB_.set_translation(p_NoBo_N);
  }

  /// Returns the transform that contains the 3x3 rotation matrix relating
  /// Nx, Ny, Nz to Bx, By, Bz and the position from No to Bo, expressed in N).
  /// @retval X_NB transform containing orientation R_NB and position p_NoBo_N.
  const math::RigidTransform<T>& transform() const { return X_NB_; }

  /// Returns the spatial velocity that contains B's angular velocity in N
  /// expressed in N and Bo's velocity in N, expressed in N.
  /// @retval V_NBo_N spatial velocity containing w_NB_N and v_NBo_N.
  const SpatialVelocity<T>& spatial_velocity() const { return V_NBo_N_; }

  /// Returns the spatial acceleration that contains B's angular velocity in N
  /// expressed in N and Bo's velocity in N, expressed in N.
  /// @retval A_NBo_N spatial acceleration containing alpha_NB_N and a_NBo_N.
  const SpatialAcceleration<T>& spatial_acceleration() const {
    return A_NBo_N_;
  }

  /// Returns the 3x3 rotation matrix relating Nx, Ny, Nz to Bx, By, Bz.
  /// @retval R_NB, the 3x3 rotation matrix relating frames N and B.
  const math::RotationMatrix<T>& rotation_matrix() const {
    return X_NB_.rotation();
  }

  /// Returns the position vector from No to Bo, expressed in N.
  /// @retval p_NoBo_N, the position vector from No to Bo, expressed in N.
  const Vector3<T>& position_vector() const { return X_NB_.translation(); }

  /// Returns B's angular velocity in N, expressed in N.
  /// @retval w_NB_N, B's angular velocity in N, expressed in N.
  const Vector3<T>& angular_velocity() const { return V_NBo_N_.rotational(); }

  /// Returns Bo's translational velocity in N, expressed in N.
  /// @retval v_NBo_N, Bo's translational velocity in N, expressed in N.
  const Vector3<T>& translational_velocity() const {
    return V_NBo_N_.translational();
  }

  /// Returns B's angular acceleration in N, expressed in N.
  /// @retval alpha_NB_N, B's angular acceleration in N, expressed in N.
  const Vector3<T>& angular_acceleration() const {
    return A_NBo_N_.rotational();
  }

  /// Returns Bo's acceleration in N, expressed in N.
  /// @retval a_NBo_N, Bo's acceleration in N, expressed in N.
  const Vector3<T>& translational_acceleration() const {
    return A_NBo_N_.translational();
  }

 private:
  // Transform containing R_NB (3x3 rotation matrix relating Nx, Ny, Nz to
  // Bx, By, Bz) and p_NoBo_N (position from No to Bo, expressed in N).
  math::RigidTransform<T> X_NB_;

  // Spatial velocity containing w_NB_N (B's angular velocity in N, expressed
  // in N) and v_NBo_N (Bo's velocity in N, expressed in N).
  SpatialVelocity<T> V_NBo_N_;

  // Spatial acceleration containing alpha_NB_N (B's angular acceleration in N,
  // expressed in N) and a_NBo_N (Bo's acceleration in N, expressed in N).
  SpatialAcceleration<T> A_NBo_N_;
};

}  // namespace test_utilities
}  // namespace multibody
}  // namespace drake
