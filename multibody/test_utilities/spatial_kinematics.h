#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_acceleration.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace multibody {
namespace test_utilities {

// For a frame Bq that has a fixed offset from a frame_B, returns A_ABq_E,
// frame Bq's spatial acceleration in a frame_A, expressed in a frame E, and
// evaluated at values of q, v, v̇ which are passed to this method via
// the arguments `context` and `vdot` (generalized accelerations).
// @param[in] plant The plant associated with the system and context.
// @param[in] context Contains the state of the multibody system.
// @param[in] vdot Generalized acceleration values used for this calculation.
// @param[in] frame_B The frame to which frame Bq is fixed/welded.
// @param[in] p_BoBq_B Position vector from Bo (frame_B's origin) to the origin
//            of frame Bq, expressed in frame_B.
// @param[in] frame_A The measured-in frame for spatial acceleration.
// @param[in] frame_E The expressed-in frame for spatial acceleration.
// TODO(Mitiguy) Merge these functions developed in connection with PR #13593
//  with the associated function implemented in PR #13773 into a utility file.
SpatialAcceleration<double> CalcSpatialAccelerationViaSpatialVelocityDerivative(
    const MultibodyPlant<double>& plant,
    const systems::Context<double>& context,
    const VectorX<double>& vdot,
    const Frame<double>& frame_B,
    const Vector3<double>& p_BoBq_B,
    const Frame<double>& frame_A,
    const Frame<double>& frame_E) {

  // Enable q_autodiff and v_autodiff to differentiate with respect to time.
  // Note: Pass MatrixXd() so the return gradient uses AutoDiffXd (for which we
  // do have explicit instantiations) instead of AutoDiffScalar<Matrix1d>.
  const VectorX<double> q = plant.GetPositions(context);
  const VectorX<double> v = plant.GetVelocities(context);
  VectorX<double> qdot(plant.num_positions());
  plant.MapVelocityToQDot(context, v, &qdot);
  auto q_autodiff =
      math::initializeAutoDiffGivenGradientMatrix(q, Eigen::MatrixXd(qdot));
  auto v_autodiff =
      math::initializeAutoDiffGivenGradientMatrix(v, Eigen::MatrixXd(vdot));

  // Convert the double plant to an AutoDiffXd plant.
  // Then, create a default context for the AutoDiffXd plant.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
      systems::System<double>::ToAutoDiffXd(plant);
  std::unique_ptr<systems::Context<AutoDiffXd>> context_autodiff =
      plant_autodiff->CreateDefaultContext();

  // Aggregate the state into a temporary vector for AutoDiffXd computations.
  // Set the context for AutoDiffXd computations.
  VectorX<AutoDiffXd> x_autodiff(plant.num_multibody_states());
  x_autodiff << q_autodiff, v_autodiff;
  plant_autodiff->GetMutablePositionsAndVelocities(context_autodiff.get()) =
      x_autodiff;

  // Using AutoDiff, compute V_AB_A (frame B's spatial velocity in frame_A,
  // expressed in frame_A), and its time derivative which is A_AB_A
  // (frame B's spatial acceleration in frame_A, expressed in frame_A).
  const Frame<AutoDiffXd>& frame_A_autodiff =
      plant_autodiff->get_frame(frame_A.index());
  const Frame<AutoDiffXd>& frame_B_autodiff =
      plant_autodiff->get_frame(frame_B.index());
  const SpatialVelocity<AutoDiffXd> V_AB_A_autodiff =
      frame_B_autodiff.CalcSpatialVelocity(*context_autodiff, frame_A_autodiff,
                                           frame_A_autodiff);

  // Form spatial acceleration via AutoDiffXd results.
  const SpatialAcceleration<double> A_AB_A(
      math::autoDiffToGradientMatrix(V_AB_A_autodiff.get_coeffs()));

  // Shift translational acceleration to point Q.
  SpatialAcceleration<double> A_ABq_A = A_AB_A;  // Calculation continues ,,,
  if (!p_BoBq_B.isZero()) {
    const math::RotationMatrix<double> R_AB =
        frame_B.CalcRotationMatrix(context, frame_A);
    const Vector3<double> p_BoBq_A = R_AB * p_BoBq_B;
    const Vector3<double> w_AB_A =
        math::autoDiffToValueMatrix(V_AB_A_autodiff.rotational());
    A_ABq_A.ShiftInPlace(p_BoBq_A,  w_AB_A);
  }

  // Shortcut return if frame_A == frame_E.
  if (frame_E.index() == frame_A.index()) return A_ABq_A;

  // Otherwise, express the result in frame_E.
  const math::RotationMatrix<double> R_EA =
      frame_A.CalcRotationMatrix(context, frame_E);
  const SpatialAcceleration<double> A_ABq_E = R_EA * A_ABq_A;
  return A_ABq_E;
}

// Returns A_AB_E, frame B's spatial acceleration in a frame_A, expressed in
// a frame E, evaluated at values of q, v, v̇ which are passed to this method
// in the arguments `context` and `vdot` (generalized accelerations).
// @param[in] plant The plant associated with the system and context.
// @param[in] context The state of the multibody system.
// @param[in] vdot Generalized acceleration values used for this calculation.
// @param[in] frame_B The frame for which spatial acceleration is calculated.B.
// @param[in] frame_A The measured-in frame for spatial acceleration.
// @param[in] frame_E The expressed-in frame for spatial acceleration.
SpatialAcceleration<double> CalcSpatialAccelerationViaSpatialVelocityDerivative(
    const MultibodyPlant<double>& plant,
    const systems::Context<double>& context,
    const VectorX<double>& vdot,
    const Frame<double>& frame_B,
    const Frame<double>& frame_A,
    const Frame<double>& frame_E) {
  const Vector3<double>& p_BoQ_B = Vector3<double>::Zero();
  return CalcSpatialAccelerationViaSpatialVelocityDerivative(plant, context,
      vdot, frame_B, p_BoQ_B, frame_A, frame_E);
}

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
