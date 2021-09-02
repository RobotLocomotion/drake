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
SpatialAcceleration<double> CalcSpatialAccelerationViaAutomaticDifferentiation(
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
  auto q_autodiff = math::InitializeAutoDiff(q, Eigen::MatrixXd(qdot));
  auto v_autodiff = math::InitializeAutoDiff(v, Eigen::MatrixXd(vdot));

  // Convert the double plant to an AutoDiffXd plant.
  // Then, create a default context for the AutoDiffXd plant.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
      systems::System<double>::ToAutoDiffXd(plant);
  std::unique_ptr<systems::Context<AutoDiffXd>> context_autodiff =
      plant_autodiff->CreateDefaultContext();

  context_autodiff->SetTimeStateAndParametersFrom(context);

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

  // Shift spatial velocity from Bo to Bq.
  const math::RotationMatrix<AutoDiffXd> R_AB_autodiff =
      frame_B_autodiff.CalcRotationMatrix(*context_autodiff, frame_A_autodiff);
  const Vector3<AutoDiffXd> p_BoBq_B_autodiff(p_BoBq_B);
  const Vector3<AutoDiffXd> p_BoBq_A_autodiff =
      R_AB_autodiff * p_BoBq_B_autodiff;
  const SpatialVelocity<AutoDiffXd> V_ABq_A_autodiff =
      V_AB_A_autodiff.Shift(p_BoBq_A_autodiff);

  // Form spatial acceleration via AutoDiffXd results.
  const SpatialAcceleration<double> A_ABq_A(
      math::ExtractGradient(V_ABq_A_autodiff.get_coeffs()));

  // Shortcut return if frame_A == frame_E.
  if (frame_E.index() == frame_A.index()) return A_ABq_A;

  // Otherwise, express the result in frame_E.
  const math::RotationMatrix<double> R_EA =
      frame_A.CalcRotationMatrix(context, frame_E);
  const SpatialAcceleration<double> A_ABq_E = R_EA * A_ABq_A;
  return A_ABq_E;
}

// Returns A_AB_E, frame B's spatial acceleration in a frame_A, expressed in a
// frame E, evaluated at values of q, v, which are passed to this method
// in the arguments `context`.
// @param[in] plant The plant associated with the system and context.
// @param[in] context The state of the multibody system.
// @param[in] frame_B The frame for which spatial acceleration is calculated.
// @param[in] frame_A The measured-in frame for spatial acceleration.
// @param[in] frame_E The expressed-in frame for spatial acceleration.
SpatialAcceleration<double>
    CalcSpatialAccelerationViaAutomaticDifferentiation(
    const MultibodyPlant<double>& plant,
    const systems::Context<double>& context,
    const Frame<double>& frame_B,
    const Frame<double>& frame_A,
    const Frame<double>& frame_E) {
  // Spatial acceleration is a function of the generalized accelerations vdot.
  // Use forward dynamics to calculate values for vdot (for the given q, v).
  const systems::ContinuousState<double>& derivs =
      plant.EvalTimeDerivatives(context);
  const VectorX<double> vdot(derivs.get_generalized_velocity().CopyToVector());
  EXPECT_EQ(vdot.size(), plant.num_velocities());

  const Vector3<double> p_BoBo_B = Vector3<double>::Zero();
  return CalcSpatialAccelerationViaAutomaticDifferentiation(
      plant, context, vdot, frame_B, p_BoBo_B, frame_A, frame_E);
}

}  // namespace test_utilities
}  // namespace multibody
}  // namespace drake
