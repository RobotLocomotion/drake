#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"

#include <limits>
#include <utility>
#include <vector>

#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

using math::RigidTransform;
using math::RotationMatrix;

template <typename T>
LinearBushingRollPitchYaw<T>::LinearBushingRollPitchYaw(
    const Frame<T>& frameA, const Frame<T>& frameC,
    const Vector3<double>& torque_stiffness_constants,
    const Vector3<double>& torque_damping_constants,
    const Vector3<double>& force_stiffness_constants,
    const Vector3<double>& force_damping_constants)
    : LinearBushingRollPitchYaw(frameC.model_instance(),
                                frameA.index(), frameC.index(),
                                torque_stiffness_constants,
                                torque_damping_constants,
                                force_stiffness_constants,
                                force_damping_constants) {}

template <typename T>
LinearBushingRollPitchYaw<T>::LinearBushingRollPitchYaw(
    ModelInstanceIndex model_instance,
    FrameIndex frameA_index, FrameIndex frameC_index,
    const Vector3<double>& torque_stiffness_constants,
    const Vector3<double>& torque_damping_constants,
    const Vector3<double>& force_stiffness_constants,
    const Vector3<double>& force_damping_constants)
    : ForceElement<T>(model_instance),
      frameA_index_(frameA_index),
      frameC_index_(frameC_index),
      torque_stiffness_constants_(torque_stiffness_constants),
      torque_damping_constants_(torque_damping_constants),
      force_stiffness_constants_(force_stiffness_constants),
      force_damping_constants_(force_damping_constants) {
  DRAKE_THROW_UNLESS(torque_stiffness_constants.minCoeff() >= 0);
  DRAKE_THROW_UNLESS(torque_damping_constants.minCoeff() >= 0);
  DRAKE_THROW_UNLESS(force_stiffness_constants.minCoeff() >= 0);
  DRAKE_THROW_UNLESS(force_damping_constants.minCoeff() >= 0);
}

template <typename T>
void LinearBushingRollPitchYaw<T>::DoCalcAndAddForceContribution(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */,
    MultibodyForces<T>* forces) const {

  // Form F_Ao_A, the spatial force at point Ao of frame A due to the bushing.
  const SpatialForce<T> F_Ao_A = CalcBushingSpatialForceOnFrameA(context);

  // Form F_Ao_W by expressing F_Ao_A in the world frame W.
  const RotationMatrix<T> R_WA = frameA().CalcRotationMatrixInWorld(context);
  const SpatialForce<T> F_Ao_W = R_WA * F_Ao_A;

  // The next calculation needs the position from Aₒ (frame A's origin) to L0ₒ
  // (the origin of link L0), expressed in the world frame W.
  const RigidTransform<T>& X_WL0 = link0().EvalPoseInWorld(context);
  const RotationMatrix<T>& R_WL0 = X_WL0.rotation();
  const Vector3<T> p_L0Ao_L0 =
      frameA().CalcPoseInBodyFrame(context).translation();
  const Vector3<T> p_AoL0_W = -(R_WL0 * p_L0Ao_L0);

  // Form the spatial force F_L0_W by shifting the spatial force F_Ao_W from
  // Aₒ (A's origin) to L0ₒ (link L0's origin).
  const SpatialForce<T> F_L0_W = F_Ao_W.Shift(p_AoL0_W);

  // The next calculation needs the position from L0 to L1 expressed in world.
  const Vector3<T>& p_WoL0_W = X_WL0.translation();
  const Vector3<T>& p_WoL1_W = link1().EvalPoseInWorld(context).translation();
  const Vector3<T> p_L0L1_W = p_WoL1_W - p_WoL0_W;

  // Form the spatial force F_L1_W by shifting the spatial force F_L0_W from L0ₒ
  // (link L0's origin) to L1ₒ (link L1's origin) and negating the result.
  const SpatialForce<T> F_L1_W = -(F_L0_W.Shift(p_L0L1_W));

  // Alias to the array of spatial forces applied to each link (body).
  std::vector<SpatialForce<T>>& F_BodyOrigin_W_array =
      forces->mutable_body_forces();

  // Apply a torque to link L0 and apply the force −𝐟 to L0ₒ.
  // Apply a torque to link L1 and apply the force +𝐟 to L1ₒ.
  F_BodyOrigin_W_array[link0().node_index()] += F_L0_W;
  F_BodyOrigin_W_array[link1().node_index()] += F_L1_W;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcPotentialEnergy(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& /* pc */) const {
  // TODO(Mitiguy) Per issues #12982 and #12752, implement this method.
  //  Currently this method has not been implemented and throws an exception.
  throw std::runtime_error(
      "Error: LinearBushingRollPitchYaw::CalcPotentialEnergy() "
      "has not been implemented.  Related: Issues #12982 and #12752.");
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcConservativePower(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */) const {
  // TODO(Mitiguy) Per issues #12982 and #12752, implement the following method.
  //  Currently this method has not been implemented and throws an exception.
  throw std::runtime_error(
      "Error: LinearBushingRollPitchYaw::CalcConservativePower() "
      "has not been implemented.  Related: Issues #12982 and #12752.");;
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcNonConservativePower(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */) const {
  // TODO(Mitiguy) Per issues #12982 and #12752, implement the following method.
  //  Currently this method has not been implemented and throws an exception.
  throw std::runtime_error(
      "Error: LinearBushingRollPitchYaw::CalcNonConservativePower() "
      "has not been implemented.  Related: Issues #12982 and #12752.");
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<ForceElement<ToScalar>>
LinearBushingRollPitchYaw<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>&) const {
  const Vector3<double>& k012 = torque_stiffness_constants();
  const Vector3<double>& d012 = torque_damping_constants();
  const Vector3<double>& kxyz = force_stiffness_constants();
  const Vector3<double>& dxyz = force_damping_constants();

  // The declaration <typename U> friend class LinearBushingRollPitchYaw
  // is needed to facilitate the _private_ use of constructor below.
  std::unique_ptr<LinearBushingRollPitchYaw<ToScalar>> bushing_clone(
      new LinearBushingRollPitchYaw<ToScalar>(this->model_instance(),
          frameA_index_, frameC_index_, k012, d012, kxyz, dxyz));

  return bushing_clone;
}

template <typename T>
std::unique_ptr<ForceElement<double>>
LinearBushingRollPitchYaw<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>>
LinearBushingRollPitchYaw<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<symbolic::Expression>>
LinearBushingRollPitchYaw<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::LinearBushingRollPitchYaw)
