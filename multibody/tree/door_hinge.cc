#include "drake/multibody/tree/door_hinge.h"

#include <tuple>

using drake::AutoDiffXd;
using drake::multibody::ForceElement;
using drake::multibody::Joint;
using drake::multibody::MultibodyForces;
using drake::multibody::RevoluteJoint;
using drake::multibody::internal::MultibodyTree;
using drake::multibody::internal::PositionKinematicsCache;
using drake::multibody::internal::VelocityKinematicsCache;
using drake::symbolic::Expression;
using drake::systems::Context;

namespace drake {
namespace multibody {
namespace {
// See class description for notation.
template <typename T>
std::tuple<T, T, T> CalcApproximationCurves(T t, T x) {
  DRAKE_THROW_UNLESS(t > 0);
  using std::tanh;
  const T s = tanh(x / t);
  const T singlet = 1. - s * s;        // = t * d/dx s
  const T doublet = 2. * s * singlet;  // = -t² * d²/dx² s
  return {s, singlet, doublet};
}
}  // namespace

template <typename T>
T DoorHinge<T>::CalcPotentialEnergy(const Context<T>& context,
                                    const PositionKinematicsCache<T>&) const {
  const T angle = joint_.GetOnePosition(context);
  return CalcHingeStoredEnergy(angle, config_);
}

template <typename T>
T DoorHinge<T>::CalcConservativePower(const Context<T>& context,
                                      const PositionKinematicsCache<T>&,
                                      const VelocityKinematicsCache<T>&) const {
  const T angle = joint_.GetOnePosition(context);
  const T angular_velocity = joint_.GetOneVelocity(context);
  return CalcHingeConservativePower(angle, angular_velocity, config_);
}

template <typename T>
T DoorHinge<T>::CalcNonConservativePower(
    const Context<T>& context, const PositionKinematicsCache<T>&,
    const VelocityKinematicsCache<T>&) const {
  const T angular_velocity = joint_.GetOneVelocity(context);
  return CalcHingeNonConservativePower(angular_velocity, config_);
}

template <typename T>
void DoorHinge<T>::DoCalcAndAddForceContribution(
    const Context<T>& context, const PositionKinematicsCache<T>&,
    const VelocityKinematicsCache<T>&, MultibodyForces<T>* forces) const {
  const T angle = joint_.GetOnePosition(context);
  const T angular_velocity = joint_.GetOneVelocity(context);
  const T torque = CalcHingeTorque(angle, angular_velocity, config_);
  joint_.AddInTorque(context, torque, forces);
}

template <typename T>
T DoorHinge<T>::CalcHingeFrictionalTorque(T angular_velocity,
                                          const DoorHingeConfig& config) const {
  if (config.motion_threshold == 0) {
    return config.viscous_friction * angular_velocity;
  } else {
    const auto [s, singlet, doublet] =
        CalcApproximationCurves(T(config.motion_threshold), angular_velocity);
    drake::unused(singlet);
    return -(config.dynamic_friction_torque * s +
             config.static_friction_torque * doublet +
             config.viscous_friction * angular_velocity);
  }
}

template <typename T>
T DoorHinge<T>::CalcHingeSpringTorque(T angle,
                                      const DoorHingeConfig& config) const {
  if (config.catch_width == 0) {
    return -config.spring_constant * (angle - config.spring_zero_angle_rad);
  } else {
    const T catch_center = config.catch_width / 2;
    const auto [s, singlet, doublet] =
        CalcApproximationCurves(catch_center, angle - catch_center);
    drake::unused(s);
    drake::unused(singlet);
    return config.catch_torque * doublet -
           config.spring_constant * (angle - config.spring_zero_angle_rad);
  }
}

template <typename T>
T DoorHinge<T>::CalcHingeTorque(T angle, T angular_velocity,
                                const DoorHingeConfig& config) const {
  return CalcHingeFrictionalTorque(angular_velocity, config) +
         CalcHingeSpringTorque(angle, config);
}

template <typename T>
T DoorHinge<T>::CalcHingeConservativePower(
    T angle, T angular_velocity, const DoorHingeConfig& config) const {
  return angular_velocity * CalcHingeSpringTorque(angle, config);
}

template <typename T>
T DoorHinge<T>::CalcHingeNonConservativePower(
    T angular_velocity, const DoorHingeConfig& config) const {
  return angular_velocity * CalcHingeFrictionalTorque(angular_velocity, config);
}

// Stored energy consists two parts. One is from the spring and another one is
// from the catch torque. The potential energy of the spring is derived as:
// E_{spring_torque} = -∫τ_ts dq = -k_ts∫(q − qs₀)dq = 0.5 * k_ts * (q − qs₀)²
// − 0.5 * k_ts * (q₀ − qs₀)² with q₀ = 0 as defined.
// Similarly, the catch torque can be viewed as a nonlinear virtual spring, the
// potential energy is the integration of the torque over the angle that the
// torque is in effective, i.e. q ∈ [0, qc₀]. To make the derivation clear, we
// have the following definition to simplify the math expressions:
// f(t,x) ≜ singlet(t,x)
// g(t,x) ≜ doublet(t,x)
// By definition, g(t, x) = -1/t*df(t, x)/dx, hence ∫g(t,x) dx = −t*f(t,x).
// Please refer to function `CalcApproximationCurves` for the details.
// With the above definition, we have τ_c = k_c g(t,x) and
// E_{catch_torque} = -∫τ_cdx = -k_c∫g(t,x)dx = k_c*t[f(t, x) - f(t, x₀)].
// Substituting x₀ = 0, t = qc₀/2, x = q - qc₀/2 and dx = dq, we have
// E_{catch_torque} = k_c*qc₀/2*[f(qc₀/2, q-qc₀/2) - f(qc₀/2, -qc₀/2)].
template <typename T>
T DoorHinge<T>::CalcHingeStoredEnergy(T angle,
                                      const DoorHingeConfig& config) const {
  T energy = 0.;
  // Compute spring torque energy.
  const T spring_offset = (angle - config.spring_zero_angle_rad);
  const T spring_offset_init = (0. - config.spring_zero_angle_rad);
  energy +=
      0.5 * config.spring_constant *
      (spring_offset * spring_offset - spring_offset_init * spring_offset_init);

  // Compute catch torque energy.
  if (config.catch_width != 0) {
    const T catch_center = config.catch_width / 2;
    const auto [s_q, singlet_q, doublet_q] =
        CalcApproximationCurves(catch_center, angle - catch_center);
    drake::unused(s_q);
    drake::unused(doublet_q);

    const auto [s_0, singlet_0, doublet_0] =
        CalcApproximationCurves(catch_center, -catch_center);
    drake::unused(s_0);
    drake::unused(doublet_0);
    energy += config.catch_torque * catch_center * (singlet_q - singlet_0);
  }
  return energy;
}

// Drake template boilerplate:
template <typename T>
template <typename ToScalar>
std::unique_ptr<ForceElement<ToScalar>> DoorHinge<T>::TemplatedClone(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const RevoluteJoint<ToScalar>& joint_clone =
      dynamic_cast<const RevoluteJoint<ToScalar>&>(
          tree_clone.get_joint(joint().index()));
  return std::make_unique<DoorHinge<ToScalar>>(joint_clone, config_);
}

template <typename T>
std::unique_ptr<ForceElement<double>> DoorHinge<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedClone(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>> DoorHinge<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedClone(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<Expression>> DoorHinge<T>::DoCloneToScalar(
    const MultibodyTree<Expression>& tree_clone) const {
  return TemplatedClone(tree_clone);
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(class DoorHinge)

}  // namespace multibody
}  // namespace drake
