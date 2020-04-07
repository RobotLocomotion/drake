#include "drake/multibody/tree/door_hinge.h"

#include <cmath>
#include <tuple>

namespace drake {
namespace multibody {

using internal::MultibodyTree;
using internal::PositionKinematicsCache;
using internal::VelocityKinematicsCache;
using symbolic::Expression;
using systems::Context;

namespace {
// See class description for notation.
template <typename T>
std::tuple<T, T, T> CalcApproximationCurves(const T& t, const T& x) {
  DRAKE_THROW_UNLESS(t > 0);
  using std::tanh;
  const T s = tanh(x / t);
  const T singlet = 1. - s * s;        // = t * d/dx s
  const T doublet = 2. * s * singlet;  // = -t² * d²/dx² s
  return {s, singlet, doublet};
}
}  // namespace

template <typename T>
DoorHinge<T>::DoorHinge(const RevoluteJoint<T>& joint,
                        const DoorHingeConfig& config)
    : DoorHinge(joint.model_instance(), joint.index(), config) {}

template <typename T>
DoorHinge<T>::DoorHinge(ModelInstanceIndex model_instance,
                        JointIndex joint_index, const DoorHingeConfig& config)
    : ForceElement<T>(model_instance),
      joint_index_(joint_index),
      config_(config) {
  DRAKE_THROW_UNLESS(config_.spring_constant >= 0);
  DRAKE_THROW_UNLESS(config_.dynamic_friction_torque >= 0);
  DRAKE_THROW_UNLESS(config_.static_friction_torque >= 0);
  DRAKE_THROW_UNLESS(config_.viscous_friction >= 0);
  DRAKE_THROW_UNLESS(config_.catch_width >= 0);
  DRAKE_THROW_UNLESS(config_.motion_threshold >= 0);
}

template <typename T>
const RevoluteJoint<T>& DoorHinge<T>::joint() const {
  const RevoluteJoint<T>* joint = dynamic_cast<const RevoluteJoint<T>*>(
      &this->get_parent_tree().get_joint(joint_index_));
  DRAKE_DEMAND(joint != nullptr);
  return *joint;
}

template <typename T>
T DoorHinge<T>::CalcPotentialEnergy(const Context<T>& context,
                                    const PositionKinematicsCache<T>&) const {
  const T& angle = joint().get_angle(context);
  return CalcHingeStoredEnergy(angle);
}

template <typename T>
T DoorHinge<T>::CalcConservativePower(const Context<T>& context,
                                      const PositionKinematicsCache<T>&,
                                      const VelocityKinematicsCache<T>&) const {
  const T& angle = joint().get_angle(context);
  const T& angular_rate = joint().get_angular_rate(context);
  return CalcHingeConservativePower(angle, angular_rate);
}

template <typename T>
T DoorHinge<T>::CalcNonConservativePower(
    const Context<T>& context, const PositionKinematicsCache<T>&,
    const VelocityKinematicsCache<T>&) const {
  const T& angular_rate = joint().get_angular_rate(context);
  return CalcHingeNonConservativePower(angular_rate);
}

template <typename T>
void DoorHinge<T>::DoCalcAndAddForceContribution(
    const Context<T>& context, const PositionKinematicsCache<T>&,
    const VelocityKinematicsCache<T>&, MultibodyForces<T>* forces) const {
  const T& angle = joint().get_angle(context);
  const T& angular_rate = joint().get_angular_rate(context);
  const T torque = CalcHingeTorque(angle, angular_rate);
  joint().AddInTorque(context, torque, forces);
}

template <typename T>
T DoorHinge<T>::CalcHingeFrictionalTorque(const T& angular_rate) const {
  if (config_.motion_threshold == 0) {
    return config_.viscous_friction * angular_rate;
  } else {
    const auto [s, singlet, doublet] =
        CalcApproximationCurves(T(config_.motion_threshold), angular_rate);
    drake::unused(singlet);
    return -(config_.dynamic_friction_torque * s +
             config_.static_friction_torque * doublet +
             config_.viscous_friction * angular_rate);
  }
}

template <typename T>
T DoorHinge<T>::CalcHingeSpringTorque(const T& angle) const {
  if (config_.catch_width == 0) {
    return -config_.spring_constant * (angle - config_.spring_zero_angle_rad);
  } else {
    const T catch_center = config_.catch_width / 2;
    const auto [s, singlet, doublet] =
        CalcApproximationCurves(catch_center, angle - catch_center);
    drake::unused(s);
    drake::unused(singlet);
    return config_.catch_torque * doublet -
           config_.spring_constant * (angle - config_.spring_zero_angle_rad);
  }
}

template <typename T>
T DoorHinge<T>::CalcHingeTorque(const T& angle,
                                const T& angular_rate) const {
  return CalcHingeFrictionalTorque(angular_rate) +
         CalcHingeSpringTorque(angle);
}

template <typename T>
T DoorHinge<T>::CalcHingeConservativePower(const T& angle,
                                           const T& angular_rate) const {
  return angular_rate * CalcHingeSpringTorque(angle);
}

template <typename T>
T DoorHinge<T>::CalcHingeNonConservativePower(const T& angular_rate) const {
  return angular_rate * CalcHingeFrictionalTorque(angular_rate);
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
T DoorHinge<T>::CalcHingeStoredEnergy(const T& angle) const {
  T energy = 0.;
  // Compute spring torque energy.
  const T spring_offset = (angle - config_.spring_zero_angle_rad);
  const T spring_offset_init = (0. - config_.spring_zero_angle_rad);
  energy +=
      0.5 * config_.spring_constant *
      (spring_offset * spring_offset - spring_offset_init * spring_offset_init);

  // Compute catch torque energy.
  if (config_.catch_width != 0) {
    const T catch_center = config_.catch_width / 2;
    const auto [s_q, singlet_q, doublet_q] =
        CalcApproximationCurves(catch_center, angle - catch_center);
    drake::unused(s_q);
    drake::unused(doublet_q);

    const auto [s_0, singlet_0, doublet_0] =
        CalcApproximationCurves(catch_center, -catch_center);
    drake::unused(s_0);
    drake::unused(doublet_0);
    energy += config_.catch_torque * catch_center * (singlet_q - singlet_0);
  }
  return energy;
}

// Drake template boilerplate:
template <typename T>
template <typename ToScalar>
std::unique_ptr<ForceElement<ToScalar>> DoorHinge<T>::TemplatedClone(
    const MultibodyTree<ToScalar>&) const {
  // N.B. We can't use std::make_unique here since this constructor is private
  // to std::make_unique.
  // N.B. We use the private constructor since it doesn't rely on a valid joint
  // reference, which might not be available during cloning.
  std::unique_ptr<DoorHinge<ToScalar>> door_hinge_clone(
      new DoorHinge<ToScalar>(this->model_instance(), joint_index_, config_));

  return door_hinge_clone;
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

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::DoorHinge)
