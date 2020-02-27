#include "drake/multibody/tree/door_hinge.h"

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
// Approximates {x<0: -1 ; x>0: 1} outside of -threshold < x < threshold.
// That is, this is a step.
template <typename T>
T sigmoid(T x, T threshold) {
  return tanh(x / threshold);
}

// First derivative of the sigmoid -- a hump at 0 that integrates to 2.
template <typename T>
T singlet(T x, T threshold) {
  return 1 - sigmoid(x, threshold) * sigmoid(x, threshold);
}

// Second derivative of the sigmoid -- a lump at negative x that integrates to
// 1 and a lump at positive x that integrates to -1.
template <typename T>
T doublet(T x, T threshold) {
  return 2 * sigmoid(x, threshold) * singlet(x, threshold);
}
}  // namespace

namespace internal {
// Free functions to compute the dynamic properties of the hinge in
// angle/torque terms, separate from Drake idioms.

template <typename T>
T hinge_frictional_torque(T angle, T angular_velocity,
                          const DoorHingeConfig& config) {
  drake::unused(angle);
  T torque = 0.;
  if (config.dynamic_friction_torque) {
    torque -= (config.dynamic_friction_torque *
               sigmoid(angular_velocity, T(config.motion_threshold)));
  }
  if (config.static_friction_torque) {
    torque -= (config.static_friction_torque *
               doublet(angular_velocity, T(config.motion_threshold)));
  }
  torque -= angular_velocity * config.viscous_friction;
  return torque;
}
// For testing, ensure this symbol exists.
template double hinge_frictional_torque<double>(double, double,
                                                const DoorHingeConfig&);

template <typename T>
T hinge_spring_torque(T angle, T angular_velocity,
                      const DoorHingeConfig& config) {
  drake::unused(angular_velocity);
  T torque = 0.;
  const T catch_center = config.catch_width / 2;
  if (config.catch_torque) {
    torque += doublet(angle - catch_center, catch_center) * config.catch_torque;
  }
  torque -= (angle - config.spring_zero_angle_rad) * config.spring_constant;
  return torque;
}
// For testing, ensure this symbol exists.
template double hinge_spring_torque<double>(double, double,
                                            const DoorHingeConfig&);

template <typename T>
T hinge_torque(T angle, T angular_velocity, const DoorHingeConfig& config) {
  T result = hinge_frictional_torque(angle, angular_velocity, config) +
             hinge_spring_torque(angle, angular_velocity, config);
  return result;
}

template <typename T>
T hinge_conservative_power(T angle, T angular_velocity,
                           const DoorHingeConfig& config) {
  return angular_velocity *
         hinge_spring_torque(angle, angular_velocity, config);
}

template <typename T>
T hinge_nonconservative_power(T angle, T angular_velocity,
                              const DoorHingeConfig& config) {
  return angular_velocity *
         hinge_frictional_torque(angle, angular_velocity, config);
}

template <typename T>
T hinge_stored_energy(T angle, T angular_velocity,
                      const DoorHingeConfig& config) {
  drake::unused(angular_velocity);
  T energy = 0.;
  const T catch_center = config.catch_width / 2;
  energy +=
      (singlet(angle - catch_center, catch_center) * config.catch_torque) /
      config.catch_width;
  const T spring_offset = (angle - config.spring_zero_angle_rad);
  energy += 0.5 * spring_offset * spring_offset * config.spring_constant;
  return energy;
}
}  // namespace internal

template <typename T>
T DoorHinge<T>::CalcPotentialEnergy(const Context<T>& context,
                                    const PositionKinematicsCache<T>&) const {
  const T angle = joint_.GetOnePosition(context);
  const T angular_velocity = joint_.GetOneVelocity(context);
  return internal::hinge_stored_energy(angle, angular_velocity, config_);
}

template <typename T>
T DoorHinge<T>::CalcConservativePower(const Context<T>& context,
                                      const PositionKinematicsCache<T>&,
                                      const VelocityKinematicsCache<T>&) const {
  const T angle = joint_.GetOnePosition(context);
  const T angular_velocity = joint_.GetOneVelocity(context);
  return internal::hinge_conservative_power(angle, angular_velocity, config_);
}

template <typename T>
T DoorHinge<T>::CalcNonConservativePower(
    const Context<T>& context, const PositionKinematicsCache<T>&,
    const VelocityKinematicsCache<T>&) const {
  const T angle = joint_.GetOnePosition(context);
  const T angular_velocity = joint_.GetOneVelocity(context);
  return internal::hinge_nonconservative_power(angle, angular_velocity,
                                               config_);
}

template <typename T>
void DoorHinge<T>::DoCalcAndAddForceContribution(
    const Context<T>& context, const PositionKinematicsCache<T>&,
    const VelocityKinematicsCache<T>&, MultibodyForces<T>* forces) const {
  const T angle = joint_.GetOnePosition(context);
  const T angular_velocity = joint_.GetOneVelocity(context);
  const T torque = internal::hinge_torque(angle, angular_velocity, config_);
  joint_.AddInTorque(context, torque, forces);
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
