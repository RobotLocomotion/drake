#include "drake/multibody/tree/revolute_spring.h"

#include <limits>
#include <utility>
#include <vector>

#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
RevoluteSpring<T>::RevoluteSpring(const RevoluteJoint<T>& joint,
                                  double free_length, double stiffness) :
    ForceElement<T>(joint.model_instance()),
    joint_(joint),
    free_length_(free_length),
    stiffness_(stiffness) {}

template <typename T>
void RevoluteSpring<T>::DoCalcAndAddForceContribution(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>&,
    const internal::VelocityKinematicsCache<T>&,
    MultibodyForces<T>* forces) const {

  const T delta = free_length_ - joint_.get_angle(context);
  const T torque = stiffness_ * delta;
  joint_.AddInTorque(context, torque, forces);
}

template <typename T>
T RevoluteSpring<T>::CalcPotentialEnergy(const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>&) const {

  const T delta = free_length_ - joint_.get_angle(context);

  return 0.5 * stiffness_ * delta * delta;
}

template <typename T>
T RevoluteSpring<T>::CalcConservativePower(const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>&,
    const internal::VelocityKinematicsCache<T>&) const {
  // Since the potential energy is:
  //  V = 1/2⋅k⋅(q-q₀)²
  // The conservative power is defined as:
  //  Pc = -d(V)/dt, or -k
  // being positive when the potential energy decreases.

  const T delta = free_length_ - joint_.get_angle(context);
  const T delta_dot = -joint_.get_angular_rate(context);

  // Since V = 1/2⋅k⋅(q-q₀)² we have that, from its definition:
  // Pc = -d(V)/dt = -k⋅(q-q₀)⋅dq/dt
  const T Pc = -stiffness_ * delta * delta_dot;
  return Pc;
}

template <typename T>
T RevoluteSpring<T>::CalcNonConservativePower(const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>&,
    const internal::VelocityKinematicsCache<T>&) const {
  // Purely conservative spring
  return 0;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<ForceElement<ToScalar>>
RevoluteSpring<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const RevoluteJoint<ToScalar>& joint_clone =
      dynamic_cast<const RevoluteJoint<ToScalar>&>(
          tree_clone.get_joint(joint().index()));

  // Make the clone.
  auto spring_clone = std::make_unique<RevoluteSpring<ToScalar>>(
      joint_clone, free_length(), stiffness());

  return std::move(spring_clone);
}

template <typename T>
std::unique_ptr<ForceElement<double>>
RevoluteSpring<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>>
RevoluteSpring<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<symbolic::Expression>>
RevoluteSpring<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RevoluteSpring)
