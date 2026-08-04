#include "drake/multibody/tree/prismatic_spring.h"

#include <memory>

#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {
namespace multibody {

template <typename T>
PrismaticSpring<T>::PrismaticSpring(const PrismaticJoint<T>& joint,
                                    double nominal_position, double stiffness)
    : PrismaticSpring(joint.model_instance(), joint.index(), nominal_position,
                      stiffness) {}

template <typename T>
PrismaticSpring<T>::PrismaticSpring(ModelInstanceIndex model_instance,
                                    JointIndex joint_index,
                                    double nominal_position, double stiffness)
    : ForceElement<T>(model_instance),
      joint_index_(joint_index),
      nominal_position_(nominal_position),
      stiffness_(stiffness) {
  DRAKE_THROW_UNLESS(stiffness >= 0);
}

template <typename T>
PrismaticSpring<T>::~PrismaticSpring() = default;

template <typename T>
const PrismaticJoint<T>& PrismaticSpring<T>::joint() const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  const PrismaticJoint<T>* joint = dynamic_cast<const PrismaticJoint<T>*>(
      &this->get_parent_tree().get_joint(joint_index_));
  DRAKE_DEMAND(joint != nullptr);
  return *joint;
}

template <typename T>
void PrismaticSpring<T>::DoCalcAndAddForceContribution(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>&,
    const internal::VelocityKinematicsCache<T>&,
    MultibodyForces<T>* forces) const {
  const T delta = nominal_position_ - joint().get_translation(context);
  const T force = stiffness_ * delta;
  joint().AddInForce(context, force, forces);
}

template <typename T>
T PrismaticSpring<T>::CalcPotentialEnergy(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>&) const {
  const T delta = nominal_position_ - joint().get_translation(context);

  return 0.5 * stiffness_ * delta * delta;
}

template <typename T>
T PrismaticSpring<T>::CalcConservativePower(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>&,
    const internal::VelocityKinematicsCache<T>&) const {
  // Since the potential energy is:
  //   V = 1/2⋅k⋅(x₀-x)²
  // The conservative power is defined as:
  //  Pc = -d(V)/dt = -[k⋅(x₀-x)⋅(-dx/dt)] = k⋅(x₀-x)⋅dx/dt
  // being positive when the potential energy decreases.
  const T delta = nominal_position_ - joint().get_translation(context);
  const T x_dot = joint().get_translation_rate(context);
  return stiffness_ * delta * x_dot;
}

template <typename T>
T PrismaticSpring<T>::CalcNonConservativePower(
    const systems::Context<T>&, const internal::PositionKinematicsCache<T>&,
    const internal::VelocityKinematicsCache<T>&) const {
  // Purely conservative spring
  return 0;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<ForceElement<ToScalar>>
PrismaticSpring<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>&) const {
  // N.B. We can't use std::make_unique here since this constructor is private
  // to std::make_unique.
  // N.B. We use the private constructor since it doesn't rely on a valid joint
  // reference, which might not be available during cloning.
  std::unique_ptr<PrismaticSpring<ToScalar>> spring_clone(
      new PrismaticSpring<ToScalar>(this->model_instance(), joint_index_,
                                    nominal_position(), stiffness()));
  return spring_clone;
}

template <typename T>
std::unique_ptr<ForceElement<double>> PrismaticSpring<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>> PrismaticSpring<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<symbolic::Expression>>
PrismaticSpring<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<T>> PrismaticSpring<T>::DoShallowClone() const {
  // N.B. We use the private constructor since joint() requires a MbT pointer.
  return std::unique_ptr<ForceElement<T>>(new PrismaticSpring<T>(
      this->model_instance(), joint_index_, nominal_position(), stiffness()));
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::PrismaticSpring);
