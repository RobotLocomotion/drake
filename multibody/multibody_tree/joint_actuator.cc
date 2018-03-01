#include "drake/multibody/multibody_tree/joint_actuator.h"

#include "drake/common/default_scalars.h"
#include "drake/multibody/multibody_tree/joints/joint.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
JointActuator<T>::JointActuator(
    const std::string& name, const Joint<T>& joint)
    : name_(name), joint_index_(joint.index()) {}

template <typename T>
const Joint<T>& JointActuator<T>::joint() const {
  return this->get_parent_tree().get_joint(joint_index_);
}

template <typename T>
void JointActuator<T>::AddInOneForce(
    const systems::Context<T>& context,
    int joint_dof,
    const T& joint_tau,
    MultibodyForces<T>* forces) const {
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(0 <= joint_dof && joint_dof < joint().num_dofs());
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(this->get_parent_tree()));
  joint().AddInOneForce(context, joint_dof, joint_tau, forces);
}

template <typename T>
std::unique_ptr<JointActuator<double>>
JointActuator<T>::DoCloneToScalar(const MultibodyTree<double>&) const {
  return std::unique_ptr<JointActuator<double>>(
      new JointActuator<double>(name_, joint_index_));
}

template <typename T>
std::unique_ptr<JointActuator<AutoDiffXd>>
JointActuator<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>&) const {
  return std::unique_ptr<JointActuator<AutoDiffXd>>(
      new JointActuator<AutoDiffXd>(name_, joint_index_));
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::JointActuator)
