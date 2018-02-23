#include "drake/multibody/multibody_tree/joint_actuator.h"

#include "drake/common/default_scalars.h"
#include "drake/multibody/multibody_tree/joints/joint.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
JointActuator<T>::JointActuator(
    const std::string& name, const Joint<T>& joint)
    : name_(name), joint_index_(joint.get_index()) {}

template <typename T>
const Joint<T>& JointActuator<T>::joint() const {
  return this->get_parent_tree().get_joint(joint_index_);
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
