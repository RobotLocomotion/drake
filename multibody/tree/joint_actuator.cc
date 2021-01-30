#include "drake/multibody/tree/joint_actuator.h"

#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
JointActuator<T>::JointActuator(const std::string& name, const Joint<T>& joint,
                                double effort_limit)
    : MultibodyElement<JointActuator, T, JointActuatorIndex>(
          joint.model_instance()),
      name_(name),
      joint_index_(joint.index()),
      effort_limit_(effort_limit) {
  if (effort_limit_ <= 0.0) {
    throw std::runtime_error("Effort limit must be strictly positive!");
  }
}

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
  DRAKE_DEMAND(0 <= joint_dof && joint_dof < joint().num_velocities());
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(this->get_parent_tree()));
  joint().AddInOneForce(context, joint_dof, joint_tau, forces);
}

template <typename T>
void JointActuator<T>::set_actuation_vector(
    const Eigen::Ref<const VectorX<T>>& u_instance,
    EigenPtr<VectorX<T>> u) const {
  DRAKE_DEMAND(u != nullptr);
  DRAKE_DEMAND(u->size() == this->get_parent_tree().num_actuated_dofs());
  DRAKE_DEMAND(u_instance.size() == joint().num_velocities());
  u->segment(topology_.actuator_index_start, joint().num_velocities()) =
      u_instance;
}

template <typename T>
void JointActuator<T>::DoSetTopology(
    const internal::MultibodyTreeTopology& mbt_topology) {
  topology_ = mbt_topology.get_joint_actuator(this->index());
}

template <typename T>
std::unique_ptr<JointActuator<double>> JointActuator<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>&) const {
  return std::unique_ptr<JointActuator<double>>(
      new JointActuator<double>(name_, joint_index_, effort_limit_,
                                default_rotor_inertia_, default_gear_ratio_));
}

template <typename T>
std::unique_ptr<JointActuator<AutoDiffXd>> JointActuator<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>&) const {
  return std::unique_ptr<JointActuator<AutoDiffXd>>(
      new JointActuator<AutoDiffXd>(name_, joint_index_, effort_limit_,
                                    default_rotor_inertia_,
                                    default_gear_ratio_));
}

template <typename T>
std::unique_ptr<JointActuator<symbolic::Expression>>
JointActuator<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>&) const {
  return std::unique_ptr<JointActuator<symbolic::Expression>>(
      new JointActuator<symbolic::Expression>(
          name_, joint_index_, effort_limit_, default_rotor_inertia_,
          default_gear_ratio_));
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::JointActuator)
