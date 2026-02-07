#include "drake/multibody/tree/joint_actuator.h"

#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
JointActuator<T>::JointActuator(const std::string& name, const Joint<T>& joint,
                                double effort_limit)
    : MultibodyElement<T>(joint.model_instance()),
      name_(name),
      joint_index_(joint.index()),
      effort_limit_(effort_limit) {
  if (effort_limit_ <= 0.0) {
    throw std::runtime_error("Effort limit must be strictly positive!");
  }
}

template <typename T>
JointActuator<T>::~JointActuator() = default;

template <typename T>
void JointActuator<T>::set_controller_gains(PdControllerGains gains) {
  DRAKE_THROW_UNLESS(std::isfinite(gains.p));
  DRAKE_THROW_UNLESS(std::isfinite(gains.d));
  DRAKE_THROW_UNLESS(gains.p >= 0.0);
  DRAKE_THROW_UNLESS(gains.d >= 0.0);
  if (gains.p == 0.0 && gains.d == 0) {
    pd_controller_gains_ = std::nullopt;
    return;
  }

  pd_controller_gains_ = gains;
}

template <typename T>
const Joint<T>& JointActuator<T>::joint() const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  return this->get_parent_tree().get_joint(joint_index_);
}

template <typename T>
void JointActuator<T>::AddInOneForce(const systems::Context<T>& context,
                                     int joint_dof, const T& joint_tau,
                                     MultibodyForces<T>* forces) const {
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(0 <= joint_dof && joint_dof < num_inputs());
  DRAKE_DEMAND(this->has_parent_tree());
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(this->get_parent_tree()));
  joint().AddInOneForce(context, joint_dof, joint_tau, forces);
}

template <typename T>
void JointActuator<T>::set_actuation_vector(
    const Eigen::Ref<const VectorX<T>>& u_actuator,
    EigenPtr<VectorX<T>> u) const {
  DRAKE_THROW_UNLESS(u != nullptr);
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  DRAKE_THROW_UNLESS(u->size() == this->get_parent_tree().num_actuated_dofs());
  DRAKE_THROW_UNLESS(u_actuator.size() == num_inputs());
  u->segment(input_start(), num_inputs()) = u_actuator;
}

template <typename T>
int JointActuator<T>::input_start() const {
  return actuator_dof_start_;
}

template <typename T>
int JointActuator<T>::num_inputs() const {
  return joint().num_velocities();
}

template <typename T>
std::unique_ptr<JointActuator<double>> JointActuator<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>&) const {
  return std::unique_ptr<JointActuator<double>>(new JointActuator<double>(
      name_, joint_index_, actuator_dof_start_, effort_limit_,
      default_rotor_inertia_, default_gear_ratio_));
}

template <typename T>
std::unique_ptr<JointActuator<AutoDiffXd>> JointActuator<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>&) const {
  return std::unique_ptr<JointActuator<AutoDiffXd>>(
      new JointActuator<AutoDiffXd>(name_, joint_index_, actuator_dof_start_,
                                    effort_limit_, default_rotor_inertia_,
                                    default_gear_ratio_));
}

template <typename T>
std::unique_ptr<JointActuator<symbolic::Expression>>
JointActuator<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>&) const {
  return std::unique_ptr<JointActuator<symbolic::Expression>>(
      new JointActuator<symbolic::Expression>(
          name_, joint_index_, actuator_dof_start_, effort_limit_,
          default_rotor_inertia_, default_gear_ratio_));
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::JointActuator);
