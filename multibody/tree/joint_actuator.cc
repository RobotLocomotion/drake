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

  // The continuous-time MultibodyPlant logic does not support implicit PD. The
  // plant's Finalize() call already checks for this condition and throws if a
  // PD controller exists during Finalize. However, take note that adding a
  // controller and removing it again before calling Finalize is valid -- the
  // only invalid condition is a PD controller on a finalized, continuous-time
  // plant. On the other hand, if set_controller_gains is called post-Finalize
  // to add a controller on a continuous-time plant, we need to reject that
  // ourselves; the plant won't know to check for it.
  const bool is_finalized = topology_.actuator_index_start >= 0;
  if (is_finalized) {
    // N.B. Calling is_state_discrete() on a non-finalized plant will segfault;
    // we must be careful to only call it inside of the if-finalized guard.
    const bool is_continuous = !this->get_parent_tree().is_state_discrete();
    if (is_continuous) {
      throw std::runtime_error(fmt::format(
          "Cannot set PD gains on the actuator named '{}'. This feature "
          "is only supported for discrete models.",
          name()));
    }
  }

  pd_controller_gains_ = gains;
}

template <typename T>
const Joint<T>& JointActuator<T>::joint() const {
  return this->get_parent_tree().get_joint(joint_index_);
}

template <typename T>
void JointActuator<T>::AddInOneForce(const systems::Context<T>& context,
                                     int joint_dof, const T& joint_tau,
                                     MultibodyForces<T>* forces) const {
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(0 <= joint_dof && joint_dof < num_inputs());
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(this->get_parent_tree()));
  joint().AddInOneForce(context, joint_dof, joint_tau, forces);
}

template <typename T>
void JointActuator<T>::set_actuation_vector(
    const Eigen::Ref<const VectorX<T>>& u_actuator,
    EigenPtr<VectorX<T>> u) const {
  DRAKE_THROW_UNLESS(u != nullptr);
  DRAKE_THROW_UNLESS(u->size() == this->get_parent_tree().num_actuated_dofs());
  DRAKE_THROW_UNLESS(u_actuator.size() == num_inputs());
  u->segment(topology_.actuator_index_start, num_inputs()) = u_actuator;
}

template <typename T>
int JointActuator<T>::input_start() const {
  if (topology_.actuator_index_start < 0) {
    throw std::runtime_error(
        "JointActuator::input_start() must be called after the MultibodyPlant "
        "is finalized.");
  }
  return topology_.actuator_index_start;
}

template <typename T>
int JointActuator<T>::num_inputs() const {
  if (topology_.actuator_index_start < 0) {
    throw std::runtime_error(
        "JointActuator::num_inputs() must be called after the MultibodyPlant "
        "is finalized.");
  }
  DRAKE_ASSERT(joint().num_velocities() == topology_.num_dofs);
  return joint().num_velocities();
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
    class ::drake::multibody::JointActuator);
