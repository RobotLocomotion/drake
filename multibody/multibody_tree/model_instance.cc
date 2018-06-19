#include "drake/multibody/multibody_tree/model_instance.h"

#include "drake/common/default_scalars.h"
#include "drake/multibody/multibody_tree/joints/joint.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
void ModelInstance<T>::set_actuation_vector(
    const Eigen::Ref<const VectorX<T>>& u_instance,
    EigenPtr<VectorX<T>> u) const {
  DRAKE_DEMAND(u != nullptr);
  DRAKE_DEMAND(u->size() == this->get_parent_tree().num_actuated_dofs());
  DRAKE_DEMAND(u_instance.size() == num_actuated_dofs_);
  int u_instance_offset = 0;
  for (const JointActuator<T>* actuator : joint_actuators_) {
    const int num_dofs = actuator->joint().num_dofs();
    actuator->set_actuation_vector(
        u_instance.segment(u_instance_offset, num_dofs), u);
    u_instance_offset += num_dofs;
    DRAKE_DEMAND(u_instance_offset <= u->size());
  }
}

template <typename T>
VectorX<T> ModelInstance<T>::get_positions_from_array(
    const Eigen::Ref<const VectorX<T>>& q_array) const {
  DRAKE_DEMAND(
      q_array.size() == this->get_parent_tree().num_positions());
  VectorX<T> positions(num_positions_);
  int position_offset = 0;
  for (const Mobilizer<T>* mobilizer : mobilizers_) {
    const int mobilizer_positions = mobilizer->num_positions();
    positions.segment(position_offset, mobilizer_positions) =
        mobilizer->get_positions_from_array(q_array);
    position_offset += mobilizer_positions;
    DRAKE_DEMAND(position_offset <= positions.size());
  }
  return positions;
}

template <typename T>
VectorX<T> ModelInstance<T>::get_velocities_from_array(
    const Eigen::Ref<const VectorX<T>>& v_array) const {
  DRAKE_DEMAND(
      v_array.size() == this->get_parent_tree().num_velocities());
  VectorX<T> velocities(num_velocities_);
  int velocity_offset = 0;
  for (const Mobilizer<T>* mobilizer : mobilizers_) {
    const int mobilizer_velocities = mobilizer->num_velocities();
    velocities.segment(velocity_offset, mobilizer_velocities) =
        mobilizer->get_velocities_from_array(v_array);
    velocity_offset += mobilizer_velocities;
    DRAKE_DEMAND(velocity_offset <= velocities.size());
  }
  return velocities;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::ModelInstance)
