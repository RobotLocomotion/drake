#include "drake/multibody/tree/model_instance.h"

#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
void ModelInstance<T>::SetActuationInArray(
    const Eigen::Ref<const VectorX<T>>& u_instance,
    EigenPtr<VectorX<T>> u) const {
  DRAKE_DEMAND(u != nullptr);
  if (u->size() != this->get_parent_tree().num_actuated_dofs() ||
      u_instance.size() != num_actuated_dofs_) {
    throw std::logic_error("Passed in array(s) is not properly sized.");
  }

  int u_instance_offset = 0;
  for (const JointActuator<T>* actuator : joint_actuators_) {
    const int num_dofs = actuator->joint().num_velocities();
    actuator->set_actuation_vector(
        u_instance.segment(u_instance_offset, num_dofs), u);
    u_instance_offset += num_dofs;
    DRAKE_DEMAND(u_instance_offset <= u->size());
  }
}

template <typename T>
VectorX<T> ModelInstance<T>::GetPositionsFromArray(
    const Eigen::Ref<const VectorX<T>>& q_array) const {
  if (q_array.size() != this->get_parent_tree().num_positions())
    throw std::logic_error("Passed in array is not properly sized.");
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

template <class T>
void ModelInstance<T>::SetPositionsInArray(
    const Eigen::Ref<const VectorX<T>>& model_q,
    EigenPtr<VectorX<T>> q_array) const {
  DRAKE_DEMAND(q_array);
  if (q_array->size() != this->get_parent_tree().num_positions() ||
      model_q.size() != num_positions()) {
    throw std::logic_error("Passed in array(s) is not properly sized.");
  }
  int position_offset = 0;
  for (const Mobilizer<T>* mobilizer : mobilizers_) {
    const int mobilizer_positions = mobilizer->num_positions();
    q_array->segment(mobilizer->position_start_in_q(), mobilizer_positions) =
        model_q.segment(position_offset, mobilizer_positions);
    position_offset += mobilizer_positions;
    DRAKE_DEMAND(position_offset <= model_q.size());
  }
}

template <typename T>
VectorX<T> ModelInstance<T>::GetVelocitiesFromArray(
    const Eigen::Ref<const VectorX<T>>& v_array) const {
  if (v_array.size() != this->get_parent_tree().num_velocities())
    throw std::logic_error("Passed in array is not properly sized.");
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

template <class T>
void ModelInstance<T>::SetVelocitiesInArray(
    const Eigen::Ref<const VectorX<T>>& model_v,
    EigenPtr<VectorX<T>> v_array) const {
  DRAKE_DEMAND(v_array->size() == this->get_parent_tree().num_velocities());
  DRAKE_DEMAND(model_v.size() == num_velocities());
  int velocity_offset = 0;
  for (const Mobilizer<T>* mobilizer : mobilizers_) {
    const int mobilizer_velocities = mobilizer->num_velocities();
    v_array->segment(mobilizer->velocity_start_in_v(), mobilizer_velocities) =
        model_v.segment(velocity_offset, mobilizer_velocities);
    velocity_offset += mobilizer_velocities;
    DRAKE_DEMAND(velocity_offset <= model_v.size());
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::ModelInstance)
