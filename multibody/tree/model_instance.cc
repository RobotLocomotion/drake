#include "drake/multibody/tree/model_instance.h"

#include <utility>

#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
ModelInstance<T>::ModelInstance(ModelInstanceIndex index, std::string name)
    : MultibodyElement<T>(/* model_instance = */ index, /* index = */ index) {
  set_name(std::move(name));
}

template <typename T>
ModelInstance<T>::~ModelInstance() = default;

template <typename T>
void ModelInstance<T>::set_name(std::string name) {
  DRAKE_THROW_UNLESS(!name.empty());
  name_ = std::move(name);
}

template <typename T>
std::vector<JointActuatorIndex> ModelInstance<T>::GetJointActuatorIndices()
    const {
  std::vector<JointActuatorIndex> instance_actuator_indexes;
  instance_actuator_indexes.reserve(joint_actuators_.size());
  for (const JointActuator<T>* actuator : joint_actuators_) {
    instance_actuator_indexes.push_back(actuator->index());
  }
  return instance_actuator_indexes;
}

template <typename T>
std::vector<JointIndex> ModelInstance<T>::GetActuatedJointIndices() const {
  std::vector<JointIndex> instance_actuated_joint_indexes;
  instance_actuated_joint_indexes.reserve(joint_actuators_.size());
  for (const JointActuator<T>* actuator : joint_actuators_) {
    instance_actuated_joint_indexes.push_back(actuator->joint().index());
  }
  return instance_actuated_joint_indexes;
}

template <typename T>
VectorX<T> ModelInstance<T>::GetActuationFromArray(
    const Eigen::Ref<const VectorX<T>>& u) const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  if (u.size() != this->get_parent_tree().num_actuated_dofs())
    throw std::logic_error("Passed in array is not properly sized.");
  VectorX<T> u_instance(num_actuated_dofs_);
  int u_instance_offset = 0;
  for (const JointActuator<T>* actuator : joint_actuators_) {
    const int num_dofs = actuator->joint().num_velocities();
    u_instance.segment(u_instance_offset, num_dofs) =
        actuator->get_actuation_vector(u);
    u_instance_offset += num_dofs;
    DRAKE_DEMAND(u_instance_offset <= u.size());
  }

  return u_instance;
}

template <typename T>
void ModelInstance<T>::SetActuationInArray(
    const Eigen::Ref<const VectorX<T>>& u_instance,
    EigenPtr<VectorX<T>> u) const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
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
  VectorX<T> positions(num_positions_);
  GetPositionsFromArray(q_array, &positions);

  return positions;
}

template <typename T>
void ModelInstance<T>::GetPositionsFromArray(
    const Eigen::Ref<const VectorX<T>>& q, EigenPtr<VectorX<T>> q_out) const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  DRAKE_DEMAND(q_out != nullptr);
  if (q.size() != this->get_parent_tree().num_positions())
    throw std::logic_error("Passed in array is not properly sized.");
  if (q_out->size() != num_positions_)
    throw std::logic_error("Output array is not properly sized.");
  int position_offset = 0;
  for (const Mobilizer<T>* mobilizer : mobilizers_) {
    const int mobilizer_positions = mobilizer->num_positions();
    q_out->segment(position_offset, mobilizer_positions) =
        mobilizer->get_positions_from_array(q);
    position_offset += mobilizer_positions;
    DRAKE_DEMAND(position_offset <= q_out->size());
  }
}

template <class T>
void ModelInstance<T>::SetPositionsInArray(
    const Eigen::Ref<const VectorX<T>>& model_q,
    EigenPtr<VectorX<T>> q_array) const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  DRAKE_DEMAND(q_array != nullptr);
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
  VectorX<T> velocities(num_velocities_);
  GetVelocitiesFromArray(v_array, &velocities);

  return velocities;
}

template <typename T>
void ModelInstance<T>::GetVelocitiesFromArray(
    const Eigen::Ref<const VectorX<T>>& v, EigenPtr<VectorX<T>> v_out) const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  DRAKE_DEMAND(v_out != nullptr);
  if (v.size() != this->get_parent_tree().num_velocities())
    throw std::logic_error("Passed in array is not properly sized.");
  if (v_out->size() != num_velocities_)
    throw std::logic_error("Output array is not properly sized.");
  int velocity_offset = 0;
  for (const Mobilizer<T>* mobilizer : mobilizers_) {
    const int mobilizer_velocities = mobilizer->num_velocities();
    v_out->segment(velocity_offset, mobilizer_velocities) =
        mobilizer->get_velocities_from_array(v);
    velocity_offset += mobilizer_velocities;
    DRAKE_DEMAND(velocity_offset <= v_out->size());
  }
}

template <class T>
void ModelInstance<T>::SetVelocitiesInArray(
    const Eigen::Ref<const VectorX<T>>& model_v,
    EigenPtr<VectorX<T>> v_array) const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  DRAKE_DEMAND(v_array != nullptr);
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
    class ::drake::multibody::internal::ModelInstance);
