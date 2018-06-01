#include "drake/multibody/multibody_tree/multibody_plant/model_instance.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

template <typename T>
ModelInstance<T>::ModelInstance(
    const std::vector<BodyIndex>& bodies,
    const std::vector<JointActuatorIndex>& joint_actuators)
    : bodies_(bodies),
      joint_actuators_(joint_actuators) {
}

template <typename T>
ModelInstance<T>::~ModelInstance() {}

template <typename T>
void ModelInstance<T>::Finalize(const MultibodyTree<T>& tree) {
  if (!tree.topology_is_valid()) {
    throw std::logic_error("The MultibodyTree must be finalized before "
                           "calling ModelInstance::Finalize().");
  }

  num_tree_positions_ = tree.num_positions();
  num_tree_velocities_ = tree.num_velocities();
  num_tree_actuated_dofs_ = tree.num_actuated_dofs();

  const MultibodyTreeTopology& topology = tree.get_topology();
  for (const BodyIndex body_index : bodies_) {
    const Body<T>& body = tree.get_body(body_index);
    const BodyNodeTopology& node_topology =
        topology.get_body_node(body.node_index());
    for (int i = node_topology.mobilizer_positions_start;
         i < node_topology.mobilizer_positions_start +
             node_topology.num_mobilizer_positions; ++i) {
      position_map_.push_back(i);
      ++num_positions_;
    }
    for (int i = node_topology.mobilizer_velocities_start_in_v;
         i < node_topology.mobilizer_velocities_start_in_v +
             node_topology.num_mobilizer_velocities; ++i) {
      velocity_map_.push_back(i);
      ++num_velocities_;
    }
  }


  for (const JointActuatorIndex actuator_index : joint_actuators_) {
    const JointActuatorTopology& actuator_topology =
        topology.get_joint_actuator(actuator_index);
    for (int i = actuator_topology.actuator_index_start;
         i < actuator_topology.actuator_index_start +
             actuator_topology.num_dofs; ++i) {
      actuator_map_.push_back(i);
      ++num_actuated_dofs_;
    }
  }
}

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::multibody_plant::ModelInstance)
