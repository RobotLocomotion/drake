#include "drake/examples/kuka_iiwa_arm/rigid_body_plant_mux.h"

#include <map>
#include <set>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
class RigidBodyPlantMux<T>::InputMux : public systems::LeafSystem<T> {
 public:
  explicit InputMux(const RigidBodyTree<T>& tree) {
    // Input ports will be declared when requested.
    this->DeclareOutputPort(systems::kVectorValued, tree.get_num_actuators());

    for (int i = 0; i < tree.get_num_actuators(); ++i) {
      const RigidBody<double>* body = tree.actuators[i].body_;
      actuator_map_[{tree.actuators[i].name_,
              body->get_model_instance_id()}] = i;
    }
  }

  const systems::SystemPortDescriptor<T>& AddInput(
      const std::vector<std::pair<std::string, int>>& actuators) {
    port_actuators_.push_back(std::vector<int>());
    auto& port_ids = port_actuators_.back();
    port_ids.reserve(actuators.size());

    for (const auto& actuator : actuators) {
      DRAKE_DEMAND(actuator_map_.count(actuator) > 0);
      const int id = actuator_map_[actuator];
      DRAKE_DEMAND(used_actuators_.count(id) == 0);
      // Don't allow two input ports to map to the same actuator.
      used_actuators_.insert(id);
      port_ids.push_back(id);
    }
    return this->DeclareInputPort(systems::kVectorValued, port_ids.size());
  }

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const {
    DRAKE_ASSERT(static_cast<int>(port_actuators_.size()) ==
                 this->get_num_input_ports());
    auto output_vec = this->GetMutableOutputVector(output, 0);
    output_vec.fill(0);
    for (size_t i = 0; i < port_actuators_.size(); ++i) {
      const auto input = this->EvalVectorInput(context, i);
      DRAKE_ASSERT(input->size() ==
                   static_cast<int>(port_actuators_[i].size()));
      for (size_t j = 0; j < port_actuators_[i].size(); ++j) {
        output_vec(port_actuators_[i][j]) = input->GetAtIndex(j);
        DRAKE_DEMAND(!(input->GetAtIndex(j) != input->GetAtIndex(j)));
      }
    }
  }

 private:
  std::map<std::pair<std::string, int>, int> actuator_map_;
  std::set<int> used_actuators_;
  std::vector<std::vector<int>> port_actuators_;
};

template <typename T>
class RigidBodyPlantMux<T>::OutputMux : public systems::LeafSystem<T> {
 public:
  explicit OutputMux(const RigidBodyTree<T>& tree)
      : num_positions_(tree.get_num_positions()) {
    // Output ports will be declared when requested.
    this->DeclareInputPort(
        systems::kVectorValued,
        tree.get_num_positions() + tree.get_num_velocities());

    for (int body_index = 0; body_index < tree.get_num_bodies();
         ++body_index) {
      const RigidBody<T>& body = tree.get_body(body_index);
      if (!body.has_parent_body()) { continue; }
      const int instance_id = body.get_model_instance_id();
      const DrakeJoint& joint = body.getJoint();
      const int position_start_index = body.get_position_start_index();
      const int num_positions = joint.get_num_positions();
      for (int i = 0; i < num_positions; ++i) {
        position_map_[{joint.get_position_name(i), instance_id}] =
            position_start_index + i;
      }

      const int velocity_start_index = body.get_velocity_start_index();
      const int num_velocities = joint.get_num_velocities();
      for (int i = 0; i < num_velocities; ++i) {
        velocity_map_[{joint.get_velocity_name(i), instance_id}] =
            velocity_start_index + i;
      }
    }
  }

  const systems::SystemPortDescriptor<T>& AddOutput(
      const std::vector<std::pair<std::string, int>>& positions,
      const std::vector<std::pair<std::string, int>>& velocities) {
    port_states_.push_back(std::vector<int>());
    auto& port_ids = port_states_.back();
    port_ids.reserve(positions.size() + velocities.size());

    for (const auto& position : positions) {
      DRAKE_DEMAND(position_map_.count(position) > 0);
      const int id = position_map_[position];
      port_ids.push_back(id);
    }
    for (const auto& velocity : velocities) {
      DRAKE_DEMAND(velocity_map_.count(velocity) > 0);
      const int id = velocity_map_[velocity] + num_positions_;
      port_ids.push_back(id);
    }
    return this->DeclareOutputPort(systems::kVectorValued, port_ids.size());
  }

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const {
    DRAKE_ASSERT(static_cast<int>(port_states_.size()) ==
                 this->get_num_output_ports());
    const auto input_vec = this->EvalVectorInput(context, 0);
    for (size_t i = 0; i < port_states_.size(); ++i) {
      auto output_vec = this->GetMutableOutputVector(output, i);
      for (size_t j = 0; j < port_states_[i].size(); ++j) {
        output_vec(j) = input_vec->GetAtIndex(port_states_[i][j]);
      }
    }
  }

 private:
  const int num_positions_;
  std::map<std::pair<std::string, int>, int> position_map_;
  std::map<std::pair<std::string, int>, int> velocity_map_;
  std::vector<std::vector<int>> port_states_;
};

template <typename T>
RigidBodyPlantMux<T>::RigidBodyPlantMux(const RigidBodyTree<T>& tree) {
  input_mux_ = std::make_unique<InputMux>(tree);
  output_mux_ = std::make_unique<OutputMux>(tree);
}

template <typename T>
RigidBodyPlantMux<T>::~RigidBodyPlantMux() {}

template <typename T>
const systems::SystemPortDescriptor<T>& RigidBodyPlantMux<T>::AddInput(
    const std::vector<std::pair<std::string, int>>& actuators) {
  DRAKE_DEMAND(static_cast<bool>(input_mux_));
  return input_mux_->AddInput(actuators);
}

template <typename T>
const systems::SystemPortDescriptor<T>& RigidBodyPlantMux<T>::AddOutput(
    const std::vector<std::pair<std::string, int>>& positions,
    const std::vector<std::pair<std::string, int>>& velocities) {
  DRAKE_DEMAND(static_cast<bool>(output_mux_));
  return output_mux_->AddOutput(positions, velocities);
}

template <typename T>
void RigidBodyPlantMux<T>::ConnectPlant(
    const systems::System<T>& plant, systems::DiagramBuilder<T>* builder) {
  DRAKE_DEMAND(static_cast<bool>(input_mux_));
  DRAKE_DEMAND(input_mux_->get_output_port(0).get_size() ==
               plant.get_input_port(0).get_size());
  DRAKE_DEMAND(static_cast<bool>(output_mux_));
  DRAKE_DEMAND(output_mux_->get_input_port(0).get_size() ==
               plant.get_output_port(0).get_size());

  auto input_mux = builder->template AddSystem(std::move(input_mux_));
  auto output_mux = builder->template AddSystem(std::move(output_mux_));
  builder->Connect(input_mux->get_output_port(0), plant.get_input_port(0));
  builder->Connect(plant.get_output_port(0), output_mux->get_input_port(0));
}

template class RigidBodyPlantMux<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
