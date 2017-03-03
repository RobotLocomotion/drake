#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/state_feedback_controller_base.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
class SimDiagramBuilder : public systems::DiagramBuilder<T> {
 public:
  struct InputPortIdLookup {
    int subsystem_input_idx{0};
    int diagram_input_idx{0};

    InputPortIdLookup(int sub_idx, int full_idx)
        : subsystem_input_idx(sub_idx), diagram_input_idx(full_idx) {}
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimDiagramBuilder)

  SimDiagramBuilder() {}

  void WireThingsTogether();

  systems::RigidBodyPlant<T>* AddPlant(std::unique_ptr<systems::RigidBodyPlant<T>> plant);

  systems::RigidBodyPlant<T>* AddPlant(std::unique_ptr<RigidBodyTree<T>> world_tree) {
    return AddPlant(std::make_unique<systems::RigidBodyPlant<T>>(std::move(world_tree)));
  }

  systems::StateFeedbackController<T>* AddController(const int instance_id, std::unique_ptr<systems::StateFeedbackController<T>> controller);

  void AddControllers(std::unordered_map<int, std::unique_ptr<systems::StateFeedbackController<T>>>& controllers);

  systems::StateFeedbackController<T>* get_controller(int instance_id) const {
    return controllers_.at(instance_id);
  }

  systems::RigidBodyPlant<T>* get_plant() const {
    return plant_;
  }

  /*
  void ExposePlantOutputPortState(int instance_id);

  void ExposePlantOutputPortFullState();

  const std::vector<InputPortIdLookup>& get_exposed_controller_input_index_pairs(int instance_id) const {
    return exposed_inputs_controller_.at(instance_id);
  }

  int get_exposed_plant_state_output_index(int instance_id) const {
    return exposed_outputs_state_.at(instance_id);
  }

  int get_plant_full_state_output_port_index() const {
    return exposed_outputs_state_.at(-1);
  }
  */

 private:
  systems::RigidBodyPlant<T>* plant_{nullptr};
  std::unordered_map<int, systems::StateFeedbackController<T>*> controllers_;

  // instance id -> output id of the resulting diagram.
  //std::unordered_map<int, int> exposed_outputs_state_;

  // instance id -> vector of InputPortIdLookup
  //std::unordered_map<int, std::vector<InputPortIdLookup>> exposed_inputs_controller_;

  //int num_exposed_output_ports_{0};
  //int num_exposed_input_ports_{0};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
