#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/controllers/state_feedback_controller_base.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
class SimDiagramBuilder : public systems::DiagramBuilder<T> {
 public:
  struct Connection {
    const systems::OutputPortDescriptor<T>& output;
    const systems::InputPortDescriptor<T>& input;

    Connection(const systems::OutputPortDescriptor<T>& out,
               const systems::InputPortDescriptor<T>& in)
        : output(out), input(in) {}
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimDiagramBuilder)

  SimDiagramBuilder() {}

  void WireThingsTogether();

  void ConnectAll(const std::vector<Connection>& connections) {
    for (const auto& connection : connections) {
      this->Connect(connection.output, connection.input);
    }
  }

  systems::RigidBodyPlant<T>* AddPlant(
      std::unique_ptr<systems::RigidBodyPlant<T>> plant);

  systems::RigidBodyPlant<T>* AddPlant(
      std::unique_ptr<RigidBodyTree<T>> world_tree);

  systems::DrakeVisualizer* AddVisualizer(drake::lcm::DrakeLcmInterface* lcm);

  systems::StateFeedbackController<T>* AddController(
      const int instance_id,
      std::unique_ptr<systems::StateFeedbackController<T>> controller);

  systems::StateFeedbackController<T>* AddController(
      std::unique_ptr<systems::StateFeedbackController<T>> controller) {
    return AddController(RigidBodyTreeConstants::kFirstNonWorldModelInstanceId,
                         std::move(controller));
  }

  systems::StateFeedbackController<T>* get_controller(int instance_id) const {
    return controllers_.at(instance_id);
  }

  systems::StateFeedbackController<T>* get_controller() const {
    return get_controller(
        RigidBodyTreeConstants::kFirstNonWorldModelInstanceId);
  }

  systems::RigidBodyPlant<T>* get_plant() const { return plant_; }

 private:
  systems::RigidBodyPlant<T>* plant_{nullptr};
  std::unordered_map<int, systems::StateFeedbackController<T>*> controllers_;

  systems::DrakeVisualizer* visualizer_{nullptr};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
