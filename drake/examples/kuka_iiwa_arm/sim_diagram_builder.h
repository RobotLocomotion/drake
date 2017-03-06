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
class SimDiagramBuilder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimDiagramBuilder)

  SimDiagramBuilder() {}

  std::unique_ptr<systems::Diagram<T>> Build() {
    WireThingsTogether();
    return builder_.Build();
  }

  void BuildInto(systems::Diagram<T>* target) {
    WireThingsTogether();
    builder_.BuildInto(target);
  }

  systems::RigidBodyPlant<T>* AddPlant(
      std::unique_ptr<systems::RigidBodyPlant<T>> plant);

  systems::RigidBodyPlant<T>* AddPlant(
      std::unique_ptr<RigidBodyTree<T>> world_tree);

  systems::DrakeVisualizer* AddVisualizer(drake::lcm::DrakeLcmInterface* lcm);

  template <class ControllerType>
  ControllerType* AddController(int instance_id,
                                std::unique_ptr<ControllerType> controller) {
    DRAKE_DEMAND(controllers_.find(instance_id) == controllers_.end());
    DRAKE_DEMAND(dynamic_cast<systems::StateFeedbackController<T>*>(
                     controller.get()) != nullptr);

    ControllerType* controller_ptr =
        builder_.template AddSystem<ControllerType>(std::move(controller));
    controllers_.emplace(instance_id, controller_ptr);

    return controller_ptr;
  }

  template <class ControllerType, typename... Args>
  ControllerType* AddController(int instance_id, Args&&... args) {
    return AddController(instance_id, std::make_unique<ControllerType>(
                                          std::forward<Args>(args)...));
  }

  systems::StateFeedbackController<T>* get_controller(int instance_id) const {
    return controllers_.at(instance_id);
  }

  systems::StateFeedbackController<T>* get_controller() const {
    return get_controller(
        RigidBodyTreeConstants::kFirstNonWorldModelInstanceId);
  }

  systems::RigidBodyPlant<T>* get_plant() const { return plant_; }

  systems::DiagramBuilder<T>* get_mutable_builder() { return &builder_; }

 private:
  void WireThingsTogether();

  systems::DiagramBuilder<T> builder_;

  systems::RigidBodyPlant<T>* plant_{nullptr};
  std::unordered_map<int, systems::StateFeedbackController<T>*> controllers_;

  systems::DrakeVisualizer* visualizer_{nullptr};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
