#pragma once

#include <memory>
#include <unordered_map>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace util {

/**
 * A wrapper class around DiagramBuilder that facilitates diagram building for
 * controlled simulation. This class provides three utilities: adding /
 * accessing RigidBodyPlant, StateFeedbackControllerInterface and
 * DrakeVisualizer.
 * Access to a mutable DiagramBuilder is provided by get_mutable_builder().
 */
template <typename T>
class DRAKE_DEPRECATED("2020-05-01",
    "The attic/manipulation/util package is being removed.")
SimDiagramBuilder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimDiagramBuilder)

  SimDiagramBuilder() {}

  /**
   * Builds a Diagram in two steps: connects all the
   * StateFeedbackControllerInterface
   * added using AddController() with the RigidBodyPlant added using
   * AddPlant(), and connects the DrakeVisualizer if available with the plant.
   * Then calls DiagramBuilder's Build() method and returns the resulting
   * diagram. Must be called after AddPlant().
   */
  std::unique_ptr<systems::Diagram<T>> Build() {
    ConnectControllersAndVisualizer();
    return builder_.Build();
  }

  /**
   * Builds a Diagram in two steps: connects all the
   * StateFeedbackControllerInterface
   * added using AddController() with the RigidBodyPlant added using
   * AddPlant(), and connects the DrakeVisualizer if available with the plant.
   * Then calls DiagramBuilder's BuildInto() method. Must be called after
   * AddPlant().
   * @param[out] target Pointer to the resulting diagram.
   */
  void BuildInto(systems::Diagram<T>* target) {
    ConnectControllersAndVisualizer();
    builder_.BuildInto(target);
  }

  /**
   * Adds a RigidBodyPlant. Can be called at most once.
   * @param plant unique pointer to the RigidBodyPlant. Ownership will be
   * transferred.
   * @return Pointer to the added plant.
   */
  systems::RigidBodyPlant<T>* AddPlant(
      std::unique_ptr<systems::RigidBodyPlant<T>> plant);

  /**
   * Adds a RigidBodyPlant. Can be called at most once.
   * @param plant unique pointer to a RigidBodyTree that is used to construct a
   * RigidBodyPlant. Ownership will be transferred.
   * @return Pointer to the added plant.
   */
  systems::RigidBodyPlant<T>* AddPlant(
      std::unique_ptr<RigidBodyTree<T>> world_tree);

  /**
   * Adds a DrakeVisualizer. Can be called at most once. Must be called after
   * AddPlant().
   * @param lcm Pointer to a lcm interface.
   * @return Pointer to the added visualizer.
   */
  systems::DrakeVisualizer* AddVisualizer(drake::lcm::DrakeLcmInterface* lcm);

  /**
   * Adds a controller of type ControllerType, which must be derived from
   * StateFeedbackControllerInterface.
   * @param instance_id Identifier for the model instance in the RigidBodyPlant
   * to be controlled by the added controller. Each model instance can have at
   * most one controller.
   * @param controller Unique pointer to the controller. Ownership will be
   * transferred.
   * @return Pointer to the added controller.
   */
  template <class ControllerType>
  ControllerType* AddController(int instance_id,
                                std::unique_ptr<ControllerType> controller) {
    DRAKE_DEMAND(controllers_.find(instance_id) == controllers_.end());
    DRAKE_DEMAND(
        dynamic_cast<
            systems::controllers::StateFeedbackControllerInterface<T>*>(
            controller.get()) != nullptr);

    ControllerType* controller_ptr =
        builder_.template AddSystem<ControllerType>(std::move(controller));
    controllers_.emplace(instance_id, controller_ptr);

    return controller_ptr;
  }

  /**
   * Adds a controller of type ControllerType, which must be derived from
   * StateFeedbackControllerInterface<T>. Assumes the RigidBodyPlant only has
   * one model instance. Can be called at most once.
   * @param controller Unique pointer to the controller. Ownership will be
   * transferred.
   * @return Pointer to the added controller.
   */
  template <class ControllerType>
  ControllerType* AddController(std::unique_ptr<ControllerType> controller) {
    return AddController(RigidBodyTreeConstants::kFirstNonWorldModelInstanceId,
                         std::move(controller));
  }

  /**
   * Adds a controller of type ControllerType, which must be derived from
   * StateFeedbackControllerInterface<T>.
   * @param instance_id Identifier for the model instance in the RigidBodyPlant
   * to be controlled by the added controller. Each model instance can have at
   * most one controller.
   * @param args Arguments used to make a new ControllerType instance.
   * @return Pointer to the added controller.
   */
  template <class ControllerType, typename... Args>
  ControllerType* AddController(int instance_id, Args&&... args) {
    return AddController(instance_id, std::make_unique<ControllerType>(
                                          std::forward<Args>(args)...));
  }

  /**
   * Returns a StateFeedbackControllerInterface pointer to the controller for
   * model
   * instance @p instance_id.
   */
  systems::controllers::StateFeedbackControllerInterface<T>* get_controller(
      int instance_id) const {
    return controllers_.at(instance_id);
  }

  /**
   * Returns a StateFeedbackControllerInterface pointer to the controller.
   * Assumes the RigidBodyPlant only has one model instance.
   */
  systems::controllers::StateFeedbackControllerInterface<T>* get_controller()
      const {
    return get_controller(
        RigidBodyTreeConstants::kFirstNonWorldModelInstanceId);
  }

  /**
   * Returns a pointer to the RigidBodyPlant.
   */
  systems::RigidBodyPlant<T>* get_plant() const { return plant_; }

  /**
   * Returns a pointer to the underlying DiagramBuilder.
   */
  systems::DiagramBuilder<T>* get_mutable_builder() { return &builder_; }

  /**
   * Returns a pointer to the DrakeVisualizer.
   */
  systems::DrakeVisualizer* get_visualizer() { return visualizer_; }

 private:
  // Connects all the controllers' torque output to plant's torque input, and
  // plant's state output to controllers' state input.
  // Connects the whole plant's state output to the visualizer.
  void ConnectControllersAndVisualizer();

  // The underlying DiagramBuilder.
  systems::DiagramBuilder<T> builder_;

  // Pointer to the added RigidBodyPlant.
  systems::RigidBodyPlant<T>* plant_{nullptr};

  // A map from instance id to pointers to the added controllers.
  std::unordered_map<int,
                     systems::controllers::StateFeedbackControllerInterface<T>*>
      controllers_;

  // Pointer to the added DrakeVisualizer.
  systems::DrakeVisualizer* visualizer_{nullptr};
};

}  // namespace util
}  // namespace manipulation
}  // namespace drake
