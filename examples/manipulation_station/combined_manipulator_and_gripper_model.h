#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"

namespace drake {

namespace systems {
template <typename T>
class DiagramBuilder;
}

namespace multibody {
template <typename T>
class MultibodyPlant;
}

namespace examples {
namespace manipulation_station {

/**
 * An abstract class for adding a combined manipulator/gripper model and its
 * controller to a Diagram and querying aspects of the model (e.g., the number
 * of manipulator generalized positions).
 */
template <typename T>
class CombinedManipulatorAndGripperModel {
 public:
  CombinedManipulatorAndGripperModel(multibody::MultibodyPlant<T>* plant) :
      plant_(plant) {}
  virtual ~CombinedManipulatorAndGripperModel() {}

  /// Gets the number of joints in the manipulator.
  virtual int num_manipulator_joints() const = 0;

  /// Gets the number of joints in the gripper.
  virtual int num_gripper_joints() const = 0;

  /// Gets the manipulator generalized positions.
  virtual VectorX<T> GetManipulatorPositions(
      const systems::Context<T>& diagram_context,
      const systems::Diagram<T>& diagram) const = 0;

  /// Gets the gripper generalized positions.
  virtual VectorX<T> GetGripperPositions(
      const systems::Context<T>& diagram_context,
      const systems::Diagram<T>& diagram) const = 0;

  /// Sets the manipulator generalized positions.
  virtual void SetManipulatorPositions(
    const systems::Context<T>& diagram_context,
    const Eigen::Ref<const VectorX<T>>& q,
    const systems::Diagram<T>& diagram,
    systems::State<T>* diagram_state) const = 0;

  /// Sets the gripper generalized positions to the default "open" position.
  virtual void SetGripperPositionsToDefaultOpen(
    const systems::Context<T>& diagram_context,
    const systems::Diagram<T>& diagram,
    systems::State<T>* diagram_state) const = 0;

  /// Sets the gripper generalized positions.
  virtual void SetGripperPositions(
    const systems::Context<T>& diagram_context,
    const Eigen::Ref<const VectorX<T>>& q,
    const systems::Diagram<T>& diagram,
    systems::State<T>* diagram_state) const = 0;

  /// Gets the manipulator generalized velocities.
  virtual VectorX<T> GetManipulatorVelocities(
      const systems::Context<T>& diagram_context,
      const systems::Diagram<T>& diagram) const = 0;

  /// Gets the gripper generalized velocities.
  virtual VectorX<T> GetGripperVelocities(
      const systems::Context<T>& diagram_context,
      const systems::Diagram<T>& diagram) const = 0;

  /// Sets the manipulator generalized velocities.
  virtual void SetManipulatorVelocities(
    const systems::Context<T>& diagram_context,
    const Eigen::Ref<const VectorX<T>>& v,
    const systems::Diagram<T>& diagram,
    systems::State<T>* diagram_state) const = 0;

  /// Sets the gripper generalized velocities.
  virtual void SetGripperVelocities(
    const systems::Context<T>& diagram_context,
    const Eigen::Ref<const VectorX<T>>& v,
    const systems::Diagram<T>& diagram,
    systems::State<T>* diagram_state) const = 0;

  /// This method adds the manipulator and gripper models to the internal plant.
  virtual void AddRobotModelToMultibodyPlant() = 0;

  /// This method builds the control diagram for the manipulator and gripper
  /// and adds it to the given builder for a Diagram.
  virtual void BuildControlDiagram(systems::DiagramBuilder<T>* builder) = 0;

  /// Gets a reference to the plant used for control.
  virtual const multibody::MultibodyPlant<T>& get_controller_plant() const = 0;

  /// Gets the model instance for the manipulator in the internal plant.
  virtual multibody::ModelInstanceIndex manipulator_model_instance() const = 0;

  /// Gets the model instance for the gripper in the internal plant.
  virtual multibody::ModelInstanceIndex gripper_model_instance() const = 0;

 protected:
  // The MultibodyPlant holding the robot model (and possibly other models as
  // well).
  multibody::MultibodyPlant<T>* plant_{nullptr};
};

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

