#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// Creates multiplexers to wrap the input/output ports of a
/// RigidBodyPlant to allow for multiple controllers to operate on
/// different parts of the plant.
template <typename T>
class RigidBodyPlantMux {
 public:
  explicit RigidBodyPlantMux(const RigidBodyTree<T>& tree);
  ~RigidBodyPlantMux();

  /// Creates a new vector valued input port where the values of the
  /// port correspond to the actuators from the rigid boty tree
  /// provided in @p actuator_names.
  const systems::SystemPortDescriptor<T>& AddInput(
      const std::vector<std::string>& actuator_names);

  /// Creates a new vector values output port where the values of the
  /// port correspond to the positions/velocities from the rigid body
  /// tree provided in @p position_names followed by @p
  /// velocity_names.
  const systems::SystemPortDescriptor<T>& AddOutput(
      const std::vector<std::string>& position_names,
      const std::vector<std::string>& velocity_names);

  /// Using @p builder, connect the ports previously created with
  /// AddInput and AddOutput to @p plant.  It is an error to call
  /// AddInput or AddOutput after ConnectPlant.
  void ConnectPlant(const systems::System<T>& plant,
                    systems::DiagramBuilder<T>* builder);

  RigidBodyPlantMux(const RigidBodyPlantMux&) = delete;
  RigidBodyPlantMux& operator=(const RigidBodyPlantMux&) = delete;

 private:
  class InputMux;
  class OutputMux;

  std::unique_ptr<InputMux> input_mux_;
  std::unique_ptr<OutputMux> output_mux_;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
