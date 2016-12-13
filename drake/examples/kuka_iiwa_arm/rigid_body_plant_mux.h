#pragma once

#include <memory>
#include <string>
#include <utility>
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

  // TODO(liangfok) Generalize to support multi-DOF actuators once
  // #4153 is resolved.

  /// Creates a new vector valued input port where the values of the
  /// port correspond to the actuators from the RigidBodyTree provided
  /// in @p actuators which consists of {name, model_instance_id}
  /// pairs.
  const systems::SystemPortDescriptor<T>& AddInput(
      const std::vector<std::pair<std::string, int>>& actuators);

  /// Creates a new vector valued output port where the values of the
  /// port correspond to the positions/velocities from the
  /// RigidBodyTree provided in @p positions followed by @p velocities
  /// which consist of {name, model_instance_id} pairs.
  const systems::SystemPortDescriptor<T>& AddOutput(
      const std::vector<std::pair<std::string, int>>& positions,
      const std::vector<std::pair<std::string, int>>& velocities);

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
