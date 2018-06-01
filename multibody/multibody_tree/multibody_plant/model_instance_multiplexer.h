#pragma once

#include <vector>

#include "drake/multibody/multibody_tree/multibody_plant/model_instance.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

/// This system provides separate input ports for the actuation
/// vectors of different model instances in a %MultibodyPlant,
/// combining all of the inputs into a single output port indended to
/// be connected to the actuation input port of a %MultibodyPlant.
/// Additionally, it provides an input port for the state vector from
/// %MultibodyPlant and an output port for the state of each model
/// instance.
template <typename T>
class ModelInstanceMultiplexer : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ModelInstanceMultiplexer)

  /// @param[in] plant A multibody plant.  This will be aliased
  /// internally and must remain valid for the lifetime of the object.
  explicit ModelInstanceMultiplexer(const MultibodyPlant<T>& plant);
  ~ModelInstanceMultiplexer();

  /// @returns the actuation input port for the model instance at index
  /// @p model_instance.
  const systems::InputPortDescriptor<T>& get_actuation_input_port(
      int model_instance) const;

  const systems::OutputPort<T>& get_actuation_output_port() const {
    return this->get_output_port(actuation_output_port_);
  }

  const systems::InputPortDescriptor<T>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

  /// @returns the state port for the model instance at index
  /// @p model_instance.
  const systems::OutputPort<T>& get_state_output_port(
      int model_instance) const;

 private:
  optional<bool> DoHasDirectFeedthrough(
      int input_port, int output_port) const final;

  void CopyActuationInputs(
      const systems::Context<T>& context,
      systems::BasicVector<T>* output) const;

  void CopyStateOutput(
      int model_instance_idx, const systems::Context<T>& context,
      systems::BasicVector<T>* output) const;

  const MultibodyPlant<T>& plant_;

  // Contains the input port index for the model instance at the
  // corresponding position in the vector of model instances passd
  // into the constructor (or -1 if that instance has no actuation).
  std::vector<int> input_port_map_;

  // Contains the output port index for the model instance at the
  // corresponding position in the vector of model instances passed
  // into the constructor.
  std::vector<int> output_port_map_;

  int state_input_port_{-1};
  int actuation_output_port_{-1};
};

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
