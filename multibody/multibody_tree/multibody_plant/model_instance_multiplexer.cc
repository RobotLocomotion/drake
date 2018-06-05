#include "drake/multibody/multibody_tree/multibody_plant/model_instance_multiplexer.h"

#include <functional>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

template <typename T>
ModelInstanceMultiplexer<T>::ModelInstanceMultiplexer(
    const MultibodyPlant<T>& plant)
    : plant_(plant) {

  for (int i = 0; i < plant_.num_model_instances(); ++i) {
    const ModelInstance<T>& instance = plant_.get_model_instance(i);
    const int model_dofs = instance.num_actuated_dofs();
    if (model_dofs != 0) {
      input_port_map_.push_back(
          this->DeclareVectorInputPort(
              systems::BasicVector<T>(model_dofs)).get_index());
    } else {
      input_port_map_.push_back(-1);
    }

    const int model_states =
        instance.num_positions() + instance.num_velocities();
    auto calc = [this, i](const systems::Context<T>& context,
                          systems::BasicVector<T>* result) {
      this->CopyStateOutput(i, context, result);
    };

    output_port_map_.push_back(
        this->DeclareVectorOutputPort(
            systems::BasicVector<T>(model_states), calc).get_index());
  }

  actuation_output_port_ = this->DeclareVectorOutputPort(
      systems::BasicVector<T>(plant_.num_actuated_dofs()),
      &ModelInstanceMultiplexer::CopyActuationInputs).get_index();

  state_input_port_ = this->DeclareVectorInputPort(
      systems::BasicVector<T>(plant_.num_multibody_states())).get_index();
}

template <typename T>
ModelInstanceMultiplexer<T>::~ModelInstanceMultiplexer() {}

template <typename T>
const systems::InputPortDescriptor<T>&
ModelInstanceMultiplexer<T>::get_actuation_input_port(
    int model_instance) const {
  DRAKE_DEMAND(model_instance < static_cast<int>(input_port_map_.size()));

  const int port_index = input_port_map_[model_instance];
  DRAKE_DEMAND(port_index >= 0);
  return this->get_input_port(port_index);
}

template <typename T>
const systems::OutputPort<T>&
ModelInstanceMultiplexer<T>::get_state_output_port(
    int model_instance) const {
  DRAKE_DEMAND(model_instance < static_cast<int>(output_port_map_.size()));
  return this->get_output_port(output_port_map_[model_instance]);
}

template <typename T>
optional<bool>
ModelInstanceMultiplexer<T>::DoHasDirectFeedthrough(
    int input_port, int output_port) const {
  if (input_port == state_input_port_ &&
      output_port == actuation_output_port_) {
    return false;
  } else {
    // This is one of the actuation input ports.  It's only direct
    // feedthrough to the actuation output port.
    if (output_port != actuation_output_port_) {
      return false;
    }
  }

  return true;
}

template <typename T>
void ModelInstanceMultiplexer<T>::CopyActuationInputs(
    const systems::Context<T>& context,
    systems::BasicVector<T>* output) const {
  VectorX<T> out(plant_.num_actuated_dofs());
  out.setZero();

  for (int i = 0; i < plant_.num_model_instances(); ++i) {
    if (input_port_map_[i] == -1) {
      continue;
    }

    auto actuation_input =
        this->EvalEigenVectorInput(context, input_port_map_[i]);
    plant_.get_model_instance(i).set_actuation_vector(actuation_input, &out);
  }

  output->set_value(out);
}

template <typename T>
void ModelInstanceMultiplexer<T>::CopyStateOutput(
    int model_instance_idx, const systems::Context<T>& context,
    systems::BasicVector<T>* output) const {
  auto state_input = this->EvalEigenVectorInput(context, state_input_port_);

  const ModelInstance<T>& model_instance =
      plant_.get_model_instance(model_instance_idx);
  VectorX<T> positions(model_instance.num_positions());
  model_instance.get_positions_from_array(
      state_input.head(plant_.num_positions()), &positions);

  VectorX<T> velocities(model_instance.num_velocities());
  model_instance.get_velocities_from_array(
      state_input.tail(plant_.num_velocities()), &velocities);

  output->get_mutable_value().head(positions.size()) = positions;
  output->get_mutable_value().tail(velocities.size()) = velocities;
}

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::multibody_plant::ModelInstanceMultiplexer)
