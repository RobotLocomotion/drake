#include "drake/systems/primitives/selector.h"

#include "absl/container/inlined_vector.h"

namespace drake {
namespace systems {

using OutputSelection = SelectorParams::OutputSelection;

template <typename T>
Selector<T>::Selector(SelectorParams params)
    : LeafSystem<T>(SystemTypeTag<Selector>{}), params_(std::move(params)) {
  // Validate input parameters and declare input ports.
  const int num_inputs = ssize(params_.input_sizes);
  DRAKE_THROW_UNLESS(params_.input_names.empty() ||
                     ssize(params_.input_names) == num_inputs);
  for (InputPortIndex i{0}; i < num_inputs; ++i) {
    const int input_size = params_.input_sizes[i];
    DRAKE_THROW_UNLESS(input_size >= 0);
    std::variant<std::string, UseDefaultName> name = kUseDefaultName;
    if (!params_.input_names.empty()) {
      name = params_.input_names[i];
    }
    this->DeclareVectorInputPort(std::move(name), input_size);
  }

  // Validate output parameters and declare output ports.
  const int num_outputs = ssize(params_.output_selections);
  DRAKE_THROW_UNLESS(params_.output_names.empty() ||
                     ssize(params_.output_names) == num_outputs);
  for (OutputPortIndex i{0}; i < num_outputs; ++i) {
    const std::vector<OutputSelection>& output_specs =
        params_.output_selections[i];
    const int output_size = ssize(output_specs);
    DRAKE_THROW_UNLESS(output_size >= 0);
    std::set<DependencyTicket> prerequisites_of_calc;
    for (const auto& [input_port_index, input_offset] : output_specs) {
      DRAKE_THROW_UNLESS(input_port_index >= 0);
      DRAKE_THROW_UNLESS(input_port_index < num_inputs);
      const int selected_input_size = params_.input_sizes[input_port_index];
      DRAKE_THROW_UNLESS(input_offset >= 0);
      DRAKE_THROW_UNLESS(input_offset < selected_input_size);
      prerequisites_of_calc.insert(
          this->input_port_ticket(InputPortIndex{input_port_index}));
    }
    if (output_size == 0) {
      prerequisites_of_calc.insert(this->nothing_ticket());
    }
    std::variant<std::string, UseDefaultName> name = kUseDefaultName;
    if (!params_.output_names.empty()) {
      name = params_.output_names[i];
    }
    this->DeclareVectorOutputPort(
        std::move(name), output_size,
        [this, i](const Context<T>& context, BasicVector<T>* output) {
          this->CalcOutput(context, OutputPortIndex{i}, output);
        },
        std::move(prerequisites_of_calc));
  }
}

template <typename T>
template <typename U>
Selector<T>::Selector(const Selector<U>& other) : Selector(other.params_) {}

template <typename T>
Selector<T>::~Selector() = default;

template <typename T>
void Selector<T>::CalcOutput(const Context<T>& context,
                             OutputPortIndex output_port_index,
                             BasicVector<T>* output) const {
  const std::vector<OutputSelection>& selections =
      params_.output_selections[output_port_index];
  absl::InlinedVector<const VectorX<T>*, 8> inputs(this->num_input_ports());
  for (int i = 0; i < ssize(selections); ++i) {
    const InputPortIndex n{selections[i].input_port_index};
    const int j = selections[i].input_offset;
    if (inputs[n] == nullptr) {
      inputs[n] = &(this->get_input_port(n).Eval(context));
    }
    (*output)[i] = (*inputs[n])[j];
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Selector);
