#include "drake/systems/primitives/selector.h"

#include "absl/container/inlined_vector.h"

namespace drake {
namespace systems {
namespace {

using OutputSelection = SelectorParams::OutputSelection;

struct CompiledProgram {
  struct Element {
    auto operator<=>(const Element&) const = default;
    int input_offset{};
    int output_offset{};
  };

  // These InlinedVector sizes are chosen to keep the program smallish but avoid
  // pointer chasing for very small outputs or outputs with only a single input.
  using PerOutput = absl::InlinedVector<
      std::pair<InputPortIndex, absl::InlinedVector<Element, 5>>, 1>;
  static_assert(sizeof(PerOutput) == 64);

  // The outer vector is indexed by OutputPortIndex.
  // The next inner vector is like a map<InputPortIndex, ...>.
  // The values in the map are the offsets within those two ports.
  std::vector<PerOutput> outputs;
};

}  // namespace

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

  // Prepare the data for our CompiledProgram.
  using Element = CompiledProgram::Element;
  std::vector<std::map<InputPortIndex, std::vector<Element>>> compiled_outputs;
  compiled_outputs.resize(num_outputs);
  for (OutputPortIndex i{0}; i < num_outputs; ++i) {
    const std::vector<OutputSelection>& output_specs =
        params_.output_selections[i];
    int output_offset = 0;
    for (const auto& [input_port_index, input_offset] : output_specs) {
      compiled_outputs[i][InputPortIndex{input_port_index}].push_back(Element{
          .input_offset = input_offset,
          .output_offset = output_offset,
      });
      ++output_offset;
    }
    for (auto& [_, elements] : compiled_outputs[i]) {
      std::sort(elements.begin(), elements.end());
    }
  }

  // Pack the data into a CompiledProgram.
  auto program = std::make_shared<CompiledProgram>();
  program->outputs.resize(num_outputs);
  for (OutputPortIndex i{0}; i < num_outputs; ++i) {
    program->outputs[i].reserve(compiled_outputs[i].size());
    for (const auto& [input_port_index, elements] : compiled_outputs[i]) {
      program->outputs[i].push_back(CompiledProgram::PerOutput::value_type{
          input_port_index, {elements.begin(), elements.end()}});
    }
  }
  compiled_program_ = std::move(program);
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
  const auto& compiled_program =
      *static_cast<const CompiledProgram*>(compiled_program_.get());
  for (const auto& [input_port_index, elements] :
       compiled_program.outputs[output_port_index]) {
    const VectorX<T>& input =
        this->get_input_port(input_port_index).Eval(context);
    for (const auto& element : elements) {
      (*output)[element.output_offset] = input[element.input_offset];
    }
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Selector);
